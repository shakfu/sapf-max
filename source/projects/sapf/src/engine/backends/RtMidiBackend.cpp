//    SAPF - Sound As Pure Form
//    Copyright (C) 2019 James McCartney
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "sapf/backends/RtMidiBackend.hpp"

#include "MidiRouter.hpp"
#include "ErrorCodes.hpp"
#include "RtMidi.h"
#include <cstdio>
#include <algorithm>
#include <memory>

namespace {

class RtMidiBackendImpl : public MidiBackend
{
public:
	RtMidiBackendImpl();
	~RtMidiBackendImpl() override;

	int initialize(int numIn, int numOut) override;
	void cleanup() override;
	void restart() override;
	void listDevices() override;
	int connectInput(int uid, int portIndex) override;
	int disconnectInput(int uid, int portIndex) override;
	void sendMessage(int port, int destIndex, const std::vector<unsigned char>& message, float latencySeconds) override;

private:
	static void midiCallback(double timeStamp, std::vector<unsigned char>* message, void* userData);

	std::vector<std::unique_ptr<RtMidiIn>> midiIn_;
	std::vector<std::unique_ptr<RtMidiOut>> midiOut_;
	std::vector<int> connectedPorts_; // Which port index each input is connected to (-1 if none)
	bool initialized_ = false;
};

struct CallbackData {
	RtMidiBackendImpl* backend;
	int portIndex;
};

std::vector<std::unique_ptr<CallbackData>> gCallbackData;

RtMidiBackendImpl::RtMidiBackendImpl()
{
}

RtMidiBackendImpl::~RtMidiBackendImpl()
{
	cleanup();
}

void RtMidiBackendImpl::midiCallback(double /*timeStamp*/, std::vector<unsigned char>* message, void* userData)
{
	if (!message || message->empty()) return;
	CallbackData* data = static_cast<CallbackData*>(userData);
	getMidiRouter().handleIncomingMessage(data->portIndex, message->data(), (int)message->size());
}

int RtMidiBackendImpl::initialize(int numIn, int numOut)
{
	cleanup();

	getMidiRouter().resetState();

	numIn = std::clamp(numIn, 1, kMaxMidiPorts);
	numOut = std::clamp(numOut, 1, kMaxMidiPorts);

	try {
		// Create input ports
		for (int i = 0; i < numIn; ++i) {
			auto in = std::make_unique<RtMidiIn>();
			in->ignoreTypes(false, false, false); // Don't ignore any message types
			midiIn_.push_back(std::move(in));
			connectedPorts_.push_back(-1);
		}

		// Create output ports
		for (int i = 0; i < numOut; ++i) {
			auto out = std::make_unique<RtMidiOut>();
			midiOut_.push_back(std::move(out));
		}

		initialized_ = true;
		listDevices();

		return errNone;
	} catch (const RtMidiError& error) {
		fprintf(stderr, "RtMidi error: %s\n", error.getMessage().c_str());
		cleanup();
		return errFailed;
	}
}

void RtMidiBackendImpl::cleanup()
{
	gCallbackData.clear();
	midiIn_.clear();
	midiOut_.clear();
	connectedPorts_.clear();
	initialized_ = false;
}

void RtMidiBackendImpl::restart()
{
	// RtMidi doesn't have a restart API, but we can reinitialize
	int numIn = (int)midiIn_.size();
	int numOut = (int)midiOut_.size();
	if (numIn > 0 || numOut > 0) {
		cleanup();
		initialize(numIn > 0 ? numIn : 1, numOut > 0 ? numOut : 1);
	}
}

void RtMidiBackendImpl::listDevices()
{
	try {
		RtMidiIn tempIn;
		RtMidiOut tempOut;

		unsigned int numSrc = tempIn.getPortCount();
		unsigned int numDst = tempOut.getPortCount();

		printf("midi sources %u destinations %u\n", numSrc, numDst);

		for (unsigned int i = 0; i < numSrc; ++i) {
			std::string name = tempIn.getPortName(i);
			// Use index as UID since RtMidi doesn't provide unique IDs
			printf("MIDI Source %2u '%s' UID: %d\n", i, name.c_str(), (int)i);
		}

		for (unsigned int i = 0; i < numDst; ++i) {
			std::string name = tempOut.getPortName(i);
			printf("MIDI Destination %2u '%s' UID: %d\n", i, name.c_str(), (int)i);
		}
	} catch (const RtMidiError& error) {
		fprintf(stderr, "RtMidi error listing devices: %s\n", error.getMessage().c_str());
	}
}

int RtMidiBackendImpl::connectInput(int uid, int portIndex)
{
	// For RtMidi, uid is the port number (device index)
	if (portIndex < 0 || portIndex >= (int)midiIn_.size()) return errOutOfRange;

	try {
		RtMidiIn tempIn;
		unsigned int numPorts = tempIn.getPortCount();
		if (uid < 0 || (unsigned int)uid >= numPorts) return errOutOfRange;

		// Disconnect existing connection if any
		if (connectedPorts_[portIndex] >= 0) {
			midiIn_[portIndex]->closePort();
		}

		// Set up callback
		auto callbackData = std::make_unique<CallbackData>();
		callbackData->backend = this;
		callbackData->portIndex = portIndex;

		midiIn_[portIndex]->setCallback(midiCallback, callbackData.get());
		midiIn_[portIndex]->openPort(uid, "SAPF Input");

		gCallbackData.push_back(std::move(callbackData));
		connectedPorts_[portIndex] = uid;

		return errNone;
	} catch (const RtMidiError& error) {
		fprintf(stderr, "RtMidi connect error: %s\n", error.getMessage().c_str());
		return errFailed;
	}
}

int RtMidiBackendImpl::disconnectInput(int uid, int portIndex)
{
	if (portIndex < 0 || portIndex >= (int)midiIn_.size()) return errOutOfRange;

	try {
		if (connectedPorts_[portIndex] == uid) {
			midiIn_[portIndex]->closePort();
			connectedPorts_[portIndex] = -1;
		}
		return errNone;
	} catch (const RtMidiError& error) {
		fprintf(stderr, "RtMidi disconnect error: %s\n", error.getMessage().c_str());
		return errFailed;
	}
}

void RtMidiBackendImpl::sendMessage(int port, int destIndex, const std::vector<unsigned char>& message, float /*latencySeconds*/)
{
	// Note: RtMidi doesn't support latency/scheduling - messages are sent immediately
	if (port < 0 || port >= (int)midiOut_.size()) return;
	if (message.empty()) return;

	try {
		RtMidiOut tempOut;
		unsigned int numPorts = tempOut.getPortCount();
		if (destIndex < 0 || (unsigned int)destIndex >= numPorts) return;

		// Open port if not already open
		if (!midiOut_[port]->isPortOpen()) {
			midiOut_[port]->openPort(destIndex, "SAPF Output");
		}

		midiOut_[port]->sendMessage(&message);
	} catch (const RtMidiError& error) {
		fprintf(stderr, "RtMidi send error: %s\n", error.getMessage().c_str());
	}
}

} // namespace

std::unique_ptr<MidiBackend> CreateRtMidiBackend()
{
	try {
		return std::make_unique<RtMidiBackendImpl>();
	} catch (const std::exception& err) {
		fprintf(stderr, "RtMidi init error: %s\n", err.what());
		return nullptr;
	}
}

bool SupportsRtMidiBackend()
{
	return true;
}
