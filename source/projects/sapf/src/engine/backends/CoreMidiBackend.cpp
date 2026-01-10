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

#include "sapf/backends/CoreMidiBackend.hpp"

#if defined(__APPLE__)

#include "MidiRouter.hpp"
#include "ErrorCodes.hpp"
#include "Object.hpp"
#include <CoreMidi/CoreMidi.h>
#include <mach/mach_time.h>
#include <cstdio>
#include <algorithm>

namespace {

class CoreMidiBackend : public MidiBackend
{
public:
	CoreMidiBackend();
	~CoreMidiBackend() override;

	int initialize(int numIn, int numOut) override;
	void cleanup() override;
	void restart() override;
	void listDevices() override;
	int connectInput(int uid, int portIndex) override;
	int disconnectInput(int uid, int portIndex) override;
	void sendMessage(int port, int destIndex, const std::vector<unsigned char>& message, float latencySeconds) override;

private:
	static void midiReadProc(const MIDIPacketList* pktlist, void* readProcRefCon, void* srcConnRefCon);
	static void midiNotifyProc(const MIDINotification* message, void* refCon);
	static MIDITimeStamp midiTime(float latencySeconds);

	MIDIClientRef client_ = 0;
	MIDIPortRef inPorts_[kMaxMidiPorts] = {};
	MIDIPortRef outPorts_[kMaxMidiPorts] = {};
	int numInPorts_ = 0;
	int numOutPorts_ = 0;
	bool initialized_ = false;
};

CoreMidiBackend::CoreMidiBackend()
{
}

CoreMidiBackend::~CoreMidiBackend()
{
	cleanup();
}

void CoreMidiBackend::midiReadProc(const MIDIPacketList* pktlist, void* /*readProcRefCon*/, void* srcConnRefCon)
{
	MIDIPacket* pkt = (MIDIPacket*)pktlist->packet;
	int srcIndex = (int)(size_t)srcConnRefCon;

	for (uint32_t i = 0; i < pktlist->numPackets; ++i) {
		getMidiRouter().handleIncomingMessage(srcIndex, pkt->data, pkt->length);
		pkt = MIDIPacketNext(pkt);
	}
}

void CoreMidiBackend::midiNotifyProc(const MIDINotification* message, void* /*refCon*/)
{
	printf("midi notification %d %d\n", (int)message->messageID, (int)message->messageSize);
}

static struct mach_timebase_info getMachTimebaseInfo() {
	struct mach_timebase_info info;
	mach_timebase_info(&info);
	return info;
}

MIDITimeStamp CoreMidiBackend::midiTime(float latencySeconds)
{
	static struct mach_timebase_info info = getMachTimebaseInfo();
	Float64 latencyNanos = 1000000000.0 * latencySeconds;
	MIDITimeStamp latencyMIDI = (latencyNanos / (Float64)info.numer) * (Float64)info.denom;
	return (MIDITimeStamp)mach_absolute_time() + latencyMIDI;
}

int CoreMidiBackend::initialize(int numIn, int numOut)
{
	cleanup();

	getMidiRouter().resetState();

	numIn = std::clamp(numIn, 1, kMaxMidiPorts);
	numOut = std::clamp(numOut, 1, kMaxMidiPorts);

	int enc = kCFStringEncodingMacRoman;
	CFAllocatorRef alloc = CFAllocatorGetDefault();

	{
		CFStringRef clientName = CFStringCreateWithCString(alloc, "SAPF", enc);
		CFReleaser clientNameReleaser(clientName);

		__block OSStatus err2 = noErr;
		dispatch_sync(dispatch_get_main_queue(), ^{
			err2 = MIDIClientCreate(clientName, midiNotifyProc, nullptr, &client_);
		});
		printf("gMIDIClient %d\n", (int)client_);
		if (err2) {
			fprintf(stderr, "Could not create MIDI client. error %d\n", (int)err2);
			return errFailed;
		}
	}

	OSStatus err = noErr;
	for (int i = 0; i < numIn; ++i) {
		char str[32];
		snprintf(str, 32, "in%d", i);
		CFStringRef inputPortName = CFStringCreateWithCString(alloc, str, enc);
		CFReleaser inputPortNameReleaser(inputPortName);

		err = MIDIInputPortCreate(client_, inputPortName, midiReadProc, this, &inPorts_[i]);
		if (err) {
			numInPorts_ = i;
			fprintf(stderr, "Could not create MIDI port %s. error %d\n", str, (int)err);
			return errFailed;
		}
	}
	numInPorts_ = numIn;

	for (int i = 0; i < numOut; ++i) {
		char str[32];
		snprintf(str, 32, "out%d", i);
		CFStringRef outputPortName = CFStringCreateWithCString(alloc, str, enc);
		CFReleaser outputPortNameReleaser(outputPortName);

		err = MIDIOutputPortCreate(client_, outputPortName, &outPorts_[i]);
		if (err) {
			numOutPorts_ = i;
			fprintf(stderr, "Could not create MIDI out port. error %d\n", (int)err);
			return errFailed;
		}
	}
	numOutPorts_ = numOut;

	initialized_ = true;
	listDevices();

	return errNone;
}

void CoreMidiBackend::cleanup()
{
	for (int i = 0; i < numOutPorts_; ++i) {
		if (outPorts_[i]) {
			MIDIPortDispose(outPorts_[i]);
			outPorts_[i] = 0;
		}
	}
	numOutPorts_ = 0;

	for (int i = 0; i < numInPorts_; ++i) {
		if (inPorts_[i]) {
			MIDIPortDispose(inPorts_[i]);
			inPorts_[i] = 0;
		}
	}
	numInPorts_ = 0;

	if (client_) {
		MIDIClientDispose(client_);
		client_ = 0;
	}

	initialized_ = false;
}

void CoreMidiBackend::restart()
{
	MIDIRestart();
}

void CoreMidiBackend::listDevices()
{
	OSStatus error;
	int numSrc = (int)MIDIGetNumberOfSources();
	int numDst = (int)MIDIGetNumberOfDestinations();

	printf("midi sources %d destinations %d\n", numSrc, numDst);

	for (int i = 0; i < numSrc; ++i) {
		MIDIEndpointRef src = MIDIGetSource(i);
		SInt32 uid = 0;
		MIDIObjectGetIntegerProperty(src, kMIDIPropertyUniqueID, &uid);

		MIDIEntityRef ent;
		error = MIDIEndpointGetEntity(src, &ent);

		CFStringRef devname, endname;
		char cendname[1024], cdevname[1024];

		if (error) {
			// Virtual sources don't have entities
			MIDIObjectGetStringProperty(src, kMIDIPropertyName, &devname);
			MIDIObjectGetStringProperty(src, kMIDIPropertyName, &endname);
			CFStringGetCString(devname, cdevname, 1024, kCFStringEncodingUTF8);
			CFStringGetCString(endname, cendname, 1024, kCFStringEncodingUTF8);
		} else {
			MIDIDeviceRef dev;
			MIDIEntityGetDevice(ent, &dev);
			MIDIObjectGetStringProperty(dev, kMIDIPropertyName, &devname);
			MIDIObjectGetStringProperty(src, kMIDIPropertyName, &endname);
			CFStringGetCString(devname, cdevname, 1024, kCFStringEncodingUTF8);
			CFStringGetCString(endname, cendname, 1024, kCFStringEncodingUTF8);
		}

		printf("MIDI Source %2d '%s', '%s' UID: %d\n", i, cdevname, cendname, uid);
	}

	for (int i = 0; i < numDst; ++i) {
		MIDIEndpointRef dst = MIDIGetDestination(i);
		SInt32 uid = 0;
		MIDIObjectGetIntegerProperty(dst, kMIDIPropertyUniqueID, &uid);

		MIDIEntityRef ent;
		error = MIDIEndpointGetEntity(dst, &ent);

		CFStringRef devname, endname;
		char cendname[1024], cdevname[1024];

		if (error) {
			// Virtual destinations don't have entities
			MIDIObjectGetStringProperty(dst, kMIDIPropertyName, &devname);
			MIDIObjectGetStringProperty(dst, kMIDIPropertyName, &endname);
			CFStringGetCString(devname, cdevname, 1024, kCFStringEncodingUTF8);
			CFStringGetCString(endname, cendname, 1024, kCFStringEncodingUTF8);
		} else {
			MIDIDeviceRef dev;
			MIDIEntityGetDevice(ent, &dev);
			MIDIObjectGetStringProperty(dev, kMIDIPropertyName, &devname);
			MIDIObjectGetStringProperty(dst, kMIDIPropertyName, &endname);
			CFStringGetCString(devname, cdevname, 1024, kCFStringEncodingUTF8);
			CFStringGetCString(endname, cendname, 1024, kCFStringEncodingUTF8);
		}

		printf("MIDI Destination %2d '%s', '%s' UID: %d\n", i, cdevname, cendname, uid);
	}
}

int CoreMidiBackend::connectInput(int uid, int portIndex)
{
	if (portIndex < 0 || portIndex >= numInPorts_) return errOutOfRange;

	MIDIEndpointRef src = 0;
	MIDIObjectType mtype;
	MIDIObjectFindByUniqueID(uid, (MIDIObjectRef*)&src, &mtype);
	if (mtype != kMIDIObjectType_Source) return errFailed;

	void* p = (void*)(uintptr_t)portIndex;
	MIDIPortConnectSource(inPorts_[portIndex], src, p);

	return errNone;
}

int CoreMidiBackend::disconnectInput(int uid, int portIndex)
{
	if (portIndex < 0 || portIndex >= numInPorts_) return errOutOfRange;

	MIDIEndpointRef src = 0;
	MIDIObjectType mtype;
	MIDIObjectFindByUniqueID(uid, (MIDIObjectRef*)&src, &mtype);
	if (mtype != kMIDIObjectType_Source) return errFailed;

	MIDIPortDisconnectSource(inPorts_[portIndex], src);

	return errNone;
}

void CoreMidiBackend::sendMessage(int port, int destIndex, const std::vector<unsigned char>& message, float latencySeconds)
{
	if (port < 0 || port >= numOutPorts_) return;
	if (message.empty()) return;

	MIDIEndpointRef dest = MIDIGetDestination(destIndex);
	if (!dest) return;

	MIDIPacketList mpktlist;
	MIDIPacketList* pktlist = &mpktlist;
	MIDIPacket* pk = MIDIPacketListInit(pktlist);

	ByteCount nData = (ByteCount)message.size();
	pk = MIDIPacketListAdd(pktlist, sizeof(MIDIPacketList), pk, midiTime(latencySeconds), nData, message.data());
	if (pk) {
		MIDISend(outPorts_[port], dest, pktlist);
	}
}

} // namespace

std::unique_ptr<MidiBackend> CreateCoreMidiBackend()
{
	return std::make_unique<CoreMidiBackend>();
}

bool SupportsCoreMidiBackend()
{
	return true;
}

#else // !__APPLE__

std::unique_ptr<MidiBackend> CreateCoreMidiBackend()
{
	return nullptr;
}

bool SupportsCoreMidiBackend()
{
	return false;
}

#endif // __APPLE__
