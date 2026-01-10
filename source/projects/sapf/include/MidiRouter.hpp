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

#pragma once

#include <cstdint>
#include <vector>

// Maximum number of MIDI input ports supported
const int kMaxMidiPorts = 16;

// Per-channel MIDI state tracking
struct MidiChanState
{
	uint8_t control[128];
	uint8_t polytouch[128];
	uint8_t keyvel[128];
	uint32_t numKeysDown;
	uint16_t bend;
	uint8_t touch;
	uint8_t program;
	uint8_t lastkey;
	uint8_t lastvel;
};

// Global MIDI state array indexed by [port][channel]
extern MidiChanState gMidiState[kMaxMidiPorts][16];

// Global MIDI debug flag
extern bool gMidiDebug;

// Global MIDI lag time for smoothing
extern double gMidiLagTime;
extern double gMidiLagMul;

// MidiRouter: Platform-agnostic MIDI message routing and state management
class MidiRouter
{
public:
	static MidiRouter& instance();

	// Initialize/reset all MIDI state
	void resetState();

	// Handle incoming MIDI message bytes from any backend
	// srcIndex: which input port (0 to kMaxMidiPorts-1)
	// data: pointer to message bytes
	// length: number of bytes in message
	void handleIncomingMessage(int srcIndex, const uint8_t* data, int length);

	// Access state for UGens
	MidiChanState& getState(int srcIndex, int chan);

private:
	MidiRouter() = default;
	~MidiRouter() = default;
	MidiRouter(const MidiRouter&) = delete;
	MidiRouter& operator=(const MidiRouter&) = delete;

	// Internal message processing
	int processSystemPacket(const uint8_t* data, int length, int chan);

	// Sysex handling state
	bool sysexFlag_ = false;
	uint8_t runningStatus_ = 0;
	std::vector<uint8_t> sysexData_;

	void sysexBegin();
	void sysexEnd(int lastUID);
	void sysexEndInvalid();
};

// Convenience function to get the router instance
inline MidiRouter& getMidiRouter() { return MidiRouter::instance(); }
