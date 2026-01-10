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

#include "MidiRouter.hpp"
#include <cstring>
#include <cstdio>
#include <cmath>

// Global MIDI state
MidiChanState gMidiState[kMaxMidiPorts][16];
bool gMidiDebug = false;

// Lag configuration
const double log001 = std::log(0.001);
double gMidiLagTime = 0.1;
double gMidiLagMul = log001 / gMidiLagTime;

MidiRouter& MidiRouter::instance()
{
	static MidiRouter router;
	return router;
}

void MidiRouter::resetState()
{
	memset(gMidiState, 0, sizeof(gMidiState));
	sysexFlag_ = false;
	runningStatus_ = 0;
	sysexData_.clear();
}

MidiChanState& MidiRouter::getState(int srcIndex, int chan)
{
	return gMidiState[srcIndex & (kMaxMidiPorts - 1)][chan & 15];
}

void MidiRouter::sysexBegin()
{
	runningStatus_ = 0; // clear running status
	sysexFlag_ = true;
}

void MidiRouter::sysexEnd(int /*lastUID*/)
{
	sysexFlag_ = false;
}

void MidiRouter::sysexEndInvalid()
{
	sysexFlag_ = false;
}

int MidiRouter::processSystemPacket(const uint8_t* data, int length, int chan)
{
	switch (chan) {
	case 7: // Sysex EOX must be taken into account if first on data packet
	case 0:
		{
		int lastUid = 0;
		int m = length;
		const uint8_t* p = data;
		uint8_t val;

		while (m--) {
			val = *p++;
			if (val & 0x80) { // status byte
				if (val == 0xF7) { // end packet
					sysexData_.push_back(val); // add EOX
					if (sysexFlag_)
						sysexEnd(lastUid);
					else
						sysexEndInvalid(); // invalid 1 byte with only EOX can happen
					break;
				}
				else if (val == 0xF0) { // new packet
					if (sysexFlag_) { // invalid new one/should not happen -- but handle in case
						sysexEndInvalid();
					}
					sysexBegin(); // new sysex in
				}
				else { // abnormal data in middle of sysex packet
					sysexEndInvalid(); // flush invalid
					m = 0; // discard all packet
					break;
				}
			}
			else if (sysexFlag_) {
				// add byte to sysex buffer if needed
			} else { // garbage - handle in case - discard it
				break;
			}
		}
		return (length - m);
		}

	case 1:
		// MTC quarter frame
		return 2;

	case 2:
		// song position pointer
		return 3;

	case 3:
		// song select
		return 2;

	case 8:  // clock
	case 10: // start
	case 11: // continue
	case 12: // stop
	case 15: // reset
		runningStatus_ = 0; // clear running status
		return 1;

	default:
		break;
	}

	return 1;
}

void MidiRouter::handleIncomingMessage(int srcIndex, const uint8_t* data, int length)
{
	if (!data || length <= 0) return;

	srcIndex = srcIndex & (kMaxMidiPorts - 1);

	int i = 0;
	while (i < length) {
		uint8_t status = data[i] & 0xF0;
		uint8_t chan = data[i] & 0x0F;
		uint8_t a, b;

		if (status & 0x80) // set the running status for voice messages
			runningStatus_ = ((status >> 4) == 0xF) ? 0 : data[i];

	processMessage:
		switch (status) {
		case 0x80: // noteOff
			if (i + 2 >= length) return;
			a = data[i+1];
			b = data[i+2];
			if (gMidiDebug) printf("midi note off %d %d %d %d\n", srcIndex, chan+1, a, b);
			gMidiState[srcIndex][chan].keyvel[a] = 0;
			if (gMidiState[srcIndex][chan].numKeysDown > 0)
				--gMidiState[srcIndex][chan].numKeysDown;
			i += 3;
			break;
		case 0x90: // noteOn
			if (i + 2 >= length) return;
			a = data[i+1];
			b = data[i+2];
			if (gMidiDebug) printf("midi note on %d %d %d %d\n", srcIndex, chan+1, a, b);
			if (b) {
				gMidiState[srcIndex][chan].lastkey = a;
				gMidiState[srcIndex][chan].lastvel = b;
				++gMidiState[srcIndex][chan].numKeysDown;
			} else {
				if (gMidiState[srcIndex][chan].numKeysDown > 0)
					--gMidiState[srcIndex][chan].numKeysDown;
			}
			gMidiState[srcIndex][chan].keyvel[a] = b;
			i += 3;
			break;
		case 0xA0: // polytouch
			if (i + 2 >= length) return;
			a = data[i+1];
			b = data[i+2];
			if (gMidiDebug) printf("midi poly %d %d %d %d\n", srcIndex, chan+1, a, b);
			gMidiState[srcIndex][chan].polytouch[a] = b;
			i += 3;
			break;
		case 0xB0: // control
			if (i + 2 >= length) return;
			a = data[i+1];
			b = data[i+2];
			if (gMidiDebug) printf("midi control %d %d %d %d\n", srcIndex, chan+1, a, b);
			gMidiState[srcIndex][chan].control[a] = b;
			if (a == 120 || (a >= 123 && a <= 127)) {
				// all notes off
				memset(gMidiState[srcIndex][chan].keyvel, 0, 128);
				gMidiState[srcIndex][chan].numKeysDown = 0;
			} else if (a == 121) {
				// reset ALL controls to zero
				memset(gMidiState[srcIndex][chan].control, 0, 128);
				gMidiState[srcIndex][chan].bend = 0x4000;
			}
			i += 3;
			break;
		case 0xC0: // program
			if (i + 1 >= length) return;
			a = data[i+1];
			gMidiState[srcIndex][chan].program = a;
			if (gMidiDebug) printf("midi program %d %d %d\n", srcIndex, chan+1, a);
			i += 2;
			break;
		case 0xD0: // touch
			if (i + 1 >= length) return;
			a = data[i+1];
			if (gMidiDebug) printf("midi touch %d %d\n", chan+1, a);
			gMidiState[srcIndex][chan].touch = a;
			i += 2;
			break;
		case 0xE0: // bend
			if (i + 2 >= length) return;
			a = data[i+1];
			b = data[i+2];
			if (gMidiDebug) printf("midi bend %d %d %d %d\n", srcIndex, chan+1, a, b);
			gMidiState[srcIndex][chan].bend = ((b << 7) | a) - 8192;
			i += 3;
			break;
		case 0xF0:
			i += processSystemPacket(data + i, length - i, chan);
			break;
		default: // data byte => continuing sysex or running status
			if (runningStatus_ && !sysexFlag_) {
				status = runningStatus_ & 0xF0;
				chan = runningStatus_ & 0x0F;
				--i;
				goto processMessage;
			}
			chan = 0;
			i += processSystemPacket(data + i, length - i, chan);
			break;
		}
	}
}
