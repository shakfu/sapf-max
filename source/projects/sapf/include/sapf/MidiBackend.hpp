#pragma once

#include <memory>
#include <vector>

class MidiBackend {
public:
	virtual ~MidiBackend() = default;

	// Initialize the MIDI subsystem with the given number of input/output ports
	virtual int initialize(int numIn, int numOut) = 0;

	// Cleanup/shutdown the MIDI subsystem
	virtual void cleanup() = 0;

	// Restart/rescan MIDI devices
	virtual void restart() = 0;

	// List available MIDI devices (prints to stdout)
	virtual void listDevices() = 0;

	// Connect a MIDI source to an input port
	// uid: unique identifier of the source device
	// portIndex: which input port to connect to (0 to numIn-1)
	virtual int connectInput(int uid, int portIndex) = 0;

	// Disconnect a MIDI source from an input port
	virtual int disconnectInput(int uid, int portIndex) = 0;

	// Send a MIDI message
	// port: output port index
	// destIndex: destination device index
	// message: MIDI bytes to send
	// latencySeconds: delay before sending
	virtual void sendMessage(int port, int destIndex, const std::vector<unsigned char>& message, float latencySeconds) = 0;
};

MidiBackend& GetMidiBackend();
void SetMidiBackend(std::unique_ptr<MidiBackend> backend);
bool HasMidiBackend();
void EnsureDefaultMidiBackend();
