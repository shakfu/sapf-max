#include "sapf/MidiBackend.hpp"

#include <stdexcept>

#include "sapf/backends/CoreMidiBackend.hpp"
#include "sapf/backends/NullMidiBackend.hpp"
#ifdef SAPF_USE_RTMIDI
#include "sapf/backends/RtMidiBackend.hpp"
#endif

namespace {

std::unique_ptr<MidiBackend> gMidiBackend;

} // namespace

MidiBackend& GetMidiBackend()
{
	if (!gMidiBackend) {
		throw std::runtime_error("MIDI backend not configured");
	}
	return *gMidiBackend;
}

void SetMidiBackend(std::unique_ptr<MidiBackend> backend)
{
	gMidiBackend = std::move(backend);
}

bool HasMidiBackend()
{
	return static_cast<bool>(gMidiBackend);
}

void EnsureDefaultMidiBackend()
{
	if (HasMidiBackend()) {
		return;
	}

#if defined(__APPLE__)
	if (SupportsCoreMidiBackend()) {
		SetMidiBackend(CreateCoreMidiBackend());
		return;
	}
#ifdef SAPF_USE_RTMIDI
	if (auto backend = CreateRtMidiBackend()) {
		SetMidiBackend(std::move(backend));
		return;
	}
#endif
#elif defined(__linux__)
#ifdef SAPF_USE_RTMIDI
	if (auto backend = CreateRtMidiBackend()) {
		SetMidiBackend(std::move(backend));
		return;
	}
#endif
#elif defined(_WIN32)
#ifdef SAPF_USE_RTMIDI
	if (auto backend = CreateRtMidiBackend()) {
		SetMidiBackend(std::move(backend));
		return;
	}
#endif
	SetMidiBackend(CreateNullMidiBackend("Windows MIDI backend will use RtMidi once configured."));
	return;
#else
#ifdef SAPF_USE_RTMIDI
	if (auto backend = CreateRtMidiBackend()) {
		SetMidiBackend(std::move(backend));
		return;
	}
#endif
#endif

	SetMidiBackend(CreateNullMidiBackend("MIDI backend not available on this platform."));
}
