#include "sapf/AudioBackend.hpp"

#include <stdexcept>

#include "sapf/backends/AlsaAudioBackend.hpp"
#include "sapf/backends/CoreAudioBackend.hpp"
#include "sapf/backends/NullAudioBackend.hpp"
#ifdef SAPF_USE_RTAUDIO
#include "sapf/backends/RtAudioBackend.hpp"
#endif

namespace {

std::unique_ptr<AudioBackend> gAudioBackend;

} // namespace

void SetAudioBackend(std::unique_ptr<AudioBackend> backend)
{
	gAudioBackend = std::move(backend);
}

AudioBackend& GetAudioBackend()
{
	if (!gAudioBackend) {
		throw std::runtime_error("Audio backend not configured");
	}
	return *gAudioBackend;
}

bool HasAudioBackend()
{
	return static_cast<bool>(gAudioBackend);
}

void EnsureDefaultAudioBackend()
{
	if (HasAudioBackend()) {
		return;
	}

#if defined(__APPLE__)
	if (SupportsCoreAudioBackend()) {
		SetAudioBackend(CreateCoreAudioBackend());
		return;
	}
#ifdef SAPF_USE_RTAUDIO
	if (auto backend = CreateRtAudioBackend()) {
		SetAudioBackend(std::move(backend));
		return;
	}
#endif
#elif defined(__linux__)
#ifdef SAPF_USE_RTAUDIO
	if (auto backend = CreateRtAudioBackend()) {
		SetAudioBackend(std::move(backend));
		return;
	}
#endif
	if (SupportsAlsaAudioBackend()) {
		auto backend = CreateAlsaAudioBackend();
		if (backend) {
			SetAudioBackend(std::move(backend));
			return;
		}
	}
#elif defined(_WIN32)
#ifdef SAPF_USE_RTAUDIO
	if (auto backend = CreateRtAudioBackend()) {
		SetAudioBackend(std::move(backend));
		return;
	}
#endif
	SetAudioBackend(CreateNullAudioBackend("Windows audio backend will use RtAudio once configured."));
	return;
#else
#ifdef SAPF_USE_RTAUDIO
	if (auto backend = CreateRtAudioBackend()) {
		SetAudioBackend(std::move(backend));
		return;
	}
#endif
#endif

	SetAudioBackend(CreateNullAudioBackend("Audio backend not available on this platform."));
}
