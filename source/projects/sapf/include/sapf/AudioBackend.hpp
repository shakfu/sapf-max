#pragma once

#include <memory>

#include "VM.hpp"

class AudioBackend {
public:
	virtual ~AudioBackend() = default;

	virtual void play(Thread& th, V& v) = 0;
	virtual void record(Thread& th, V& v, Arg filename) = 0;
	virtual void stopAll() = 0;
	virtual void stopFinished() = 0;
};

void SetAudioBackend(std::unique_ptr<AudioBackend> backend);
AudioBackend& GetAudioBackend();
bool HasAudioBackend();
void EnsureDefaultAudioBackend();
