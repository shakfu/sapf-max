#pragma once

#include <memory>
#include <vector>

#include "sapf/AudioBackend.hpp"

class MaxAudioBackend : public AudioBackend {
public:
	MaxAudioBackend();
	~MaxAudioBackend() override;

	void play(Thread& th, V& v) override;
	void record(Thread& th, V& v, Arg filename) override;
	void stopAll() override;
	void stopFinished() override;

	void render(float** outputs, int numChannels, int numFrames);

private:
	struct Player;
	void removeFinishedLocked();

	std::vector<std::unique_ptr<Player>> players_;
	std::vector<float> scratch_;
};

MaxAudioBackend& InstallMaxMSPBackend();
void MaxMSPProcessAudio(float** outputs, int numChannels, int numFrames);
