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

	// Audio input support
	void setInputBuffers(float** inputs, int numChannels, int numFrames);
	float** getInputBuffers() const { return inputBuffers_; }
	int getInputChannels() const { return inputChannels_; }
	int getInputFrames() const { return inputFrames_; }

private:
	struct Player;
	void removeFinishedLocked();

	std::vector<std::unique_ptr<Player>> players_;
	std::vector<float> scratch_;

	// Audio input state
	float** inputBuffers_ = nullptr;
	int inputChannels_ = 0;
	int inputFrames_ = 0;
};

MaxAudioBackend& InstallMaxMSPBackend();
void MaxMSPProcessAudio(float** outputs, int numChannels, int numFrames);

// Audio input functions
void MaxMSPSetInputBuffers(float** inputs, int numChannels, int numFrames);
MaxAudioBackend* GetMaxAudioBackend();

// Register adc/adcn primitives - call after engine initialization
void RegisterMaxAudioPrimitives();
