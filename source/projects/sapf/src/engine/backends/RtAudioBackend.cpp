#include "sapf/backends/RtAudioBackend.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <vector>

#include "RtAudio.h"
#include "SoundFiles.hpp"

namespace {

const int kMaxChannels = 32;
const unsigned int kFramesPerBuffer = 256;

class RtAudioBackend : public AudioBackend
{
public:
	RtAudioBackend();
	~RtAudioBackend() override;

	void play(Thread& th, V& v) override;
	void record(Thread& th, V& v, Arg filename) override;
	void stopAll() override;
	void stopFinished() override;

private:
	struct Player {
		Player(const Thread& parentThread, int channels)
			: th(parentThread)
			, numChannels(channels)
			, in(channels)
		{
		}

		Thread th;
		int numChannels;
		std::vector<ZIn> in;
		bool done = false;
#if defined(__APPLE__)
		ExtAudioFileRef recordFile = nullptr;
		std::string recordPath;
		std::vector<std::vector<float>> recordChannels;
#endif
	};

	static int audioCallback(void* outputBuffer, void* inputBuffer, unsigned int nFrames, double streamTime, RtAudioStreamStatus status, void* userData);
	int render(float* output, unsigned int frames);
	bool ensureStreamLocked();
	void closeStreamLocked();
	void removeFinishedLocked();
	std::unique_ptr<Player> createPlayer(Thread& th, V& v);
	void finalizePlayer(std::unique_ptr<Player>& player);

	RtAudio audio_;
	std::mutex mutex_;
	std::vector<std::unique_ptr<Player>> players_;
	std::vector<float> scratch_;
	std::vector<float> mix_;
	bool streamOpen_ = false;
	int streamChannels_ = 0;
	unsigned int streamSampleRate_ = 0;
};

} // namespace

RtAudioBackend::RtAudioBackend()
{
}

RtAudioBackend::~RtAudioBackend()
{
	stopAll();
}

std::unique_ptr<RtAudioBackend::Player> RtAudioBackend::createPlayer(Thread& th, V& v)
{
	if (!v.isList()) wrongType("play : s", "List", v);

	std::unique_ptr<Player> player;

	if (v.isZList()) {
		player = std::make_unique<Player>(th, 1);
		player->in[0].set(v);
	} else {
		if (!v.isFinite()) indefiniteOp("play : s", "");
		P<List> s = (List*)v.o();
		s = s->pack(th, kMaxChannels);
		if (!s()) {
			post("Too many channels. Max is %d.\n", kMaxChannels);
			return nullptr;
		}
		Array* a = s->mArray();
		int asize = (int)a->size();
		player = std::make_unique<Player>(th, asize);
		for (int i = 0; i < asize; ++i) {
			player->in[i].set(a->at(i));
		}
		s = nullptr;
		a = nullptr;
	}
	v.o = nullptr;
	return player;
}

void RtAudioBackend::play(Thread& th, V& v)
{
	auto player = createPlayer(th, v);
	if (!player) {
		return;
	}

	std::lock_guard<std::mutex> lock(mutex_);
	if (!streamOpen_) {
		if (!ensureStreamLocked()) {
			throw errFailed;
		}
	}
	players_.push_back(std::move(player));
}

void RtAudioBackend::record(Thread& th, V& v, Arg filename)
{
#if defined(__APPLE__)
	auto player = createPlayer(th, v);
	if (!player) {
		return;
	}

	char path[1024];
	makeRecordingPath(filename, path, sizeof(path));
	ExtAudioFileRef xaf = sfcreate(th, path, player->numChannels, 0., false);
	if (!xaf) {
		throw errFailed;
	}
	player->recordFile = xaf;
	player->recordPath = path;
	player->recordChannels.resize(player->numChannels);

	std::lock_guard<std::mutex> lock(mutex_);
	if (!streamOpen_) {
		if (!ensureStreamLocked()) {
			ExtAudioFileDispose(player->recordFile);
			player->recordFile = nullptr;
			throw errFailed;
		}
	}
	players_.push_back(std::move(player));
#else
	(void)th; (void)v; (void)filename;
	post("record: Recording not implemented on this platform (requires AudioToolbox).\n");
	post("        Use 'play' instead, or contribute a libsndfile-based implementation.\n");
#endif
}

void RtAudioBackend::stopAll()
{
	std::lock_guard<std::mutex> lock(mutex_);
	for (auto& player : players_) {
		finalizePlayer(player);
	}
	players_.clear();
	closeStreamLocked();
}

void RtAudioBackend::stopFinished()
{
	std::lock_guard<std::mutex> lock(mutex_);
	removeFinishedLocked();
	if (players_.empty()) {
		closeStreamLocked();
	}
}

int RtAudioBackend::audioCallback(void* outputBuffer, void*, unsigned int nFrames, double, RtAudioStreamStatus, void* userData)
{
	auto* backend = static_cast<RtAudioBackend*>(userData);
	return backend->render(static_cast<float*>(outputBuffer), nFrames);
}

int RtAudioBackend::render(float* output, unsigned int frames)
{
	std::lock_guard<std::mutex> lock(mutex_);
	const int numChannels = std::max(1, streamChannels_);
	const size_t samples = static_cast<size_t>(frames) * numChannels;

	if (!output) {
		return 0;
	}

	if (!streamOpen_ || players_.empty()) {
		std::fill(output, output + samples, 0.f);
		if (players_.empty()) {
			closeStreamLocked();
		}
		return 0;
	}

	mix_.assign(samples, 0.f);
	scratch_.resize(frames);

	auto it = players_.begin();
	while (it != players_.end()) {
		Player& player = *(*it);
		const int channels = std::min(player.numChannels, numChannels);
		bool done = true;

		for (int ch = 0; ch < channels; ++ch) {
			int framesToFill = static_cast<int>(frames);
			bool finished = player.in[ch].fill(player.th, framesToFill, scratch_.data(), 1);
			for (int i = 0; i < framesToFill; ++i) {
				const size_t idx = static_cast<size_t>(i) * numChannels + ch;
				mix_[idx] += scratch_[i];
			}
#if defined(__APPLE__)
			if (player.recordFile) {
				if (player.recordChannels.size() < static_cast<size_t>(player.numChannels)) {
					player.recordChannels.resize(player.numChannels);
				}
				auto& chanBuf = player.recordChannels[ch];
				chanBuf.assign(frames, 0.f);
				std::copy_n(scratch_.data(), framesToFill, chanBuf.data());
			}
#endif
			done = done && finished;
		}

#if defined(__APPLE__)
		if (player.recordFile) {
			for (int ch = channels; ch < player.numChannels; ++ch) {
				player.recordChannels[ch].assign(frames, 0.f);
			}
			size_t ablSize = sizeof(AudioBufferList) + (player.numChannels - 1) * sizeof(AudioBuffer);
			std::vector<uint8_t> storage(ablSize);
			auto* abl = reinterpret_cast<AudioBufferList*>(storage.data());
			abl->mNumberBuffers = player.numChannels;
			for (int ch = 0; ch < player.numChannels; ++ch) {
				abl->mBuffers[ch].mNumberChannels = 1;
				abl->mBuffers[ch].mDataByteSize = frames * sizeof(float);
				abl->mBuffers[ch].mData = player.recordChannels[ch].data();
			}
			OSStatus err = ExtAudioFileWrite(player.recordFile, frames, abl);
			if (err) {
				post("ExtAudioFileWrite failed %d\n", (int)err);
				ExtAudioFileDispose(player.recordFile);
				player.recordFile = nullptr;
			}
		}
#endif

		player.done = done;
		if (player.done) {
			finalizePlayer(*it);
			it = players_.erase(it);
		} else {
			++it;
		}
	}

	std::copy(mix_.begin(), mix_.end(), output);

	if (players_.empty()) {
		closeStreamLocked();
	}

	return 0;
}

bool RtAudioBackend::ensureStreamLocked()
{
	if (streamOpen_) {
		return true;
	}

	if (audio_.getDeviceCount() == 0) {
		post("RtAudio: no audio devices available.\n");
		return false;
	}

	RtAudio::StreamParameters params;
	params.deviceId = audio_.getDefaultOutputDevice();
	RtAudio::DeviceInfo info = audio_.getDeviceInfo(params.deviceId);
	streamChannels_ = std::min(kMaxChannels, static_cast<int>(info.outputChannels));
	if (streamChannels_ <= 0) {
		post("RtAudio: selected device has no output channels.\n");
		return false;
	}
	params.nChannels = streamChannels_;
	params.firstChannel = 0;

	RtAudio::StreamOptions options;
	options.flags = 0;

	streamSampleRate_ = static_cast<unsigned int>(vm.ar.sampleRate);
	unsigned int frames = kFramesPerBuffer;

	RtAudioErrorType err = audio_.openStream(&params, nullptr, RTAUDIO_FLOAT32, streamSampleRate_, &frames, &RtAudioBackend::audioCallback, this, &options);
	if (err != RTAUDIO_NO_ERROR) {
		post("RtAudio open error: %s\n", audio_.getErrorText().c_str());
		streamOpen_ = false;
		return false;
	}

	err = audio_.startStream();
	if (err != RTAUDIO_NO_ERROR) {
		post("RtAudio start error: %s\n", audio_.getErrorText().c_str());
		audio_.closeStream();
		streamOpen_ = false;
		return false;
	}

	streamOpen_ = true;
	return true;
}

void RtAudioBackend::closeStreamLocked()
{
	if (!streamOpen_) {
		return;
	}
	if (audio_.isStreamRunning()) {
		audio_.stopStream();
	}
	if (audio_.isStreamOpen()) {
		audio_.closeStream();
	}
	streamOpen_ = false;
	streamChannels_ = 0;
}

void RtAudioBackend::removeFinishedLocked()
{
	auto it = players_.begin();
	while (it != players_.end()) {
		if ((*it)->done) {
			finalizePlayer(*it);
			it = players_.erase(it);
		} else {
			++it;
		}
	}
}

void RtAudioBackend::finalizePlayer(std::unique_ptr<Player>& player)
{
#if defined(__APPLE__)
	if (player->recordFile) {
		ExtAudioFileDispose(player->recordFile);
		player->recordFile = nullptr;
		if (!player->recordPath.empty()) {
			char cmd[1100];
			snprintf(cmd, sizeof(cmd), "open \"%s\"", player->recordPath.c_str());
			system(cmd);
		}
	}
#else
	(void)player;
#endif
}

std::unique_ptr<AudioBackend> CreateRtAudioBackend()
{
	try {
		return std::unique_ptr<AudioBackend>(new RtAudioBackend());
	} catch (const std::exception& err) {
		post("RtAudio init error: %s\n", err.what());
		return nullptr;
	}
}

bool SupportsRtAudioBackend()
{
	return true;
}
