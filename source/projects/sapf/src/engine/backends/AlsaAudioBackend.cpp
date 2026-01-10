#include "sapf/backends/AlsaAudioBackend.hpp"

#if defined(__linux__)

#include <alsa/asoundlib.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <cstdio>

namespace {

const int kMaxChannels = 32;
const unsigned int kFramesPerBuffer = 256;

} // namespace

class AlsaAudioBackend : public AudioBackend
{
public:
	AlsaAudioBackend();
	~AlsaAudioBackend() override;

	void play(Thread& th, V& v) override;
	void record(Thread& th, V& v, Arg filename) override;
	void stopAll() override;
	void stopFinished() override;

private:
	struct Player {
		Player(const Thread& parentThread, int channels)
			: th(parentThread), numChannels(channels), in(channels)
		{
		}
		Thread th;
		int numChannels;
		std::vector<ZIn> in;
		bool done = false;
		// Recording not implemented on Linux - would need libsndfile
	};

	void audioThreadLoop();
	bool ensurePcmLocked();
	void closePcmLocked();
	void removeFinishedLocked();
	void wakeThread();
	std::unique_ptr<Player> createPlayer(Thread& th, V& v);
	void finalizePlayer(std::unique_ptr<Player>& player);

	std::mutex mutex_;
	std::condition_variable cv_;
	std::vector<std::unique_ptr<Player>> players_;
	std::thread audioThread_;
	bool running_ = true;

	snd_pcm_t* pcm_ = nullptr;
	unsigned int sampleRate_ = 0;
	int numChannels_ = 2;

	std::vector<float> mixBuffer_;
	std::vector<float> scratch_;
};

AlsaAudioBackend::AlsaAudioBackend()
{
	sampleRate_ = static_cast<unsigned int>(vm.ar.sampleRate);
	audioThread_ = std::thread([this]() { audioThreadLoop(); });
}

AlsaAudioBackend::~AlsaAudioBackend()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		running_ = false;
		closePcmLocked();
		players_.clear();
	}
	cv_.notify_all();
	if (audioThread_.joinable()) {
		audioThread_.join();
	}
}

std::unique_ptr<AlsaAudioBackend::Player> AlsaAudioBackend::createPlayer(Thread& th, V& v)
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

void AlsaAudioBackend::play(Thread& th, V& v)
{
	auto player = createPlayer(th, v);
	if (!player) {
		return;
	}

	{
		std::lock_guard<std::mutex> lock(mutex_);
		players_.push_back(std::move(player));
	}
	wakeThread();
}

void AlsaAudioBackend::record(Thread& th, V& v, Arg filename)
{
	post("record: Recording not implemented on Linux (requires libsndfile).\n");
	post("        Use 'play' instead, or contribute a libsndfile-based implementation.\n");
}

void AlsaAudioBackend::stopAll()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		for (auto& player : players_) {
			finalizePlayer(player);
		}
		players_.clear();
		closePcmLocked();
	}
	wakeThread();
}

void AlsaAudioBackend::stopFinished()
{
	std::lock_guard<std::mutex> lock(mutex_);
	removeFinishedLocked();
	if (players_.empty()) {
		closePcmLocked();
	}
}

void AlsaAudioBackend::audioThreadLoop()
{
	std::unique_lock<std::mutex> lock(mutex_);
	while (running_) {
		if (players_.empty()) {
			cv_.wait(lock, [this]() { return !running_ || !players_.empty(); });
			if (!running_) break;
		}

		if (!ensurePcmLocked()) {
			players_.clear();
			cv_.wait_for(lock, std::chrono::milliseconds(10));
			continue;
		}

		size_t frames = kFramesPerBuffer;
		size_t samples = frames * static_cast<size_t>(numChannels_);
		mixBuffer_.assign(samples, 0.f);
		scratch_.resize(frames);

		for (auto it = players_.begin(); it != players_.end();) {
			Player& player = *(*it);
			int channels = std::min(player.numChannels, numChannels_);
			bool done = true;

			for (int ch = 0; ch < channels; ++ch) {
				int framesToFill = static_cast<int>(frames);
				bool finished = player.in[ch].fill(player.th, framesToFill, scratch_.data(), 1);
				for (int i = 0; i < framesToFill; ++i) {
					size_t idx = static_cast<size_t>(i) * numChannels_ + ch;
					mixBuffer_[idx] += scratch_[i];
				}
				if (framesToFill < static_cast<int>(frames)) {
					for (size_t i = framesToFill; i < frames; ++i) {
						size_t idx = i * numChannels_ + ch;
						// already zero
					}
				}
				done = done && finished;
			}

			player.done = done;
			if (player.done) {
				it = players_.erase(it);
			} else {
				++it;
			}
		}

		std::vector<float> localBuffer = mixBuffer_;
		lock.unlock();

		if (pcm_) {
			const float* data = localBuffer.data();
			size_t framesLeft = frames;
			while (framesLeft > 0) {
				snd_pcm_sframes_t written = snd_pcm_writei(pcm_, data, framesLeft);
				if (written < 0) {
					if (written == -EPIPE) {
						snd_pcm_prepare(pcm_);
						continue;
					} else if (written == -EAGAIN) {
						continue;
					} else {
						post("ALSA write error: %s\n", snd_strerror(written));
						break;
					}
				} else {
					framesLeft -= static_cast<size_t>(written);
					data += written * numChannels_;
				}
			}
		}

		lock.lock();
		if (players_.empty()) {
			closePcmLocked();
		}
	}
}

bool AlsaAudioBackend::ensurePcmLocked()
{
	if (pcm_) return true;

	snd_pcm_t* handle = nullptr;
	int err = snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
	if (err < 0) {
		post("ALSA open error: %s\n", snd_strerror(err));
		return false;
	}

	snd_pcm_hw_params_t* params = nullptr;
	snd_pcm_hw_params_malloc(&params);
	snd_pcm_hw_params_any(handle, params);
	snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
	snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_FLOAT);
	snd_pcm_hw_params_set_channels(handle, params, numChannels_);
	unsigned int rate = sampleRate_;
	snd_pcm_hw_params_set_rate_near(handle, params, &rate, 0);
	snd_pcm_uframes_t period = kFramesPerBuffer;
	snd_pcm_hw_params_set_period_size_near(handle, params, &period, 0);

	err = snd_pcm_hw_params(handle, params);
	snd_pcm_hw_params_free(params);
	if (err < 0) {
		post("ALSA hw params error: %s\n", snd_strerror(err));
		snd_pcm_close(handle);
		return false;
	}

	err = snd_pcm_prepare(handle);
	if (err < 0) {
		post("ALSA prepare error: %s\n", snd_strerror(err));
		snd_pcm_close(handle);
		return false;
	}

	pcm_ = handle;
	return true;
}

void AlsaAudioBackend::closePcmLocked()
{
	if (pcm_) {
		snd_pcm_drain(pcm_);
		snd_pcm_close(pcm_);
		pcm_ = nullptr;
	}
}

void AlsaAudioBackend::removeFinishedLocked()
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

void AlsaAudioBackend::wakeThread()
{
	cv_.notify_one();
}

void AlsaAudioBackend::finalizePlayer(std::unique_ptr<Player>& player)
{
	// Recording not implemented on Linux - nothing to finalize
	(void)player;
}

std::unique_ptr<AudioBackend> CreateAlsaAudioBackend()
{
	return std::unique_ptr<AudioBackend>(new AlsaAudioBackend());
}

bool SupportsAlsaAudioBackend()
{
	return true;
}

#else

std::unique_ptr<AudioBackend> CreateAlsaAudioBackend()
{
	return nullptr;
}

bool SupportsAlsaAudioBackend()
{
	return false;
}

#endif
