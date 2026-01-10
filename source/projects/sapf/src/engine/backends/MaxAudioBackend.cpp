/*
 * To use this from a Max external, call InstallMaxMSPBackend() during setup,
 * then invoke MaxMSPProcessAudio(outputs, numChannels, numFrames) inside your
 * MSP render callback to stream SAPF audio.
 */

#include "sapf/MaxAudioBackend.hpp"

#include <algorithm>
#include <cstring>
#include <pthread.h>

namespace {

const int kMaxChannels = 32;
pthread_mutex_t gMaxBackendMutex = PTHREAD_MUTEX_INITIALIZER;

} // namespace

struct MaxAudioBackend::Player {
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
};

MaxAudioBackend::MaxAudioBackend() = default;
MaxAudioBackend::~MaxAudioBackend() = default;

void MaxAudioBackend::play(Thread& th, V& v)
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
			return;
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

	Locker lock(&gMaxBackendMutex);
	players_.push_back(std::move(player));
}

void MaxAudioBackend::record(Thread& th, V& v, Arg filename)
{
	(void)th;
	(void)v;
	(void)filename;
	post("record is not supported in the Max/MSP backend.\n");
	throw errFailed;
}

void MaxAudioBackend::stopAll()
{
	Locker lock(&gMaxBackendMutex);
	players_.clear();
}

void MaxAudioBackend::stopFinished()
{
	Locker lock(&gMaxBackendMutex);
	removeFinishedLocked();
}

void MaxAudioBackend::render(float** outputs, int numChannels, int numFrames)
{
	if (numChannels <= 0 || numFrames <= 0) {
		return;
	}

	scratch_.resize(numFrames);

	Locker lock(&gMaxBackendMutex);
	for (int c = 0; c < numChannels; ++c) {
		std::memset(outputs[c], 0, static_cast<size_t>(numFrames) * sizeof(float));
	}

	auto it = players_.begin();
	while (it != players_.end()) {
		Player& player = *(*it);
		bool done = true;
		int channels = std::min(player.numChannels, numChannels);

		for (int ch = 0; ch < channels; ++ch) {
			int frames = numFrames;
			bool channelDone = player.in[ch].fill(player.th, frames, scratch_.data(), 1);
			for (int i = 0; i < frames; ++i) {
				outputs[ch][i] += scratch_[i];
			}
			if (frames < numFrames) {
				// Remaining samples already zeroed.
			}
			done = done && channelDone;
		}

		player.done = done;

		if (player.done) {
			it = players_.erase(it);
		} else {
			++it;
		}
	}
}

void MaxAudioBackend::removeFinishedLocked()
{
	auto it = players_.begin();
	while (it != players_.end()) {
		if ((*it)->done) {
			it = players_.erase(it);
		} else {
			++it;
		}
	}
}

MaxAudioBackend& InstallMaxMSPBackend()
{
	auto backend = std::make_unique<MaxAudioBackend>();
	MaxAudioBackend* raw = backend.get();
	SetAudioBackend(std::move(backend));
	return *raw;
}

void MaxMSPProcessAudio(float** outputs, int numChannels, int numFrames)
{
	if (!HasAudioBackend()) {
		return;
	}
	AudioBackend* base = nullptr;
	try {
		base = &GetAudioBackend();
	} catch (...) {
		return;
	}
	auto* backend = dynamic_cast<MaxAudioBackend*>(base);
	if (!backend) {
		return;
	}
	backend->render(outputs, numChannels, numFrames);
}
