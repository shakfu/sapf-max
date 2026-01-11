/*
 * To use this from a Max external, call InstallMaxMSPBackend() during setup,
 * then invoke MaxMSPProcessAudio(outputs, numChannels, numFrames) inside your
 * MSP render callback to stream SAPF audio.
 */

#include "sapf/MaxAudioBackend.hpp"
#include "Object.hpp"
#include "VM.hpp"

#include <algorithm>
#include <cstring>
#include <pthread.h>

namespace {

const int kMaxChannels = 32;
pthread_mutex_t gMaxBackendMutex = PTHREAD_MUTEX_INITIALIZER;

} // namespace

// ADC Generator - reads from Max audio input
class AdcGen : public Gen
{
	int mChannel;
public:
	AdcGen(Thread& th, int channel)
		: Gen(th, itemTypeZ, false)  // infinite stream
		, mChannel(channel)
	{
	}

	virtual const char* TypeName() const override { return "AdcGen"; }

	virtual void pull(Thread& th) override
	{
		MaxAudioBackend* backend = GetMaxAudioBackend();

		// Use the actual input buffer size from Max, not SAPF's mBlockSize
		// This ensures we produce exactly one audio callback's worth of samples
		int frames = (backend && backend->getInputFrames() > 0)
			? backend->getInputFrames()
			: mBlockSize;

		Z* out = mOut->fulfillz(frames);

		if (backend) {
			float** inputs = backend->getInputBuffers();
			int numChannels = backend->getInputChannels();

			if (inputs && mChannel < numChannels && inputs[mChannel]) {
				// Copy input samples (float to double conversion)
				for (int i = 0; i < frames; ++i) {
					out[i] = static_cast<Z>(inputs[mChannel][i]);
				}
			} else {
				// Channel not available - output silence
				std::memset(out, 0, frames * sizeof(Z));
			}
		} else {
			// No backend - output silence
			std::memset(out, 0, frames * sizeof(Z));
		}

		// Advance mOut to the next list node for subsequent pulls
		mOut = mOut->nextp();
	}
};

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
			sapf_post("Too many channels. Max is %d.\n", kMaxChannels);
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
	sapf_post("record is not supported in the Max/MSP backend.\n");
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

void MaxAudioBackend::setInputBuffers(float** inputs, int numChannels, int numFrames)
{
	inputBuffers_ = inputs;
	inputChannels_ = numChannels;
	inputFrames_ = numFrames;
}

void MaxMSPSetInputBuffers(float** inputs, int numChannels, int numFrames)
{
	auto* backend = GetMaxAudioBackend();
	if (backend) {
		backend->setInputBuffers(inputs, numChannels, numFrames);
	}
}

MaxAudioBackend* GetMaxAudioBackend()
{
	if (!HasAudioBackend()) {
		return nullptr;
	}
	AudioBackend* base = nullptr;
	try {
		base = &GetAudioBackend();
	} catch (...) {
		return nullptr;
	}
	return dynamic_cast<MaxAudioBackend*>(base);
}

// adc primitive: channel -> signal
// Reads audio from Max input at the specified channel (0-indexed)
static void adc_(Thread& th, Prim* prim)
{
	V channelArg = th.pop();
	int channel = static_cast<int>(channelArg.asInt());

	if (channel < 0) {
		sapf_post("adc: channel must be >= 0\n");
		throw errFailed;
	}

	P<List> result = new List(new AdcGen(th, channel));
	th.push(result);
}

// adc primitive for multichannel: numChannels -> [signals...]
// Returns a list of signals for channels 0 to numChannels-1
static void adcn_(Thread& th, Prim* prim)
{
	V numArg = th.pop();
	int numChannels = static_cast<int>(numArg.asInt());

	if (numChannels <= 0) {
		sapf_post("adcn: numChannels must be > 0\n");
		throw errFailed;
	}

	P<List> result = new List(itemTypeV, numChannels);
	for (int i = 0; i < numChannels; ++i) {
		P<List> channelSignal = new List(new AdcGen(th, i));
		result->add(channelSignal);
	}
	th.push(result);
}

void RegisterMaxAudioPrimitives()
{
	vm.def("adc", new Prim(adc_, V(), 1, 1, "adc", "channel -- signal. Read audio input from Max."));
	vm.def("adcn", new Prim(adcn_, V(), 1, 1, "adcn", "numChannels -- [signals...]. Read multiple audio input channels from Max."));
}
