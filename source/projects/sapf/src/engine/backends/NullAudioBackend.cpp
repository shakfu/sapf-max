#include "sapf/backends/NullAudioBackend.hpp"

#include <utility>

namespace {

class NullAudioBackend : public AudioBackend
{
public:
	explicit NullAudioBackend(std::string message)
		: message_(std::move(message))
	{
	}

	void play(Thread&, V&) override
	{
		post("%s\n", message_.c_str());
		throw errFailed;
	}

	void record(Thread&, V&, Arg) override
	{
		post("%s\n", message_.c_str());
		throw errFailed;
	}

	void stopAll() override {}
	void stopFinished() override {}

private:
	std::string message_;
};

} // namespace

std::unique_ptr<AudioBackend> CreateNullAudioBackend(std::string message)
{
	return std::unique_ptr<AudioBackend>(new NullAudioBackend(std::move(message)));
}
