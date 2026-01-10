#include "sapf/Engine.hpp"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#if defined(__APPLE__)
#include <dispatch/dispatch.h>
#else
#include <thread>
#endif

#ifdef SAPF_USE_MANTA
#include "Manta.h"
#endif

#include "sapf/AudioBackend.hpp"

extern void AddCoreOps();
extern void AddMathOps();
extern void AddStreamOps();
extern void AddLFOps();
extern void AddUGenOps();
extern void AddSetOps();
extern void AddRandomOps();
extern void AddMidiOps();

namespace {

const char* gVersionString = "0.1.22";

#ifdef SAPF_USE_MANTA
class MyManta : public Manta
{
	void PadEvent(int row, int column, int id, int value) override {
		printf("pad %d %d %d %d\n", row, column, id, value);
	}
	void SliderEvent(int id, int value) override {
		printf("slider %d %d\n", id, value);
	}
	void ButtonEvent(int id, int value) override {
		printf("button %d %d\n", id, value);
	}
	void PadVelocityEvent(int row, int column, int id, int velocity) override {
		printf("pad vel %d %d %d %d\n", row, column, id, velocity);
	}
	void ButtonVelocityEvent(int id, int velocity) override {
		printf("button vel %d %d\n", id, velocity);
	}
	void FrameEvent(uint8_t *frame) override {}
	void DebugPrint(const char *fmt, ...) override {}
};

Manta* manta()
{
	static MyManta* sManta = new MyManta();
	return sManta;
}
#endif // SAPF_USE_MANTA

} // namespace

SapfEngine::SapfEngine()
	: config_()
	, initialized_(false)
{
}

void SapfEngine::configure(const SapfEngineConfig& config)
{
	config_ = config;
	if (config.sampleRate > 0.) {
		vm.setSampleRate(config.sampleRate);
	}
	if (config.preludeFile) {
		vm.prelude_file = config.preludeFile;
	}
	if (config.logFile) {
		vm.log_file = config.logFile;
	}
}

void SapfEngine::initialize()
{
	if (initialized_) {
		return;
	}
	registerBuiltins();
	configureLogFile();
	EnsureDefaultAudioBackend();
	initialized_ = true;
}

void SapfEngine::registerBuiltins()
{
	vm.addBifHelp("Argument Automapping legend:");
	vm.addBifHelp("   a - as is. argument is not automapped.");
	vm.addBifHelp("   z - argument is expected to be a signal or scalar, streams are auto mapped.");
	vm.addBifHelp("   k - argument is expected to be a scalar, signals and streams are automapped.");
	vm.addBifHelp("");

	AddCoreOps();
	AddMathOps();
	AddStreamOps();
	AddRandomOps();
	AddUGenOps();
	AddMidiOps();
	AddSetOps();
}

void SapfEngine::configureLogFile()
{
	if (vm.log_file) {
		return;
	}
	vm.log_file = getenv("SAPF_LOG");
	if (!vm.log_file) {
		const char* home_dir = getenv("HOME");
		static char logfilename[PATH_MAX];
		snprintf(logfilename, PATH_MAX, "%s/sapf-log.txt", home_dir);
		vm.log_file = logfilename;
	}
}

void SapfEngine::startMantaEventLoop() const
{
#ifdef SAPF_USE_MANTA
	if (!config_.enableManta) {
		return;
	}

	auto m = manta();
	try {
		m->Connect();
	} catch(...) {
	}
	printf("Manta %s connected.\n", m->IsConnected() ? "is" : "IS NOT");

#if defined(__APPLE__)
	dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
		while(true) {
			try {
				MantaUSB::HandleEvents();
				usleep(5000);
			} catch(...) {
				sleep(1);
			}
		}
	});
#else
	std::thread([]() {
		while(true) {
			try {
				MantaUSB::HandleEvents();
				usleep(5000);
			} catch(...) {
				sleep(1);
			}
		}
	}).detach();
#endif
#endif // SAPF_USE_MANTA
}

void SapfEngine::loadPrelude(Thread& th) const
{
	if (!vm.prelude_file) {
		vm.prelude_file = getenv("SAPF_PRELUDE");
	}
	if (vm.prelude_file) {
		loadFile(th, vm.prelude_file);
	}
}

const char* SapfEngine::versionString() const
{
	return gVersionString;
}

const char* SapfEngine::logFile() const
{
	return vm.log_file;
}

SapfEngine& GetSapfEngine()
{
	static SapfEngine engine;
	return engine;
}

const char* SapfGetVersionString()
{
	return gVersionString;
}
