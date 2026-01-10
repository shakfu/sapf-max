#pragma once

#include "VM.hpp"

struct SapfEngineConfig {
	double sampleRate = kDefaultSampleRate;
	const char* preludeFile = nullptr;
	const char* logFile = nullptr;
	bool enableManta = true;
};

class SapfEngine {
public:
	SapfEngine();

	void configure(const SapfEngineConfig& config);
	void initialize();
	void startMantaEventLoop() const;
	void loadPrelude(Thread& th) const;

	const char* versionString() const;
	const char* logFile() const;

private:
	void configureLogFile();
	void registerBuiltins();

	SapfEngineConfig config_;
	bool initialized_;
};

SapfEngine& GetSapfEngine();
const char* SapfGetVersionString();
