#pragma once

#include <memory>

#include "sapf/AudioBackend.hpp"

std::unique_ptr<AudioBackend> CreateRtAudioBackend();
bool SupportsRtAudioBackend();
