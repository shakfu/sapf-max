#pragma once

#include <memory>
#include <string>

#include "sapf/AudioBackend.hpp"

std::unique_ptr<AudioBackend> CreateNullAudioBackend(std::string message);
