#pragma once

#include <memory>
#include <string>

#include "sapf/MidiBackend.hpp"

std::unique_ptr<MidiBackend> CreateNullMidiBackend(const std::string& reason = "");
