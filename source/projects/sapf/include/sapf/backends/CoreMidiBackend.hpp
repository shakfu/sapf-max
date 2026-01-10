#pragma once

#include <memory>

#include "sapf/MidiBackend.hpp"

std::unique_ptr<MidiBackend> CreateCoreMidiBackend();
bool SupportsCoreMidiBackend();
