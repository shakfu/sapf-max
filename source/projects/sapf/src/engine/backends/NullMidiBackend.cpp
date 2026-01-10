//    SAPF - Sound As Pure Form
//    Copyright (C) 2019 James McCartney
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "sapf/backends/NullMidiBackend.hpp"

#include "MidiRouter.hpp"
#include "ErrorCodes.hpp"
#include <cstdio>

namespace {

class NullMidiBackend : public MidiBackend
{
public:
	explicit NullMidiBackend(const std::string& reason) : reason_(reason) {}
	~NullMidiBackend() override = default;

	int initialize(int numIn, int numOut) override
	{
		if (!reason_.empty()) {
			printf("NullMidiBackend: %s\n", reason_.c_str());
		}
		getMidiRouter().resetState();
		return errNone;
	}

	void cleanup() override {}
	void restart() override {}

	void listDevices() override
	{
		printf("midi sources 0 destinations 0\n");
		if (!reason_.empty()) {
			printf("(NullMidiBackend: %s)\n", reason_.c_str());
		}
	}

	int connectInput(int /*uid*/, int /*portIndex*/) override
	{
		return errNone;
	}

	int disconnectInput(int /*uid*/, int /*portIndex*/) override
	{
		return errNone;
	}

	void sendMessage(int /*port*/, int /*destIndex*/, const std::vector<unsigned char>& /*message*/, float /*latencySeconds*/) override
	{
		// No-op
	}

private:
	std::string reason_;
};

} // namespace

std::unique_ptr<MidiBackend> CreateNullMidiBackend(const std::string& reason)
{
	return std::make_unique<NullMidiBackend>(reason);
}
