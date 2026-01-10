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

#include "elapsedTime.hpp"

#if defined(__APPLE__)
#include <mach/mach_time.h>

static double gHostClockFreq;

extern "C" {

void initElapsedTime()
{
	struct mach_timebase_info info;
	mach_timebase_info(&info);
	gHostClockFreq = 1e9 * ((double)info.numer / (double)info.denom);
}

double elapsedTime()
{
	return (double)mach_absolute_time() / gHostClockFreq;
}

}

#else
// Linux/other platforms using clock_gettime
#include <time.h>

extern "C" {

void initElapsedTime()
{
	// No initialization needed for clock_gettime
}

double elapsedTime()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

}

#endif
