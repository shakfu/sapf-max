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

#include "VM.hpp"
#include <stdio.h>
#include <histedit.h>
#include <algorithm>
#include <sys/stat.h>
#include "primes.hpp"
#include <complex>
#include <dispatch/dispatch.h>
#include <CoreFoundation/CoreFoundation.h>
#include "Manta.h"

class MyManta : public Manta
{
	virtual void PadEvent(int row, int column, int id, int value) {
		printf("pad %d %d %d %d\n", row, column, id, value);
	}
	virtual void SliderEvent(int id, int value) {
		printf("slider %d %d\n", id, value);
	}
	virtual void ButtonEvent(int id, int value) {
		printf("button %d %d\n", id, value);
	}
	virtual void PadVelocityEvent(int row, int column, int id, int velocity) {
		printf("pad vel %d %d %d %d\n", row, column, id, velocity);

	}
	virtual void ButtonVelocityEvent(int id, int velocity) {
		printf("button vel %d %d\n", id, velocity);
	}
	virtual void FrameEvent(uint8_t *frame) {}
	virtual void DebugPrint(const char *fmt, ...) {}
};

Manta* manta();
Manta* manta()
{
	static MyManta* sManta = new MyManta();
	return sManta;
}

/* issue:

[These comments are very old and I have not checked if they are still relevant.]

TableData alloc should use new

bugs:

itd should have a tail time. currently the ugen stops as soon as its input, cutting off the delayed signal.

+ should not stop until both inputs stop?
other additive binops: - avg2 sumsq

no, use a  operator 

---

adsrg (gate a d s r --> out) envelope generator with gate. 
adsr (dur a d s r --> out) envelope generator with duration. 
evgg - (gate levels times curves suspt --> out) envelope generator with gate. suspt is the index of the sustain level.  
evg - (dur levels times curves suspt --> out) envelope generator with duration. suspt is the index of the sustain level.

blip (freq phase nharm --> out) band limited impulse oscillator.
dsf1 (freq phase nharm lharm hmul --> out) sum of sines oscillator.

formant (freq formfreq bwfreq --> out) formant oscillator

svf (in freq rq --> [lp hp bp bs]) state variable filter.
moogf (in freq rq --> out) moog ladder low pass filter.

*/

extern void AddCoreOps();
extern void AddMathOps();
extern void AddStreamOps();
extern void AddLFOps();
extern void AddUGenOps();
extern void AddSetOps();
extern void AddRandomOps();
extern void AddMidiOps();

const char* gVersionString = "0.1.21";

static void usage()
{
	fprintf(stdout, "sapf [-r sample-rate][-p prelude-file]\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "sapf [-h]\n");
	fprintf(stdout, "    print this help\n");
	fprintf(stdout, "\n");	
}

int main (int argc, const char * argv[]) 
{
	post("------------------------------------------------\n");	
	post("A tool for the expression of sound as pure form.\n");	
	post("------------------------------------------------\n");	
	post("--- version %s\n", gVersionString);
	
	for (int i = 1; i < argc;) {
		int c = argv[i][0];
		if (c == '-') {
			c = argv[i][1];
			switch (c) {
				case 'r' : {
					if (argc <= i+1) { post("expected sample rate after -r\n"); return 1; }
						
					double sr = atof(argv[i+1]);
					if (sr < 1000. || sr > 768000.) { post("sample rate out of range.\n"); return 1; }
					vm.setSampleRate(sr);
					post("sample rate set to %g\n", vm.ar.sampleRate);
					i += 2;
				} break;
				case 'p' : {
					if (argc <= i+1) { post("expected prelude file name after -p\n"); return 1; }
					vm.prelude_file = argv[i+1];
					i += 2;
				} break;
				case 'h' : {
					usage();
					exit(0);
				} break;
				default: 
					post("unrecognized option -%c\n", c);
			}
		} else {
			post("expected option, got \"%s\"\n", argv[i]);
			++i;
		}
	}
	
	
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
	
	
	vm.log_file = getenv("SAPF_LOG");
	if (!vm.log_file) {
		const char* home_dir = getenv("HOME");
		char logfilename[PATH_MAX];
		snprintf(logfilename, PATH_MAX, "%s/sapf-log.txt", home_dir);
		vm.log_file = strdup(logfilename);
	}
	
	__block Thread th;

	auto m = manta();
	try {
		m->Connect();
	} catch(...) {
	}
	printf("Manta %s connected.\n", m->IsConnected() ? "is" : "IS NOT");

	dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
		/*** see at bottom for better way ***/
		while(true) {
			try {
				MantaUSB::HandleEvents();
				usleep(5000);
			} catch(...) {
				sleep(1);
			}
		}
	});
	
	if (!vm.prelude_file) {
		vm.prelude_file = getenv("SAPF_PRELUDE");
	}
	if (vm.prelude_file) {
		loadFile(th, vm.prelude_file);
	}
	
	dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
		th.repl(stdin, vm.log_file);
		exit(0);
	});
	
	CFRunLoopRun();
	
	return 0;
}

