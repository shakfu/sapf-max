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

#include "DelayUGens.hpp"
#include "UGen.hpp"

#include "VM.hpp"
#include "clz.hpp"
#include "primes.hpp"
#include <cmath>
#include <float.h>
#include <vector>
#include <algorithm>
#include <Accelerate/Accelerate.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class DelayN : public Gen
{
	ZIn in_;
	ZIn delay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
public:

	DelayN(Thread& th, Arg in, Arg delay, Z maxdelay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		bufSize = NEXTPOWEROFTWO((int32_t)ceil(sr * maxdelay));
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~DelayN() { free(buf); }
	
	virtual const char* TypeName() const override { return "DelayN"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride;
			Z *in, *delay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay)) {
				setDone();
				break;
			} else {
                for (int i = 0; i < n; ++i) {
					Z zdelay = *delay;
					zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                    int32_t offset = std::max(1,(int32_t)floor(zdelay * sr + .5));
                    out[i] = buf[(bufPos-offset) & bufMask];
                    buf[bufPos & bufMask] = *in;
                    in += inStride;
                    delay += delayStride;
                    ++bufPos;
                }
				in_.advance(n);
				delay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void delayn_(Thread& th, Prim* prim)
{
	Z maxdelay = th.popFloat("delayn : maxdelay");
	V delay = th.popZIn("delayn : delay");
	V in = th.popZIn("delayn : in");
	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("delayn : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
    
	th.push(new List(new DelayN(th, in, delay, maxdelay)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

class DelayL : public Gen
{
	ZIn in_;
	ZIn delay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
public:
	
	DelayL(Thread& th, Arg in, Arg delay, Z maxdelay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		bufSize = NEXTPOWEROFTWO((int32_t)ceil(sr * maxdelay));
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~DelayL() { free(buf); }
	
	virtual const char* TypeName() const override { return "DelayL"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride;
			Z *in, *delay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay)) {
				setDone();
				break;
			} else {
                if (delayStride == 0) {
					Z zdelay = *delay;
					zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                    Z fpos = std::max(1., zdelay * sr);
                    Z ipos = floor(fpos);
                    Z frac = fpos - ipos;
                    int32_t offset = (int32_t)ipos;
                    for (int i = 0; i < n; ++i) {
                        int32_t offset2 = bufPos-offset;
                        Z a = buf[(offset2) & bufMask];
                        Z b = buf[(offset2-1) & bufMask];
                        out[i] = a + frac * (b - a);
                        buf[bufPos & bufMask] = *in;
                        in += inStride;
                        ++bufPos;
                    }
                } else {
                    for (int i = 0; i < n; ++i) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                        Z fpos = std::max(1., zdelay * sr);
                        Z ipos = floor(fpos);
                        Z frac = fpos - ipos;
                        int32_t offset = bufPos-(int32_t)ipos;
                        Z a = buf[(offset) & bufMask];
                        Z b = buf[(offset-1) & bufMask];
                        out[i] = a + frac * (b - a);
                        buf[bufPos & bufMask] = *in;
                        in += inStride;
                        delay += delayStride;
                        ++bufPos;
                    }
                }
				in_.advance(n);
				delay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void delayl_(Thread& th, Prim* prim)
{
	Z maxdelay = th.popFloat("delayl : maxdelay");
	V delay = th.popZIn("delayl : delay");
	V in = th.popZIn("delayl : in");
 	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("delayl : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
   
	th.push(new List(new DelayL(th, in, delay, maxdelay)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

class DelayC : public Gen
{
	ZIn in_;
	ZIn delay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
public:
	
	DelayC(Thread& th, Arg in, Arg delay, Z maxdelay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		bufSize = NEXTPOWEROFTWO((int32_t)ceil(sr * maxdelay));
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~DelayC() { free(buf); }
	
	virtual const char* TypeName() const override { return "DelayC"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride;
			Z *in, *delay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay)) {
				setDone();
				break;
			} else {
                if (delayStride == 0) {
					Z zdelay = *delay;
					zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                    Z fpos = std::max(2., zdelay * sr);
                    Z ipos = floor(fpos);
                    Z frac = fpos - ipos;
                    int32_t offset = (int32_t)ipos;
                    for (int i = 0; i < n; ++i) {
                        int32_t offset2 = bufPos-offset;
                        Z a = buf[(offset2+1) & bufMask];
                        Z b = buf[(offset2  ) & bufMask];
                        Z c = buf[(offset2-1) & bufMask];
                        Z d = buf[(offset2-2) & bufMask];
                        out[i] = lagrangeInterpolate(frac, a, b, c, d);
                        buf[bufPos & bufMask] = *in;
                        in += inStride;
                        ++bufPos;
                    }
                } else {
                    for (int i = 0; i < n; ++i) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                        Z fpos = std::max(2., zdelay * sr);
                        Z ipos = floor(fpos);
                        Z frac = fpos - ipos;
                        int32_t offset = bufPos-(int32_t)ipos;
                        Z a = buf[(offset+1) & bufMask];
                        Z b = buf[(offset  ) & bufMask];
                        Z c = buf[(offset-1) & bufMask];
                        Z d = buf[(offset-2) & bufMask];
                        out[i] = lagrangeInterpolate(frac, a, b, c, d);
                        buf[bufPos & bufMask] = *in;
                        in += inStride;
                        delay += delayStride;
                        ++bufPos;
                    }
                }
				in_.advance(n);
				delay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void delayc_(Thread& th, Prim* prim)
{
	Z maxdelay = th.popFloat("delayc : maxdelay");
	V delay = th.popZIn("delayc : delay");
	V in = th.popZIn("delayc : in");
	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("delayc : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
    
	th.push(new List(new DelayC(th, in, delay, maxdelay)));
}


////////////////////////////////////////////////////////////////////////////////////////////////////////

class Flange : public Gen
{
	ZIn in_;
	ZIn delay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	int32_t half;
    Z fhalf;
	Z* buf;
	Z sr;
public:
	
	Flange(Thread& th, Arg in, Arg delay, Z maxdelay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		fhalf = ceil(sr * maxdelay + .5);
		half = (int32_t)fhalf;
		bufSize = NEXTPOWEROFTWO(2 * half);
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~Flange() { free(buf); }
	
	virtual const char* TypeName() const override { return "Flange"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride;
			Z *in, *delay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay)) {
				setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					Z zdelay = *delay;
					zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
					Z fpos = std::max(2., zdelay * sr + fhalf);
					Z ipos = floor(fpos);
					Z frac = fpos - ipos;
					int32_t offset = bufPos-(int32_t)ipos;
					Z a = buf[(offset+1) & bufMask];
					Z b = buf[(offset  ) & bufMask];
					Z c = buf[(offset-1) & bufMask];
					Z d = buf[(offset-2) & bufMask];
					Z zin = buf[(bufPos-half) & bufMask];
					out[i] = lagrangeInterpolate(frac, a, b, c, d) - zin;
					buf[bufPos & bufMask] = *in;
					in += inStride;
					delay += delayStride;
					++bufPos;
				}
				in_.advance(n);
				delay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

class Flangep : public Gen
{
	ZIn in_;
	ZIn delay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	int32_t half;
    Z fhalf;
	Z* buf;
	Z sr;
public:
	
	Flangep(Thread& th, Arg in, Arg delay, Z maxdelay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		fhalf = ceil(sr * maxdelay + .5);
		half = (int32_t)fhalf;
		bufSize = NEXTPOWEROFTWO(2 * half);
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~Flangep() { free(buf); }
	
	virtual const char* TypeName() const override { return "Flangep"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride;
			Z *in, *delay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay)) {
				setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					Z zdelay = *delay;
					zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
					Z fpos = std::max(2., zdelay * sr + fhalf);
					Z ipos = floor(fpos);
					Z frac = fpos - ipos;
					int32_t offset = bufPos-(int32_t)ipos;
					Z a = buf[(offset+1) & bufMask];
					Z b = buf[(offset  ) & bufMask];
					Z c = buf[(offset-1) & bufMask];
					Z d = buf[(offset-2) & bufMask];
					Z zin = buf[(bufPos-half) & bufMask];
					out[i] = lagrangeInterpolate(frac, a, b, c, d) + zin;
					buf[bufPos & bufMask] = *in;
					in += inStride;
					delay += delayStride;
					++bufPos;
				}
				in_.advance(n);
				delay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void flange_(Thread& th, Prim* prim)
{
	Z maxdelay = th.popFloat("flange : maxdelay");
	V delay = th.popZIn("flange : delay");
	V in = th.popZIn("flange : in");
	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("flange : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
    
	th.push(new List(new Flange(th, in, delay, maxdelay)));
}

static void flangep_(Thread& th, Prim* prim)
{
	Z maxdelay = th.popFloat("flangep : maxdelay");
	V delay = th.popZIn("flangep : delay");
	V in = th.popZIn("flangep : in");
	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("flangep : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
    
	th.push(new List(new Flangep(th, in, delay, maxdelay)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////


class CombN : public Gen
{
	ZIn in_;
	ZIn delay_;
	ZIn decay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
public:
	
	CombN(Thread& th, Arg in, Arg delay, Z maxdelay, Arg decay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), decay_(decay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		bufSize = NEXTPOWEROFTWO((int32_t)ceil(sr * maxdelay + 1.));
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~CombN() { free(buf); }
	
	virtual const char* TypeName() const override { return "CombN"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride, decayStride;
			Z *in, *delay, *decay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay) || decay_(th, n, decayStride, decay)) {
				setDone();
				break;
			} else {
				if (decayStride == 0) {
					double rdecay = 1. / *decay;
					for (int i = 0; i < n; ++i) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay * rdecay);
						int32_t offset = std::max(1,(int32_t)floor(std::abs(zdelay) * sr + .5));
						Z z = fb * buf[(bufPos-offset) & bufMask];
						out[i] = z;
						buf[bufPos & bufMask] = *in + z;
						in += inStride;
						delay += delayStride;
						++bufPos;
					}
				} else {
					for (int i = 0; i < n; ++i) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay / *decay);
						int32_t offset = (int32_t)floor(std::abs(zdelay) * sr + .5);
						Z z = fb * buf[(bufPos-offset) & bufMask];
						out[i] = z;
						buf[bufPos & bufMask] = *in + z;
						in += inStride;
						delay += delayStride;
						decay += decayStride;
						++bufPos;
					}
				}
				in_.advance(n);
				delay_.advance(n);
				decay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void combn_(Thread& th, Prim* prim)
{
	V decay = th.popZIn("combn : decay");
	Z maxdelay = th.popFloat("combn : maxdelay");
	V delay = th.popZIn("combn : delay");
	V in = th.popZIn("combn : in");
	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("combn : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
    
	th.push(new List(new CombN(th, in, delay, maxdelay, decay)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

class CombL : public Gen
{
	ZIn in_;
	ZIn delay_;
	ZIn decay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
public:
	
	CombL(Thread& th, Arg in, Arg delay, Z maxdelay, Arg decay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), decay_(decay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		bufSize = NEXTPOWEROFTWO((int32_t)ceil(sr * maxdelay));
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~CombL() { free(buf); }
	
	virtual const char* TypeName() const override { return "CombL"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride, decayStride;
			Z *in, *delay, *decay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay) || decay_(th, n, decayStride, decay)) {
				setDone();
				break;
			} else {
				if (decayStride == 0) {
                    if (delayStride == 0) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay / *decay);
                        Z fpos = std::max(1., zdelay * sr);
                        Z ipos = floor(fpos);
                        Z frac = fpos - ipos;
                        int32_t offset = (int32_t)ipos;
                        for (int i = 0; i < n; ++i) {
							int32_t offset2 = bufPos-offset;
                            Z a = buf[(offset2) & bufMask];
                            Z b = buf[(offset2-1) & bufMask];
                            Z z = fb * (a + frac * (b - a));
                            out[i] = z;
                            buf[bufPos & bufMask] = *in + z;
                            in += inStride;
                            ++bufPos;
                        }
                    } else {
                        double rdecay = 1. / *decay;
                        for (int i = 0; i < n; ++i) {
							Z zdelay = *delay;
							zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                            Z fb = calcDecay(zdelay * rdecay);
                            Z fpos = std::max(1., zdelay * sr);
                            Z ipos = floor(fpos);
                            Z frac = fpos - ipos;
                            int32_t offset = bufPos-(int32_t)ipos;
                            Z a = buf[offset & bufMask];
                            Z b = buf[(offset-1) & bufMask];
                            Z z = fb * (a + frac * (b - a));
                            out[i] = z;
                            buf[bufPos & bufMask] = *in + z;
                            in += inStride;
                            delay += delayStride;
                            ++bufPos;
                        }
                    }
				} else {
					for (int i = 0; i < n; ++i) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay / *decay);
                        Z fpos = std::max(1., zdelay * sr);
                        Z ipos = floor(fpos);
                        Z frac = fpos - ipos;
						int32_t offset = bufPos-(int32_t)ipos;
                        Z a = buf[offset & bufMask];
                        Z b = buf[(offset-1) & bufMask];
						Z z = fb * (a + frac * (b - a));
						out[i] = z;
						buf[bufPos & bufMask] = *in + z;
						in += inStride;
						delay += delayStride;
						++bufPos;
					}
				}
				in_.advance(n);
				delay_.advance(n);
				decay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void combl_(Thread& th, Prim* prim)
{
	V decay = th.popZIn("combl : decay");
	Z maxdelay = th.popFloat("combl : maxdelay");
	V delay = th.popZIn("combl : delay");
	V in = th.popZIn("combl : in");
	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("combl : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
    
	th.push(new List(new CombL(th, in, delay, maxdelay, decay)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

class CombC : public Gen
{
	ZIn in_;
	ZIn delay_;
	ZIn decay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
public:
	
	CombC(Thread& th, Arg in, Arg delay, Z maxdelay, Arg decay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), decay_(decay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		bufSize = NEXTPOWEROFTWO((int32_t)ceil(sr * maxdelay));
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~CombC() { free(buf); }
	
	virtual const char* TypeName() const override { return "CombC"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride, decayStride;
			Z *in, *delay, *decay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay) || decay_(th, n, decayStride, decay)) {
				setDone();
				break;
			} else {
				if (decayStride == 0) {
                    if (delayStride == 0) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay / *decay);
                        Z fpos = std::max(2., zdelay * sr);
                        Z ipos = floor(fpos);
                        Z frac = fpos - ipos;
                        int32_t offset = (int32_t)ipos;
                        for (int i = 0; i < n; ++i) {
							int32_t offset2 = bufPos-offset;
							Z a = buf[(offset2+1) & bufMask];
							Z b = buf[(offset2  ) & bufMask];
							Z c = buf[(offset2-1) & bufMask];
							Z d = buf[(offset2-2) & bufMask];
							Z z = fb * lagrangeInterpolate(frac, a, b, c, d);
                            out[i] = z;
                            buf[bufPos & bufMask] = *in + z;
                            in += inStride;
                            ++bufPos;
                        }
                    } else {
                        double rdecay = 1. / *decay;
                        for (int i = 0; i < n; ++i) {
							Z zdelay = *delay;
							zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                            Z fb = calcDecay(zdelay * rdecay);
                            Z fpos = std::max(2., zdelay * sr);
                            Z ipos = floor(fpos);
                            Z frac = fpos - ipos;
							int32_t offset = bufPos-(int32_t)ipos;
							Z a = buf[(offset+1) & bufMask];
							Z b = buf[(offset  ) & bufMask];
							Z c = buf[(offset-1) & bufMask];
							Z d = buf[(offset-2) & bufMask];
							Z z = fb * lagrangeInterpolate(frac, a, b, c, d);
                            out[i] = z;
                            buf[bufPos & bufMask] = *in + z;
                            in += inStride;
                            delay += delayStride;
                            ++bufPos;
                        }
                    }
				} else {
					for (int i = 0; i < n; ++i) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay / *decay);
                        Z fpos = std::max(2., zdelay * sr);
                        Z ipos = floor(fpos);
                        Z frac = fpos - ipos;
						int32_t offset = bufPos-(int32_t)ipos;
                        Z a = buf[(offset+1) & bufMask];
                        Z b = buf[(offset  ) & bufMask];
                        Z c = buf[(offset-1) & bufMask];
                        Z d = buf[(offset-2) & bufMask];
						Z z = fb * lagrangeInterpolate(frac, a, b, c, d);
						out[i] = z;
						buf[bufPos & bufMask] = *in + z;
						in += inStride;
						delay += delayStride;
						++bufPos;
					}
				}
				in_.advance(n);
				delay_.advance(n);
				decay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void combc_(Thread& th, Prim* prim)
{
	V decay = th.popZIn("combc : decay");
	Z maxdelay = th.popFloat("combc : maxdelay");
	V delay = th.popZIn("combc : delay");
	V in = th.popZIn("combc : in");
	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("combc : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
    
	th.push(new List(new CombC(th, in, delay, maxdelay, decay)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

class LPCombC : public Gen
{
	ZIn in_;
	ZIn delay_;
	ZIn decay_;
	ZIn lpfreq_;
	Z maxdelay_;
	Z y1_;
	Z freqmul_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
public:
	
	LPCombC(Thread& th, Arg in, Arg delay, Z maxdelay, Arg decay, Arg lpfreq) : Gen(th, itemTypeZ, false),
			in_(in), delay_(delay), decay_(decay), lpfreq_(lpfreq), maxdelay_(maxdelay), y1_(0.), freqmul_(th.rate.invNyquistRate * kFirstOrderCoeffScale)
	{
		sr = th.rate.sampleRate;
		bufSize = NEXTPOWEROFTWO((int32_t)ceil(sr * maxdelay));
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~LPCombC() { free(buf); }
	
	virtual const char* TypeName() const override { return "LPCombC"; }
    
	virtual void pull(Thread& th) override 
	{
		Z y1 = y1_;
		Z freqmul = freqmul_;
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride, decayStride, lpfreqStride;
			Z *in, *delay, *decay, *lpfreq;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay) || decay_(th, n, decayStride, decay) || lpfreq_(th, n, lpfreqStride, lpfreq)) {
				setDone();
				break;
			} else {
                if (lpfreqStride == 0) {
                    Z b1 = t_firstOrderCoeff(*lpfreq * freqmul);
                    if (decayStride == 0) {
                        if (delayStride == 0) {
							Z zdelay = *delay;
							zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                            Z fb = calcDecay(zdelay / *decay);
                            Z fpos = std::max(2., zdelay * sr);
                            Z ipos = floor(fpos);
                            Z frac = fpos - ipos;
                            int32_t offset = (int32_t)ipos;
                            for (int i = 0; i < n; ++i) {
                                int32_t offset2 = bufPos-offset;
                                Z a = buf[(offset2+1) & bufMask];
                                Z b = buf[(offset2  ) & bufMask];
                                Z c = buf[(offset2-1) & bufMask];
                                Z d = buf[(offset2-2) & bufMask];
                                Z z = fb * lagrangeInterpolate(frac, a, b, c, d);
                                z = z + b1 * (y1 - z);

                                out[i] = y1 = z;
                                buf[bufPos & bufMask] = *in + z;
                                in += inStride;
                                ++bufPos;
                            }
                        } else {
                            double rdecay = 1. / *decay;
                            for (int i = 0; i < n; ++i) {
								Z zdelay = *delay;
								zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                                Z fb = calcDecay(zdelay * rdecay);
                                Z fpos = std::max(2., zdelay * sr);
                                Z ipos = floor(fpos);
                                Z frac = fpos - ipos;
                                int32_t offset = bufPos-(int32_t)ipos;
                                Z a = buf[(offset+1) & bufMask];
                                Z b = buf[(offset  ) & bufMask];
                                Z c = buf[(offset-1) & bufMask];
                                Z d = buf[(offset-2) & bufMask];
                                Z z = fb * lagrangeInterpolate(frac, a, b, c, d);
                                z = z + b1 * (y1 - z);
                                out[i] = y1 = z;
                                buf[bufPos & bufMask] = *in + z;
                                in += inStride;
                                delay += delayStride;
                                ++bufPos;
                            }
                        }
                    } else {
                        for (int i = 0; i < n; ++i) {
							Z zdelay = *delay;
							zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                            Z fb = calcDecay(zdelay / *decay);
                            Z fpos = std::max(2., zdelay * sr);
                            Z ipos = floor(fpos);
                            Z frac = fpos - ipos;
                            int32_t offset = bufPos-(int32_t)ipos;
                            Z a = buf[(offset+1) & bufMask];
                            Z b = buf[(offset  ) & bufMask];
                            Z c = buf[(offset-1) & bufMask];
                            Z d = buf[(offset-2) & bufMask];
                            Z z = fb * lagrangeInterpolate(frac, a, b, c, d);
                            z = z + b1 * (y1 - z);
                            out[i] = y1 = z;
                            buf[bufPos & bufMask] = *in + z;
                            in += inStride;
                            delay += delayStride;
                            ++bufPos;
                        }
                    }
               } else {
                    if (decayStride == 0) {
                        if (delayStride == 0) {
							Z zdelay = *delay;
							zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                            Z fb = calcDecay(zdelay / *decay);
                            Z fpos = std::max(2., zdelay * sr);
                            Z ipos = floor(fpos);
                            Z frac = fpos - ipos;
                            int32_t offset = (int32_t)ipos;
                            for (int i = 0; i < n; ++i) {
                                int32_t offset2 = bufPos-offset;
                                Z a = buf[(offset2+1) & bufMask];
                                Z b = buf[(offset2  ) & bufMask];
                                Z c = buf[(offset2-1) & bufMask];
                                Z d = buf[(offset2-2) & bufMask];
                                Z z = fb * lagrangeInterpolate(frac, a, b, c, d);
                                Z b1 = t_firstOrderCoeff(*lpfreq * freqmul);
                                z = z + b1 * (y1 - z);
                                out[i] = y1 = z;
                                buf[bufPos & bufMask] = *in + z;
                                in += inStride;
                                lpfreq += lpfreqStride;
                                ++bufPos;
                            }
                        } else {
                            double rdecay = 1. / *decay;
                            for (int i = 0; i < n; ++i) {
								Z zdelay = *delay;
								zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                                Z fb = calcDecay(zdelay * rdecay);
                                Z fpos = std::max(2., zdelay * sr);
                                Z ipos = floor(fpos);
                                Z frac = fpos - ipos;
                                int32_t offset = bufPos-(int32_t)ipos;
                                Z a = buf[(offset+1) & bufMask];
                                Z b = buf[(offset  ) & bufMask];
                                Z c = buf[(offset-1) & bufMask];
                                Z d = buf[(offset-2) & bufMask];
                                Z z = fb * lagrangeInterpolate(frac, a, b, c, d);
                                Z b1 = t_firstOrderCoeff(*lpfreq * freqmul);
                                z = z + b1 * (y1 - z);
                                out[i] = y1 = z;
                                buf[bufPos & bufMask] = *in + z;
                                in += inStride;
                                delay += delayStride;
                                lpfreq += lpfreqStride;
                                ++bufPos;
                            }
                        }
                    } else {
                        for (int i = 0; i < n; ++i) {
							Z zdelay = *delay;
							zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                            Z fb = calcDecay(zdelay / *decay);
                            Z fpos = std::max(2., zdelay * sr);
                            Z ipos = floor(fpos);
                            Z frac = fpos - ipos;
                            int32_t offset = bufPos-(int32_t)ipos;
                            Z a = buf[(offset+1) & bufMask];
                            Z b = buf[(offset  ) & bufMask];
                            Z c = buf[(offset-1) & bufMask];
                            Z d = buf[(offset-2) & bufMask];
                            Z z = fb * lagrangeInterpolate(frac, a, b, c, d);
                            Z b1 = t_firstOrderCoeff(*lpfreq * freqmul);
                            z = z + b1 * (y1 - z);
                            out[i] = y1 = z;
                            buf[bufPos & bufMask] = *in + z;
                            in += inStride;
                            delay += delayStride;
                            lpfreq += lpfreqStride;
                            ++bufPos;
                        }
                    }
                }
				y1_ = y1;
				in_.advance(n);
				delay_.advance(n);
				decay_.advance(n);
				lpfreq_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void lpcombc_(Thread& th, Prim* prim)
{
	V lpfreq = th.popZIn("lpcombc : lpfreq");
	V decay = th.popZIn("lpcombc : decay");
	Z maxdelay = th.popFloat("lpcombc : maxdelay");
	V delay = th.popZIn("lpcombc : delay");
	V in = th.popZIn("lpcombc : in");
	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("lpcombc : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
    
	th.push(new List(new LPCombC(th, in, delay, maxdelay, decay, lpfreq)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////


class AllpassN : public Gen
{
	ZIn in_;
	ZIn delay_;
	ZIn decay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
public:
	
	AllpassN(Thread& th, Arg in, Arg delay, Z maxdelay, Arg decay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), decay_(decay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		bufSize = NEXTPOWEROFTWO((int32_t)ceil(sr * maxdelay));
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~AllpassN() { free(buf); }
	
	virtual const char* TypeName() const override { return "AllpassN"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride, decayStride;
			Z *in, *delay, *decay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay) || decay_(th, n, decayStride, decay)) {
				setDone();
				break;
			} else {
				if (decayStride == 0) {
					double rdecay = 1. / *decay;
					for (int i = 0; i < n; ++i) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay * rdecay);
						int32_t offset = std::max(1,(int32_t)floor(zdelay * sr + .5));
                        Z drd = buf[(bufPos-offset) & bufMask];
                        Z dwr = drd * fb + *in;
						buf[bufPos & bufMask] = dwr;
						out[i] = drd - fb * dwr;
						in += inStride;
						delay += delayStride;
						++bufPos;
					}
				} else {
					for (int i = 0; i < n; ++i) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay / *decay);
						int32_t offset = (int32_t)floor(zdelay * sr + .5);
                        Z drd = buf[(bufPos-offset) & bufMask];
                        Z dwr = drd * fb + *in;
						buf[bufPos & bufMask] = dwr;
						out[i] = drd - fb * dwr;
						in += inStride;
						delay += delayStride;
						decay += decayStride;
						++bufPos;
					}
				}
				in_.advance(n);
				delay_.advance(n);
				decay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void alpasn_(Thread& th, Prim* prim)
{
	V decay = th.popZIn("alpasn : decay");
	Z maxdelay = th.popFloat("alpasn : maxdelay");
	V delay = th.popZIn("alpasn : delay");
	V in = th.popZIn("alpasn : in");
	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("alpasn : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
    
	th.push(new List(new AllpassN(th, in, delay, maxdelay, decay)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

class AllpassL : public Gen
{
	ZIn in_;
	ZIn delay_;
	ZIn decay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
public:
	
	AllpassL(Thread& th, Arg in, Arg delay, Z maxdelay, Arg decay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), decay_(decay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		bufSize = NEXTPOWEROFTWO((int32_t)ceil(sr * maxdelay));
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~AllpassL() { free(buf); }
	
	virtual const char* TypeName() const override { return "AllpassL"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride, decayStride;
			Z *in, *delay, *decay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay) || decay_(th, n, decayStride, decay)) {
				setDone();
				break;
			} else {
				if (decayStride == 0) {
                    if (delayStride == 0) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay / *decay);
                        Z fpos = std::max(1., zdelay * sr);
                        Z ipos = floor(fpos);
                        Z frac = fpos - ipos;
                        int32_t offset = (int32_t)ipos;
                        for (int i = 0; i < n; ++i) {
							int32_t offset2 = bufPos-offset;
                            Z a = buf[(offset2) & bufMask];
                            Z b = buf[(offset2-1) & bufMask];
                            Z drd = a + frac * (b - a);
                            Z dwr = drd * fb + *in;
                            buf[bufPos & bufMask] = dwr;
                            out[i] = drd - fb * dwr;
                            in += inStride;
                            ++bufPos;
                        }
                    } else {
                        double rdecay = 1. / *decay;
                        for (int i = 0; i < n; ++i) {
							Z zdelay = *delay;
							zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                            Z fb = calcDecay(zdelay * rdecay);
                            Z fpos = std::max(1., zdelay * sr);
                            Z ipos = floor(fpos);
                            Z frac = fpos - ipos;
                            int32_t offset = bufPos-(int32_t)ipos;
                            Z a = buf[(offset) & bufMask];
                            Z b = buf[(offset-1) & bufMask];
                            Z drd = a + frac * (b - a);
                            Z dwr = drd * fb + *in;
                            buf[bufPos & bufMask] = dwr;
                            out[i] = drd - fb * dwr;
                            in += inStride;
                            delay += delayStride;
                            ++bufPos;
                        }
                    }
				} else {
					for (int i = 0; i < n; ++i) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay / *decay);
                        Z fpos = std::max(1., zdelay * sr);
                        Z ipos = floor(fpos);
                        Z frac = fpos - ipos;
						int32_t offset = bufPos-(int32_t)ipos;
                        Z a = buf[(offset) & bufMask];
                        Z b = buf[(offset-1) & bufMask];
                        Z drd = a + frac * (b - a);
                        Z dwr = drd * fb + *in;
                        buf[bufPos & bufMask] = dwr;
                        out[i] = drd - fb * dwr;
						in += inStride;
						delay += delayStride;
						++bufPos;
					}
				}
				in_.advance(n);
				delay_.advance(n);
				decay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void alpasl_(Thread& th, Prim* prim)
{
	V decay = th.popZIn("alpasl : decay");
	Z maxdelay = th.popFloat("alpasl : maxdelay");
	V delay = th.popZIn("alpasl : delay");
	V in = th.popZIn("alpasl : in");
 	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("alpasl : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
   
	th.push(new List(new AllpassL(th, in, delay, maxdelay, decay)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

class AllpassC : public Gen
{
	ZIn in_;
	ZIn delay_;
	ZIn decay_;
	Z maxdelay_;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
public:
	
	AllpassC(Thread& th, Arg in, Arg delay, Z maxdelay, Arg decay) : Gen(th, itemTypeZ, false), in_(in), delay_(delay), decay_(decay), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		bufSize = NEXTPOWEROFTWO((int32_t)ceil(sr * maxdelay));
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~AllpassC() { free(buf); }
	
	virtual const char* TypeName() const override { return "AllpassC"; }
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int inStride, delayStride, decayStride;
			Z *in, *delay, *decay;
			if (in_(th, n, inStride, in) || delay_(th, n, delayStride, delay) || decay_(th, n, decayStride, decay)) {
				setDone();
				break;
			} else {
				if (decayStride == 0) {
                    if (delayStride == 0) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay / *decay);
                        Z fpos = std::max(2., zdelay * sr);
                        Z ipos = floor(fpos);
                        Z frac = fpos - ipos;
                        int32_t offset = (int32_t)ipos;
                        for (int i = 0; i < n; ++i) {
							int32_t offset2 = bufPos-offset;
							Z a = buf[(offset2+1) & bufMask];
							Z b = buf[(offset2  ) & bufMask];
							Z c = buf[(offset2-1) & bufMask];
							Z d = buf[(offset2-2) & bufMask];
                            Z drd = lagrangeInterpolate(frac, a, b, c, d);
                            Z dwr = drd * fb + *in;
                            buf[bufPos & bufMask] = dwr;
                            out[i] = drd - fb * dwr;
                            in += inStride;
                            ++bufPos;
                        }
                    } else {
                        double rdecay = 1. / *decay;
                        for (int i = 0; i < n; ++i) {
							Z zdelay = *delay;
							zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
                            Z fb = calcDecay(zdelay * rdecay);
                            Z fpos = std::max(2., zdelay * sr);
                            Z ipos = floor(fpos);
                            Z frac = fpos - ipos;
							int32_t offset = bufPos-(int32_t)ipos;
							Z a = buf[(offset+1) & bufMask];
							Z b = buf[(offset  ) & bufMask];
							Z c = buf[(offset-1) & bufMask];
							Z d = buf[(offset-2) & bufMask];
                            Z drd = lagrangeInterpolate(frac, a, b, c, d);
                            Z dwr = drd * fb + *in;
                            buf[bufPos & bufMask] = dwr;
                            out[i] = drd - fb * dwr;
                            in += inStride;
                            delay += delayStride;
                            ++bufPos;
                        }
                    }
				} else {
					for (int i = 0; i < n; ++i) {
						Z zdelay = *delay;
						zdelay = std::clamp(zdelay, -maxdelay_, maxdelay_);
						Z fb = calcDecay(zdelay / *decay);
                        Z fpos = std::max(2., zdelay * sr);
                        Z ipos = floor(fpos);
                        Z frac = fpos - ipos;
						int32_t offset = bufPos-(int32_t)ipos;
                        Z a = buf[(offset+1) & bufMask];
                        Z b = buf[(offset  ) & bufMask];
                        Z c = buf[(offset-1) & bufMask];
                        Z d = buf[(offset-2) & bufMask];
                        Z drd = lagrangeInterpolate(frac, a, b, c, d);
                        Z dwr = drd * fb + *in;
                        buf[bufPos & bufMask] = dwr;
                        out[i] = drd - fb * dwr;
						in += inStride;
						delay += delayStride;
						++bufPos;
					}
				}
				in_.advance(n);
				delay_.advance(n);
				decay_.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void alpasc_(Thread& th, Prim* prim)
{
	V decay = th.popZIn("alpasc : decay");
	Z maxdelay = th.popFloat("alpasc : maxdelay");
	V delay = th.popZIn("alpasc : delay");
	V in = th.popZIn("alpasc : in");
	
	if (maxdelay == 0.) {
		if (delay.isReal()) {
			maxdelay = delay.f;
		} else {
			post("alpasc : maxdelay is zero and delay is a signal\n");
			throw errFailed;
		}
	}
    
	th.push(new List(new AllpassC(th, in, delay, maxdelay, decay)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
class FDN;

class FDN_OutputChannel : public Gen
{
	friend class FDN;
	P<FDN> mFDN;
	
public:	
	FDN_OutputChannel(Thread& th, bool inFinite, FDN* inFDN);
	
	virtual void norefs() override
	{
		mOut = nullptr;
		mFDN = nullptr;
	}
	
	virtual const char* TypeName() const override { return "FDN_OutputChannel"; }
	
	virtual void pull(Thread& th) override;
};

class FDN : public Object
{
	ZIn in_;
	ZIn wet_;
	Z decayLo_;
	Z decayMid_;
	Z decayHi_;
	Z mindelay_;
	Z maxdelay_;
	Z sr_;
	
	Z a1Lo, a1Hi;
	Z scaleLoLPF, scaleLoHPF, scaleHiLPF, scaleHiHPF;
	FDN_OutputChannel* mLeft;
	FDN_OutputChannel* mRight;
	
	static const int kNumDelays = 16;
	
	struct FDNDelay
	{
		Z* buf;
		int size, mask, rpos, wpos, offset;
		Z delay, gain;
		Z fbLo, fbMid, fbHi;
		Z x1A, x1B, x1C, x1D;
		Z y1A, y1B, y1C, y1D;
		
		FDNDelay()
		{
		}
		~FDNDelay() { free(buf); }
		
		void set(Thread& th, const FDN& fdn, Z inDelay, int& ioSampleDelay)
		{
			int sampleDelay = (int)(th.rate.sampleRate * inDelay);
			if (sampleDelay <= ioSampleDelay) sampleDelay = ioSampleDelay + 2;
			sampleDelay = (int)nextPrime(sampleDelay);
			ioSampleDelay = sampleDelay;
			Z actualDelay = (Z)sampleDelay * th.rate.invSampleRate;
			size = NEXTPOWEROFTWO(sampleDelay);
			mask = size - 1;
			printf("delay %6d %6d %6d %f\n", sampleDelay, size, mask, actualDelay);
			rpos = 0;
			wpos = sampleDelay;
			buf = (Z*)calloc(size, sizeof(Z));
			const Z n1 = 1. / sqrt(kNumDelays);
			//const Z n1 = 1. / kNumDelays;
			fbLo  = n1 * calcDecay(actualDelay / fdn.decayLo_);
			fbMid = n1 * calcDecay(actualDelay / fdn.decayMid_);
			fbHi  = n1 * calcDecay(actualDelay / fdn.decayHi_);
			y1A = 0.;
			y1B = 0.;
			y1C = 0.;
			y1D = 0.;
			x1A = 0.;
			x1B = 0.;
			x1C = 0.;
			x1D = 0.;
		}
	};

public:

	FDNDelay mDelay[kNumDelays];
	
	FDN(Thread& th, Arg in, Arg wet, Z mindelay, Z maxdelay, Z decayLo, Z decayMid, Z decayHi, Z seed)
		: in_(in), wet_(wet),
		decayLo_(decayLo), decayMid_(decayMid), decayHi_(decayHi),
		mindelay_(mindelay), maxdelay_(maxdelay)
	{
		sr_ = th.rate.sampleRate;
		Z freqmul = th.rate.invNyquistRate * kFirstOrderCoeffScale;
		a1Lo = t_firstOrderCoeff(freqmul * 200.);
		a1Hi = t_firstOrderCoeff(freqmul * 2000.);
		scaleLoLPF = .5 * (1. - a1Lo);
		scaleLoHPF = .5 * (1. + a1Lo);
		scaleHiLPF = .5 * (1. - a1Hi);
		scaleHiHPF = .5 * (1. + a1Hi);
		
		Z delay = mindelay;
		Z ratio = maxdelay / mindelay;
		Z interval = pow(ratio, 1. / (kNumDelays - 1.));
		int prevSampleDelay = 0;
		for (int i = 0; i < kNumDelays; ++i) {
			double expon = (random() / 2147483647. - .5) * 0.8;
			double deviation = pow(interval,  expon);
			mDelay[i].set(th, *this, delay * deviation, prevSampleDelay);
			delay *= interval;
		}
		
	}
	
	~FDN() { delete mLeft; delete mRight; }
	
	virtual const char* TypeName() const override { return "FDN"; }

	P<List> createOutputs(Thread& th);
	
	void matrix(Z x[kNumDelays])
	{
		Z a0  = x[ 0];
		Z a1  = x[ 1];
		Z a2  = x[ 2];
		Z a3  = x[ 3];
		Z a4  = x[ 4];
		Z a5  = x[ 5];
		Z a6  = x[ 6];
		Z a7  = x[ 7];
		Z a8  = x[ 8];
		Z a9  = x[ 9];
		Z a10 = x[10];
		Z a11 = x[11];
		Z a12 = x[12];
		Z a13 = x[13];
		Z a14 = x[14];
		Z a15 = x[15];

		Z b0  =  a0 + a1;
		Z b1  =  a0 - a1;
		Z b2  =  a2 + a3;
		Z b3  =  a2 - a3;
		Z b4  =  a4 + a5;
		Z b5  =  a4 - a5;
		Z b6  =  a6 + a7;
		Z b7  =  a6 - a7;
		Z b8  =  a8 + a9;
		Z b9  =  a8 - a9;
		Z b10 = a10 + a11;
		Z b11 = a10 - a11;
		Z b12 = a12 + a13;
		Z b13 = a12 - a13;
		Z b14 = a14 + a15;
		Z b15 = a14 - a15;

		Z c0  =  b0 + b2;
		Z c1  =  b1 + b3;
		Z c2  =  b0 - b2;
		Z c3  =  b1 - b3;
		Z c4  =  b4 + b6;
		Z c5  =  b5 + b7;
		Z c6  =  b4 - b6;
		Z c7  =  b5 - b7;
		Z c8  =  b8 + b10;
		Z c9  =  b9 + b11;
		Z c10 =  b8 - b10;
		Z c11 =  b9 - b11;
		Z c12 = b12 + b14;
		Z c13 = b13 + b15;
		Z c14 = b12 - b14;
		Z c15 = b13 - b15;

		Z d0  =  c0 + c4;
		Z d1  =  c1 + c5;
		Z d2  =  c2 + c6;
		Z d3  =  c3 + c7;
		Z d4  =  c0 - c4;
		Z d5  =  c1 - c5;
		Z d6  =  c2 - c6;
		Z d7  =  c3 - c7;
		Z d8  =  c8 + c12;
		Z d9  =  c9 + c13;
		Z d10 = c10 + c14;
		Z d11 = c11 + c15;
		Z d12 =  c8 - c12;
		Z d13 =  c9 - c13;
		Z d14 = c10 - c14;
		Z d15 = c11 - c15;

		x[ 0] = d0 + d8;
		x[ 1] = d1 + d9;
		x[ 2] = d2 + d10;
		x[ 3] = d3 + d11;
		x[ 4] = d4 + d12;
		x[ 5] = d5 + d13;
		x[ 6] = d6 + d14;
		x[ 7] = d7 + d15;
		x[ 8] = d0 - d8;
		x[ 9] = d1 - d9;
		x[10] = d2 - d10;
		x[11] = d3 - d11;
		x[12] = d4 - d12;
		x[13] = d5 - d13;
		x[14] = d6 - d14;
		x[15] = d7 - d15;
	}
    
	virtual void pull(Thread& th) 
	{
		int framesToFill = mLeft->mBlockSize;

		Z Sink = 0.;
		Z* Lout;
		Z* Rout;
		int Loutstride = 1;
		int Routstride = 1;

		if (mLeft->mOut) {
			Lout = mLeft->mOut->fulfillz(framesToFill);
		} else {
			Lout = &Sink;
			Loutstride = 0;
		}

		if (mRight->mOut) {
			Rout = mRight->mOut->fulfillz(framesToFill);
		} else {
			Rout = &Sink;
			Routstride = 0;
		}
		
		while (framesToFill) {
			int n = framesToFill;
			int inStride, wetStride;
			Z *in;
			Z *wet;
			if (in_(th, n, inStride, in) || wet_(th, n, wetStride, wet)) {
				mLeft->setDone();
				mRight->setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					Z x[kNumDelays];

					for (UInt32 j = 0; j < kNumDelays; ++j)
					{
						FDNDelay& d = mDelay[j];
						// read from delay line
						Z x0 = d.buf[d.rpos & d.mask];

						// attenuate and filter the output of the delay line.
						
						// high crossover
						Z x0A = scaleHiHPF * x0;
						Z x0B = scaleHiLPF * x0 ;
						Z y0A = x0A - d.x1A + a1Hi * d.y1A;	// hpf -> high band
						Z y0B = x0B + d.x1B + a1Hi * d.y1B;	// lpf -> low + mid
						d.y1A = y0A;
						d.y1B = y0B;
						d.x1A = x0A;
						d.x1B = x0B;
						
						// low crossover
						Z x0C = scaleLoHPF * y0B;
						Z x0D = scaleLoLPF * y0B;
						Z y0C = x0C - d.x1C + a1Lo * d.y1C;	// hpf -> mid band
						Z y0D = x0D + d.x1D + a1Lo * d.y1D;	// lpf -> low band
						d.y1C = y0C;
						d.y1D = y0D;
						d.x1C = x0C;
						d.x1D = x0D;
						
						x[j] = d.fbLo * y0D  +  d.fbMid * y0C  +  d.fbHi * y0A;

						++d.rpos;
					}
					
					matrix(x);
					
					Z ini = *in;
					Z w = *wet;
					*Lout = ini + w * (x[1] - ini);
					*Rout = ini + w * (x[2] - ini);
					Lout += Loutstride;
					Rout += Routstride;

					// write back to delay line
					for (UInt32 j = 0; j < kNumDelays; ++j) 
					{
						FDNDelay& d = mDelay[j];						
						
						d.buf[d.wpos & d.mask] = x[j] + ini;
						++d.wpos;
						
					}
					in += inStride;
					wet += wetStride;
				}
				in_.advance(n);
				framesToFill -= n;
			}
		}

		if (mLeft->mOut)  mLeft->produce(framesToFill);
		if (mRight->mOut) mRight->produce(framesToFill);
	}
};

FDN_OutputChannel::FDN_OutputChannel(Thread& th, bool inFinite, FDN* inFDN)
                                : Gen(th, itemTypeZ, inFinite), mFDN(inFDN)
{
}

void FDN_OutputChannel::pull(Thread& th)
{
	mFDN->pull(th);
}

P<List> FDN::createOutputs(Thread& th)
{
	mLeft = new FDN_OutputChannel(th, finite, this);
	mRight = new FDN_OutputChannel(th, finite, this);
	
	P<Gen> left = mLeft;
	P<Gen> right = mRight;
	
	P<List> s = new List(itemTypeV, 2);
	P<Array> a = s->mArray;
	a->add(new List(left));
	a->add(new List(right));
	
	return s;
}


static void fdn_(Thread& th, Prim* prim)
{
	Z seed = th.popFloat("fdn : seed");
	Z maxdelay = th.popFloat("fdn : maxdelay");
	Z mindelay = th.popFloat("fdn : mindelay");
	Z decayHi = th.popFloat("fdn : decayHi");
	Z decayMid = th.popFloat("fdn : decayMid");
	Z decayLo = th.popFloat("fdn : decayLo");
	V wet = th.popZIn("fdn : wet");
	V in = th.popZIn("fdn : in");
    
	P<FDN> fdn = new FDN(th, in, wet, mindelay, maxdelay, decayLo, decayMid, decayHi, seed);
	
	P<List> s = fdn->createOutputs(th);

	th.push(s);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

#define DEF(NAME, N, HELP) 	vm.def(#NAME, N, NAME##_, HELP);
#define DEFMCX(NAME, N, HELP) 	vm.defmcx(#NAME, N, NAME##_, HELP);
#define DEFAM(NAME, MASK, HELP) 	vm.defautomap(#NAME, #MASK, NAME##_, HELP);

void AddDelayUGenOps()
{
	vm.addBifHelp("\n*** delay unit generators ***");
	DEFAM(delayn, zzk, "(in delay maxdelay --> out) delay line with no interpolation.");
	DEFAM(delayl, zzk, "(in delay maxdelay --> out) delay line with linear interpolation.");
	DEFAM(delayc, zzk, "(in delay maxdelay --> out) delay line with cubic interpolation.");
	DEFAM(flange, zzk, "(in delay maxdelay --> out) flanger with cubic interpolation. delay can be negative. latency is maxdelay.");
	DEFAM(flangep, zzk, "(in delay maxdelay --> out) flanger with cubic interpolation. adds delayed signal instead of subtracts.");
	
	DEFAM(combn, zzkz, "(in delay maxdelay decayTime --> out) comb delay filter with no interpolation.");
	DEFAM(combl, zzkz, "(in delay maxdelay decayTime --> out) comb delay filter with linear interpolation.");
	DEFAM(combc, zzkz, "(in delay maxdelay decayTime --> out) comb delay filter with cubic interpolation.");
	DEFAM(lpcombc, zzkzz, "(in delay maxdelay decayTime lpfreq --> out) low pass comb delay filter with cubic interpolation.");
	
	DEFAM(alpasn, zzkz, "(in delay maxdelay decayTime --> out) all pass delay filter with no interpolation.");
	DEFAM(alpasl, zzkz, "(in delay maxdelay decayTime --> out) all pass delay filter with linear interpolation.");
	DEFAM(alpasc, zzkz, "(in delay maxdelay decayTime --> out) all pass delay filter with cubic interpolation.");
	//DEFAM(fdn, zzkkkkkk, "(in wet decayLo decayMid decayHi mindelay maxdelay rseed --> out) feedback delay network reverb.");
}


