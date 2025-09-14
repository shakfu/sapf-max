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

#include "UGen.hpp"

#include "VM.hpp"
#include "MultichannelExpansion.hpp"
#include "clz.hpp"
#include <cmath>
#include <float.h>
#include <vector>
#include <algorithm>
#include <Accelerate/Accelerate.h>



struct MulAdd : public ThreeInputUGen<MulAdd>
{
	MulAdd(Thread& th, Arg a, Arg b, Arg c) : ThreeInputUGen<MulAdd>(th, a, b, c)
	{
	}
	
	virtual const char* TypeName() const override { return "MulAdd"; }
		
	void calc(int n, Z* out, Z* a, Z* b, Z* c, int aStride, int bStride, int cStride) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = *a * *b + *c;
			a += aStride;
			b += bStride;
			c += cStride;
		}
	}
};

extern UnaryOp* gUnaryOpPtr_neg;
extern BinaryOp* gBinaryOpPtr_plus;
extern BinaryOp* gBinaryOpPtr_minus;
extern BinaryOp* gBinaryOpPtr_mul;

static void madd_(Thread& th, Prim* prim)
{
	V c = th.popZIn("*+ : c");
	V b = th.popZIn("*+ : b");
	V a = th.popZIn("*+ : a");

	if (a.isReal() && b.isReal() && c.isReal()) {
		th.push(a.f * b.f + c.f);
	} else {
		if (c.isReal()) {
			if (c.f == 0.) {
				if (a.isReal() && a.f == 1.) { th.push(b); return; }
				if (b.isReal() && b.f == 1.) { th.push(a); return; }
				if (a.isReal() && a.f == -1.) { th.push(b.unaryOp(th, gUnaryOpPtr_neg)); return; }
				if (b.isReal() && b.f == -1.) { th.push(a.unaryOp(th, gUnaryOpPtr_neg)); return; }
				th.push(a.binaryOp(th, gBinaryOpPtr_mul, b)); return;
			}
		}
		if (a.isReal()) {
			if (a.f == 0.) { th.push(c); return; }
			if (a.f == 1.) { th.push(b.binaryOp(th, gBinaryOpPtr_plus, c)); return; }
			if (a.f == -1.) { th.push(b.binaryOp(th, gBinaryOpPtr_minus, c)); return; }
		}
		if (b.isReal()) {
			if (b.f == 0.) { th.push(c); return; } 
			if (b.f == 1.) { th.push(a.binaryOp(th, gBinaryOpPtr_plus, c)); return; } 
			if (b.f == -1.) { th.push(a.binaryOp(th, gBinaryOpPtr_minus, c)); return; }
		}
		th.push(new List(new MulAdd(th, a, b, c)));
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Fadeout : Gen
{
	ZIn _a;
	int64_t _sustainTime;
	int64_t _fadeTime;
	Z _amp, _fade;
	
	Fadeout(Thread& th, Arg a, Z sustainTime, Z fadeTime) : Gen(th, itemTypeZ, true), _a(a)
	{
		_sustainTime = (int64_t)floor(th.rate.sampleRate * sustainTime + .5);
		_fadeTime = (int64_t)floor(th.rate.sampleRate * fadeTime + .5);
		_sustainTime = std::max(1LL, _sustainTime);
		_fadeTime = std::max(1LL, _fadeTime);
		_amp = 1.001;
		_fade = pow(.001, 1. / _fadeTime);
	}
	virtual const char* TypeName() const override { return "Fadeout"; }
    	
	virtual void pull(Thread& th) override {
		if (_sustainTime <= 0) {
			if (_fadeTime <= 0) {
				end();
			} else {
				int framesToFill = (int)std::min(_fadeTime, (int64_t)mBlockSize);
				Z* out = mOut->fulfillz(framesToFill);
				_fadeTime -= framesToFill;
				while (framesToFill) {
					int n = framesToFill;
					int astride;
					Z *a;
					if (_a(th, n,astride, a)) {
					  setDone();
						break;
					} else {
						Z amp = _amp;
						Z fade = _fade;
						for (int i = 0; i < n; ++i) {
							out[i] = *a * (amp - .001);
							amp *= fade;
							a += astride;
						}
						_amp = amp;
						_a.advance(n);
						framesToFill -= n;
						out += n;
					}
				}
			   produce(framesToFill);
			}
		} else {
            int framesToFill = (int)std::min(_sustainTime, (int64_t)mBlockSize);
            Z* out = mOut->fulfillz(framesToFill);
            _sustainTime -= framesToFill;
            while (framesToFill) {
                int n = framesToFill;
                int astride;
                Z *a;
                if (_a(th, n,astride, a)) {
                  setDone();
                    break;
                } else {
                    for (int i = 0; i < n; ++i) {
                        out[i] = *a;
						a += astride;
                    }
                    _a.advance(n);
                    framesToFill -= n;
                    out += n;
                }
            }
           produce(framesToFill);
        }
	}
};


struct Fadein : Gen
{
	ZIn _a;
	int64_t _fadeTime;
	Z _amp, _fade;
	
	Fadein(Thread& th, Arg a, Z fadeTime) : Gen(th, itemTypeZ, true), _a(a)
	{
		_fadeTime = (int64_t)floor(th.rate.sampleRate * fadeTime + .5);
		_fadeTime = std::max(1LL, _fadeTime);
		_amp = .001;
		_fade = pow(1000., 1. / _fadeTime);
	}
	virtual const char* TypeName() const override { return "Fadein"; }
    	
	virtual void pull(Thread& th) override {
		if (_fadeTime <= 0) {
			_a.link(th, mOut);
			setDone();
		} else {
			int framesToFill = (int)std::min(_fadeTime, (int64_t)mBlockSize);
			Z* out = mOut->fulfillz(framesToFill);
			_fadeTime -= framesToFill;
			while (framesToFill) {
				int n = framesToFill;
				int astride;
				Z *a;
				if (_a(th, n,astride, a)) {
				  setDone();
					break;
				} else {
					Z amp = _amp;
					Z fade = _fade;
					for (int i = 0; i < n; ++i) {
						out[i] = *a * (amp - .001);
						amp *= fade;
						a += astride;
					}
					_amp = amp;
					_a.advance(n);
					framesToFill -= n;
					out += n;
				}
			}
		   produce(framesToFill);
		}
	}
};

static void fadeout_(Thread& th, Prim* prim)
{
	Z fade = th.popFloat("fadeout : fadeTime");
	Z sustain = th.popFloat("fadeout : sustainTime");
	V in = th.popZIn("fadeout : in");

	th.push(new List(new Fadeout(th, in, sustain, fade)));
}

static void fadein_(Thread& th, Prim* prim)
{
	Z fade = th.popFloat("fadein : fadeTime");
	V in = th.popZIn("fadein : in");

	th.push(new List(new Fadein(th, in, fade)));
}

struct Endfade : Gen
{
	ZIn _a;
	int64_t _startupTime;
	int64_t _holdTime;
	int64_t _holdTimeRemaining;
	int64_t _fadeTime;
	Z _amp, _fade, _threshold;
	
	Endfade(Thread& th, Arg a, Z startupTime, Z holdTime, Z fadeTime, Z threshold) : Gen(th, itemTypeZ, true), _a(a)
	{
		_startupTime = (int64_t)floor(th.rate.sampleRate * startupTime + .5);
		_holdTime = (int64_t)floor(th.rate.sampleRate * holdTime + .5);
		_fadeTime = (int64_t)floor(th.rate.sampleRate * fadeTime + .5);
		_startupTime = std::max(0LL, _startupTime);
		_holdTime = std::max(1LL, _holdTime);
		_holdTimeRemaining = _holdTime;
		_fadeTime = std::max(1LL, _fadeTime);
		_threshold = threshold;
		_amp = 1.001;
		_fade = pow(.001, 1. / _fadeTime);
	}
	virtual const char* TypeName() const override { return "Endfade"; }
    	
	virtual void pull(Thread& th) override {
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		
		while (framesToFill) {
			if (_startupTime > 0) {
				int n = (int)std::min((int64_t)framesToFill, _startupTime);
				int astride;
				Z *a;
				if (_a(th, n, astride, a)) {
					setDone();
					break;
				} else {
					for (int i = 0; i < n; ++i) {
						out[i] = *a;
						a += astride;
					}
					_a.advance(n);
					framesToFill -= n;
					out += n;
					_startupTime -= n;
				}
			} else if (_holdTimeRemaining > 0) {
 				int n = framesToFill;
				int astride;
				Z *a;
				if (_a(th, n, astride, a)) {
					setDone();
					break;
				} else {
					int framesFilled = 0;
					for (int i = 0; i < n && _holdTimeRemaining > 0; ++i) {
						Z z = *a;
						if (std::abs(z) >= _threshold) {
							_holdTimeRemaining = _holdTime;
						} else {
							--_holdTimeRemaining;
						}
						out[i] = z;
						a += astride;
						++framesFilled;
					}
					_a.advance(framesFilled);
					framesToFill -= framesFilled;
					out += framesFilled;
				}
			} else if (_fadeTime > 0) {
				int n = (int)std::min((int64_t)framesToFill, _fadeTime);
				int astride;
				Z *a;
				if (_a(th, n,astride, a)) {
					setDone();
					break;
				} else {
					Z amp = _amp;
					Z fade = _fade;
					for (int i = 0; i < n; ++i) {
						out[i] = *a * (amp - .001);
						amp *= fade;
						a += astride;
					}
					_amp = amp;
					_a.advance(n);
					framesToFill -= n;
					out += n;
					_fadeTime -= n;
				}
			} else {
				setDone();
				break;
			}
		}
		produce(framesToFill);
	}
};



static void endfade_(Thread& th, Prim* prim)
{
	Z threshold = th.popFloat("endfade : threshold");
	Z fade = th.popFloat("endfade : fadeTime");
	Z hold = th.popFloat("endfade : holdTime");
	Z startup = th.popFloat("endfade : startupTime");
	V in = th.popZIn("endfade : in");

	th.push(new List(new Endfade(th, in, startup, hold, fade, threshold)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Imps : public Gen
{
	BothIn durs_;
	BothIn vals_;
	ZIn rate_;
	Z val_;
	Z phase_, dur_, invdur_;
	Z freqmul_;
	bool once;

	Imps(Thread& th, Arg durs, Arg vals, Arg rate) : Gen(th, itemTypeZ, mostFinite(vals, durs, rate)), durs_(durs), vals_(vals), rate_(rate),
		phase_(0.), dur_(0.), freqmul_(th.rate.invSampleRate), once(false)
	{
	}

	virtual const char* TypeName() const override { return "Imps"; }

	virtual void pull(Thread& th) override
	{	
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}
			for (int i = 0; i < n; ++i) {
				while (phase_ >= dur_) {
					phase_ -= dur_;
					do {
						if (vals_.onez(th, val_) || durs_.onez(th, dur_)) {
							setDone();
							goto leave;
						}
					} while (dur_ <= 0.);
					once = true;
				}

				if (once) {
					out[i] = val_;
					once = false;
				} else {
					out[i] = 0.;
				}
				phase_ += *rate * freqmul_;
				rate += rateStride;
				--framesToFill;
			}
			out += n;
			
		}
leave:
		produce(framesToFill);
	}
};

struct Steps : public Gen
{
	BothIn durs_;
	BothIn vals_;
	ZIn rate_;
	Z val_;
	Z phase_, dur_;
	Z freqmul_;

	Steps(Thread& th, Arg durs, Arg vals, Arg rate) : Gen(th, itemTypeZ, mostFinite(vals, durs, rate)), durs_(durs), vals_(vals), rate_(rate),
		phase_(0.), dur_(0.), freqmul_(th.rate.invSampleRate)
	{
	}

	virtual const char* TypeName() const override { return "Steps"; }

	virtual void pull(Thread& th) override
	{	
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}

			for (int i = 0; i < n; ++i) {
				while (phase_ >= dur_) {
					phase_ -= dur_;
					do {
						if (vals_.onez(th, val_) || durs_.onez(th, dur_)) {
							setDone();
							goto leave;
						}
					} while (dur_ <= 0.);
				}

				out[i] = val_;

				phase_ += *rate * freqmul_;
				rate += rateStride;
				--framesToFill;
			}
			out += n;
			rate_.advance(n);
		}
leave:
		produce(framesToFill);
	}
};

struct Gates : public Gen
{
	BothIn durs_;
	BothIn vals_;
	BothIn hold_;
	ZIn rate_;
	Z val_;
	Z phase_, dur_, hdur_;
	Z freqmul_;

	Gates(Thread& th, Arg durs, Arg vals, Arg hold, Arg rate) : Gen(th, itemTypeZ, mostFinite(vals, durs, rate, hold)), durs_(durs), vals_(vals), hold_(hold), rate_(rate),
		phase_(0.), dur_(0.), freqmul_(th.rate.invSampleRate)
	{
	}

	virtual const char* TypeName() const override { return "Gates"; }

	virtual void pull(Thread& th) override
	{	
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}

			for (int i = 0; i < n; ++i) {
				while (phase_ >= dur_) {
					phase_ -= dur_;
					do {
						if (vals_.onez(th, val_) || durs_.onez(th, dur_) || hold_.onez(th, hdur_)) {
							setDone();
							goto leave;
						}
					} while (dur_ <= 0.);
				}

				out[i] = phase_ < hdur_ ? val_ : 0.;

				phase_ += *rate * freqmul_;
				rate += rateStride;
				--framesToFill;
			}
			out += n;
			rate_.advance(n);
		}
leave:
		produce(framesToFill);
	}
};

struct Lines : public Gen
{
	BothIn durs_;
	BothIn vals_;
	ZIn rate_;
	Z oldval_, newval_, slope_;
	Z phase_, dur_;
	Z freqmul_;
	bool once = true;

	Lines(Thread& th, Arg durs, Arg vals, Arg rate) : Gen(th, itemTypeZ, mostFinite(vals, durs, rate)), durs_(durs), vals_(vals), rate_(rate),
		phase_(0.), dur_(0.), freqmul_(th.rate.invSampleRate)
	{
	}

	virtual const char* TypeName() const override { return "Lines"; }

	virtual void pull(Thread& th) override
	{
		if (once) {
			once = false;
			vals_.onez(th, newval_);
		}
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}

			for (int i = 0; i < n; ++i) {
				while (phase_ >= dur_) {
					phase_ -= dur_;
					do {
						oldval_ = newval_;
						if (vals_.onez(th, newval_) || durs_.onez(th, dur_)) {
							setDone();
							goto leave;
						}
					} while (dur_ <= 0.);
					slope_ = (newval_ - oldval_) / dur_;
				}

				out[i] = oldval_ + slope_ * phase_;

				phase_ += *rate * freqmul_;
				rate += rateStride;
				--framesToFill;
			}
			out += n;
			rate_.advance(n);
		}
leave:
		produce(framesToFill);
	}
};

struct XLines : public Gen
{
	BothIn durs_;
	BothIn vals_;
	ZIn rate_;
	Z oldval_, newval_, ratio_, step_;
	Z phase_, dur_, invdur_, freq_;
	Z freqmul_;
	bool once = true;
	//bool cheat;

	XLines(Thread& th, Arg durs, Arg vals, Arg rate) : Gen(th, itemTypeZ, mostFinite(vals, durs, rate)), durs_(durs), vals_(vals), rate_(rate),
		phase_(0.), dur_(0.), freqmul_(th.rate.invSampleRate)
	{
	}

	virtual const char* TypeName() const override { return "XLines"; }

	virtual void pull(Thread& th) override
	{	
		if (once) {
			once = false;
			vals_.onez(th, newval_);
		}
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}

			if (rateStride == 0) {
				for (int i = 0; i < n; ++i) {
					while (phase_ >= dur_) {
						phase_ -= dur_;
						do {
							oldval_ = newval_;
							if (vals_.onez(th, newval_) || durs_.onez(th, dur_)) {
								setDone();
								goto leave;
							}
						} while (dur_ <= 0.);
						invdur_ = 1. / dur_;
						ratio_ = newval_ / oldval_;
						//oldval_ = oldval_ * pow(ratio_, phase_ * invdur_);
						freq_ = *rate * freqmul_;
						step_ = pow(ratio_, freq_ * invdur_);
					}

					out[i] = oldval_;
					oldval_ *= step_;

					phase_ += freq_;
					--framesToFill;
				}
			} else {
				for (int i = 0; i < n; ++i) {
					while (phase_ >= dur_) {
						phase_ -= dur_;
						do {
							oldval_ = newval_;
							if (vals_.onez(th, newval_) || durs_.onez(th, dur_)) {
								setDone();
								goto leave;
							}
						} while (dur_ <= 0.);
						invdur_ = 1. / dur_;
						ratio_ = newval_ / oldval_;
					}

					out[i] = oldval_ * pow(ratio_, phase_ * invdur_);

					phase_ += *rate * freqmul_;
					rate += rateStride;
					--framesToFill;
				}
			}
			out += n;
			rate_.advance(n);
		}
leave:
		produce(framesToFill);
	}
};

struct Curves : public Gen
{
	BothIn durs_;
	BothIn vals_;
	BothIn curves_;
	ZIn rate_;
	Z oldval_, newval_, step_;
	Z phase_, dur_, curve_, invdur_, freq_;
	Z b1_, a2_;
	Z freqmul_;
	bool once = true;
	//bool cheat;

	Curves(Thread& th, Arg durs, Arg vals, Arg curves, Arg rate) : Gen(th, itemTypeZ, mostFinite(vals, durs, rate, curves)), durs_(durs), vals_(vals), curves_(curves), rate_(rate),
		phase_(0.), dur_(0.), freqmul_(th.rate.invSampleRate)
	{
	}

	virtual const char* TypeName() const override { return "Curves"; }

	virtual void pull(Thread& th) override
	{	
		if (once) {
			once = false;
			vals_.onez(th, newval_);
		}
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}

			if (rateStride == 0) {
				for (int i = 0; i < n; ++i) {
					while (phase_ >= dur_) {
						phase_ -= dur_;
						do {
							oldval_ = newval_;
							if (vals_.onez(th, newval_) || curves_.onez(th, curve_) || durs_.onez(th, dur_)) {
								setDone();
								goto leave;
							}
						} while (dur_ <= 0.);
						dur_ = std::max(dur_, 1e-4);
						invdur_ = 1. / dur_;
						Z a1 = (newval_ - oldval_) / (1. - exp(curve_));
						a2_ = oldval_ + a1;
						b1_ = a1;
						freq_ = *rate * freqmul_;
						step_ = exp(curve_ * freq_ * invdur_);
					}

					out[i] = oldval_;
					b1_ *= step_;
					oldval_ = a2_ - b1_;

					phase_ += freq_;
					--framesToFill;
				}
			} else {
                //!! not correct
				for (int i = 0; i < n; ++i) {
					while (phase_ >= dur_) {
						phase_ -= dur_;
						do {
							oldval_ = newval_;
							if (vals_.onez(th, newval_) || curves_.onez(th, curve_) || durs_.onez(th, dur_)) {
								setDone();
								goto leave;
							}
						} while (dur_ <= 0.);
						invdur_ = 1. / dur_;
						Z a1 = (newval_ - oldval_) / (1. - exp(curve_));
						a2_ = oldval_ + a1;
						b1_ = a1;
						freq_ = freqmul_;
						step_ = exp(curve_ * freq_ * invdur_);
					}
                    
					out[i] = oldval_;
					b1_ *= step_;
					oldval_ = a2_ - b1_;
                    
					phase_ += *rate * freqmul_;
					--framesToFill;
				}
			}
			out += n;
			rate_.advance(n);
		}
leave:
		produce(framesToFill);
	}
};


struct Cubics : public Gen
{
	BothIn vals_;
	ZIn rate_;
	Z y0, y1, y2, y3;
	Z c0, c1, c2, c3;
	Z phase_;
	Z freqmul_;
	bool once = true;

	Cubics(Thread& th, Arg vals, Arg rate) : Gen(th, itemTypeZ, mostFinite(vals, rate)), vals_(vals), rate_(rate),
		phase_(1.), freqmul_(th.rate.invSampleRate)
	{
	}

	virtual const char* TypeName() const override { return "Cubics"; }

	virtual void pull(Thread& th) override
	{	
		if (once) {
			once = false;
			y0 = 0.;
			y1 = 0.;
			vals_.onez(th, y2);
			vals_.onez(th, y3);
		}
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		Z x = phase_;
		Z freqmul = freqmul_;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}

			for (int i = 0; i < n; ++i) {
				while (x >= 1.) {
					x -= 1.;
					y0 = y1;
					y1 = y2;
					y2 = y3;
					if (vals_.onez(th, y3)) {
						setDone();
						goto leave;
					} else {
						c0 = y1;
						c1 = .5 * (y2 - y0);
						c2 = y0 - 2.5 * y1 + 2. * y2 - .5 * y3;
						c3 = 1.5 * (y1 - y2) + .5 * (y3 - y0);
					}
				}

				out[i] = ((c3 * x + c2) * x + c1) * x + c0;

				x += *rate * freqmul;
				rate += rateStride;
				--framesToFill;
			}
			out += n;
			rate_.advance(n);
		}
leave:
		produce(framesToFill);
		phase_ = x;
	}
};

/*
d = duration of accelerando in beats
a = duration of accelerando in seconds

r0 = starting tempo in beats per second
r1 = ending tempo in beats per second

c = (r1 - r0) / d

duration of accelerando in seconds
a = log(r1/r0) / c

tempo for next sample
r(0) = r0
r(n) = r(n-1) * exp(c / sr)

beat for next sample
b(n) = r(n)/c - r0/c

b(0) = 0
b(n) = b(n-1) + r(n)/sr 

*/


struct Tempo : public Gen
{
	BothIn vals_;
	ZIn rate_;
	Z beat_ = 0.;
	Z dur_ = 0.;
	Z lastTime_ = 0.;
	Z nextTime_ = 0.;
	Z invsr_;
	Z c_, r0_, r1_;
	bool once = true;

	Tempo(Thread& th, Arg vals, Arg rate) : Gen(th, itemTypeZ, mostFinite(vals, rate)), vals_(vals), rate_(rate),
		invsr_(th.rate.invSampleRate)
	{
	}

	virtual const char* TypeName() const override { return "Tempo"; }

	virtual void pull(Thread& th) override
	{	
		if (once) {
			once = false;
			if (vals_.onez(th, r1_)) {
				setDone();
				return;
			}
		}
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}

			for (int i = 0; i < n; ++i) {
				// numerically it would be better to subtract off dur each time, but we need to recreate the same loss of precision over time
				// as will be experienced from an integration of tempo occuring outside of this generator. otherwise there would be a drift
				// between when tempo changes occur and the beat time as integrated from the tempo.
				while (beat_ >= nextTime_) {
					do {
						r0_ = r1_;
						if (vals_.onez(th, dur_)) {
							setDone();
							goto leave;
						}
						if (vals_.onez(th, r1_)) {
							setDone();
							goto leave;
						}
					} while (dur_ <= 0.);
					c_ = (r1_ - r0_) / dur_;
					lastTime_ = nextTime_;
					nextTime_ += dur_;
				}

				Z tempo = *rate * (r0_ + (beat_ - lastTime_) * c_);
				out[i] = tempo;
				beat_ += tempo * invsr_;

				rate += rateStride;
				--framesToFill;
			}

			out += n;
			rate_.advance(n);
		}
leave:
		produce(framesToFill);
	}
};


struct Beats : public Gen
{
	ZIn tempo_;
	Z beat_ = 0.;
	Z invsr_;

	Beats(Thread& th, Arg tempo) : Gen(th, itemTypeZ, tempo.isFinite()), tempo_(tempo),
		invsr_(th.rate.invSampleRate)
	{
	}

	virtual const char* TypeName() const override { return "Beats"; }

	virtual void pull(Thread& th) override
	{	

		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		Z beat = beat_;
		while (framesToFill) {
			Z* tempo;
			int n = framesToFill;
			int tempoStride;
			if (tempo_(th, n, tempoStride, tempo)) {
				setDone();
				break;
			}

			Z invsr = invsr_;
			for (int i = 0; i < n; ++i) {
				out[i] = beat;
				beat += *tempo * invsr;
				tempo += tempoStride;
			}
			framesToFill -= n;
			out += n;
			tempo_.advance(n);
		}
		beat_ = beat;
		produce(framesToFill);
	}
};


template <int NumStages>
struct ADSR : public Gen
{
	Z levels_[NumStages+1];
	Z durs_[NumStages];
	Z curves_[NumStages];
	ZIn rate_;
	int stage_ = 0;
	Z oldval_ = 0.;
	Z newval_ = 0.;
	Z step_;
	Z phase_ = 0.;
	Z dur_ = 0.;
	Z noteOff_;
	Z curve_;
	Z b1_, a2_;
	Z freqmul_;
	Z beat_ = 0.;
	bool once = true;
	int sustainStage_;

	ADSR(Thread& th, Z* levels, Z* durs, Z* curves, Arg rate, int sustainStage) : Gen(th, itemTypeZ, true),
		rate_(rate), sustainStage_(sustainStage),
		freqmul_(th.rate.invSampleRate)
	{
		memcpy(levels_, levels, (NumStages+1)*sizeof(Z));
		memcpy(durs_,   durs,   NumStages*sizeof(Z));
		memcpy(curves_, curves, NumStages*sizeof(Z));
		noteOff_ = durs_[sustainStage_];

		oldval_ = levels[0];
		newval_ = levels_[1];
		curve_ = curves_[0];
		dur_ = durs_[0];

		calcStep();
	}

	virtual const char* TypeName() const override { return "ADSR"; }

	void calcStep()
	{
		if (fabs(curve_) < .01) {
			a2_ = oldval_;
			b1_ = 0.;
			step_ = 1.;
		} else {

			dur_ = std::max(dur_, 1e-5);
			Z invdur = 1. / dur_;
			Z a1 = (newval_ - oldval_) / (1. - exp(curve_));
			a2_ = oldval_ + a1;
			b1_ = a1;
			step_ = exp(curve_ * freqmul_ * invdur);
		}
	}

	virtual void pull(Thread& th) override
	{
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}
			
			// rework this loop!!!
			//adsr
			//	if (stage < releaseStage) {
			//		integrate tempo
			//		search from end. break when < noteoff.
			//		remember note off sample index.
			//	}
			//	
			//	when there is a new stage and it is not the sustainStage
			//	compute the number of samples in the stage
			//	
			//	limit n to the min(blockSize, stageSamplesRemaining, [noteOffSample])
			//	after a stage 


			for (int i = 0; i < n; ++i) {
				while (1) {
					if (stage_ < sustainStage_) {
						if (phase_ >= dur_) {
							phase_ -= dur_;
							++stage_;
						} else if (beat_ >= noteOff_) {
							phase_ = 0.;
							stage_ = sustainStage_+1; // go into release mode
						} else break;
					} else if (stage_ == sustainStage_) {
						if (beat_ >= noteOff_) {
							phase_ = 0.;
							stage_ = sustainStage_+1;
						} else break;
					} else if (stage_ < NumStages){
						if (phase_ >= dur_) {
							phase_ -= dur_;
							++stage_;
						} else break;
					} else {
						setDone();
						goto leave;
					}					
										
					newval_ = levels_[stage_+1];
					curve_ = curves_[stage_];
					dur_ = durs_[stage_];
					
					calcStep();
				}

				out[i] = oldval_;
				b1_ *= step_;
				oldval_ = a2_ - b1_;

				beat_ += *rate * freqmul_;
				rate += rateStride;
				phase_ += freqmul_;
				--framesToFill;
			}
			out += n;
			rate_.advance(n);
		}
leave:
		produce(framesToFill);
	}
};


template <int NumStages>
struct GatedADSR : public Gen
{
	Z levels_[NumStages+1];
	Z durs_[NumStages];
	Z curves_[NumStages];
	ZIn gate_;
	int stage_ = NumStages;
	Z oldval_ = 0.;
	Z newval_ = 0.;
	Z step_;
	Z phase_ = 0.;
	Z dur_ = 0.;
	Z curve_;
	Z b1_, a2_;
	Z freqmul_;
	bool once = true;
	int sustainStage_;

	GatedADSR(Thread& th, Z* levels, Z* durs, Z* curves, Arg gate, int sustainStage) : Gen(th, itemTypeZ, true),
		gate_(gate), sustainStage_(sustainStage),
		freqmul_(th.rate.invSampleRate)
	{
		memcpy(levels_, levels, (NumStages+1)*sizeof(Z));
		memcpy(durs_,   durs,   NumStages*sizeof(Z));
		memcpy(curves_, curves, NumStages*sizeof(Z));

		oldval_ = levels[0];
		newval_ = levels_[1];
		curve_ = curves_[0];
		dur_ = durs_[0];

		calcStep();
	}

	virtual const char* TypeName() const override { return "GatedADSR"; }

	void calcStep()
	{
		if (fabs(curve_) < .01) {
			a2_ = oldval_;
			b1_ = 0.;
			step_ = 1.;
		} else {

			dur_ = std::max(dur_, 1e-5);
			Z invdur = 1. / dur_;
			Z a1 = (newval_ - oldval_) / (1. - exp(curve_));
			a2_ = oldval_ + a1;
			b1_ = a1;
			step_ = exp(curve_ * freqmul_ * invdur);
		}
	}

	virtual void pull(Thread& th) override
	{
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		while (framesToFill) {
			Z* gate;
			int n = framesToFill;
			int gateStride;
			if (gate_(th, n, gateStride, gate)) {
				setDone();
				break;
			}
			
			// rework this loop!!!
			//adsr
			//	if (stage < releaseStage) {
			//		integrate tempo
			//		search from end. break when < noteoff.
			//		remember note off sample index.
			//	}
			//	
			//	when there is a new stage and it is not the sustainStage
			//	compute the number of samples in the stage
			//	
			//	limit n to the min(blockSize, stageSamplesRemaining, [noteOffSample])
			//	after a stage 


			for (int i = 0; i < n; ++i) {
                Z g = *gate;
                gate += gateStride;
                if (stage_ >= NumStages) {
                    // waiting for trigger
                    if (*gate > 0.) {
                        stage_ = 0;
                    } else {
                        out[i] = 0.;
                        --framesToFill;
                       continue;
                    }
                }
				while (1) {                    
                    if (stage_ < sustainStage_) {
						if (g <= 0.) {
							phase_ = 0.;
							stage_ = sustainStage_+1; // go into release mode
						} else if (phase_ >= dur_) {
							phase_ -= dur_;
							++stage_;
						} else break;
					} else if (stage_ == sustainStage_) {
						if (g <= 0.) {
							phase_ = 0.;
							stage_ = sustainStage_+1;
						} else break;
					} else {
						if (phase_ >= dur_) {
							phase_ -= dur_;
							++stage_;
						} else break;
					}
										
					newval_ = levels_[stage_+1];
					curve_ = curves_[stage_];
					dur_ = durs_[stage_];
					
					calcStep();
				}

				out[i] = oldval_;
				b1_ *= step_;
				oldval_ = a2_ - b1_;

				phase_ += freqmul_;
				--framesToFill;
			}
			out += n;
			gate_.advance(n);
		}
		produce(framesToFill);
	}
};



static void imps_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("imps : rate");
	V durs = th.pop();
	V vals = th.pop();

	th.push(new List(new Imps(th, durs, vals, rate)));
}

static void steps_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("steps : rate");
	V durs = th.pop();
	V vals = th.pop();

	th.push(new List(new Steps(th, durs, vals, rate)));
}

static void gates_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("gates : rate");
	V hold = th.pop();
	V durs = th.pop();
	V vals = th.pop();

	th.push(new List(new Gates(th, durs, vals, hold, rate)));
}

static void lines_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("lines : rate");
	V durs = th.pop();
	V vals = th.pop();

	th.push(new List(new Lines(th, durs, vals, rate)));
}

static void xlines_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("xlines : rate");
	V durs = th.pop();
	V vals = th.pop();

	th.push(new List(new XLines(th, durs, vals, rate)));
}

static void cubics_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("cubics : rate");
	V vals = th.pop();

	th.push(new List(new Cubics(th, vals, rate)));
}

static void curves_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("curves : rate");
	V durs = th.pop();
	V param = th.pop();
	V vals = th.pop();

	th.push(new List(new Curves(th, durs, vals, param, rate)));
}

static void tempo_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("tempo : rate");
	V vals = th.pop();

	th.push(new List(new Tempo(th, vals, rate)));
}

static void beats_(Thread& th, Prim* prim)
{
	V tempo = th.popZIn("beats : tempo");

	th.push(new List(new Beats(th, tempo)));
}

static void adsr_(Thread& th, Prim* prim)
{
	
	V rate    = th.popZIn("adsr :  tempo");
	Z noteDur = th.popFloat("adsr : noteDur");
	Z amp     = th.popFloat("adsr : amp");

	P<List> list   = th.popList("adsr : [attack decay sustain release]");
	const int kNumADSRStages = 4;
	Z env[kNumADSRStages];
	if (list->fillz(th, kNumADSRStages, env) != kNumADSRStages) {
		post("adsr : [attack decay sustain release] list should have 4 elements.");
	}
		
	Z relTime = env[3];
	Z susLvl  = env[2];
	Z dcyTime = env[1];
	Z atkTime = env[0];
	
	Z levels[kNumADSRStages+1] = { 0., amp, amp*susLvl, amp*susLvl, 0. };
	Z durs[kNumADSRStages] = { atkTime, dcyTime, noteDur, relTime };
	Z curves[kNumADSRStages] = { -1., -5., 0., -5. };

	th.push(new List(new ADSR<kNumADSRStages>(th, levels, durs, curves, rate, 2)));
}

static void dadsr_(Thread& th, Prim* prim)
{
	
	V rate    = th.popZIn("dadsr :  tempo");
	Z noteDur = th.popFloat("dadsr : noteDur");
	Z amp     = th.popFloat("dadsr : amp");

	P<List> list   = th.popList("dadsr : [delay attack decay sustain release]");
	
	const int kNumADSRStages = 5;
	Z env[kNumADSRStages];
	if (list->fillz(th, kNumADSRStages, env) != kNumADSRStages) {
		post("dahdsr : [delay attack decay sustain release] list should have 5 elements.");
	}
	
	Z relTime = env[4];
	Z susLvl  = env[3];
	Z dcyTime = env[2];
	Z atkTime = env[1];
	Z dlyTime = env[0];
	
	Z levels[kNumADSRStages+1] = { 0., 0., amp, amp*susLvl, amp*susLvl, 0. };
	Z durs[kNumADSRStages] = { dlyTime, atkTime, dcyTime, noteDur, relTime };
	Z curves[kNumADSRStages] = { 0., -1., -5., 0., -5. };

	th.push(new List(new ADSR<kNumADSRStages>(th, levels, durs, curves, rate, 3)));
}

static void dahdsr_(Thread& th, Prim* prim)
{
	
	V rate    = th.popZIn("dahdsr :  tempo");
	Z noteDur = th.popFloat("dahdsr : noteDur");
	Z amp     = th.popFloat("dahdsr : amp");

	P<List> list   = th.popList("dahdsr : [delay attack hold decay sustain release]");
	
	const int kNumADSRStages = 6;
	Z env[kNumADSRStages];
	if (list->fillz(th, kNumADSRStages, env) != kNumADSRStages) {
		post("dahdsr : [delay attack hold decay sustain release] list should have 6 elements.");
	}
	
	Z relTime = env[5];
	Z susLvl  = env[4];
	Z dcyTime = env[3];
	Z hldTime = env[2];
	Z atkTime = env[1];
	Z dlyTime = env[0];
	
	Z levels[kNumADSRStages+1] = { 0., 0., amp, amp, amp*susLvl, amp*susLvl, 0. };
	Z durs[kNumADSRStages] = { dlyTime, atkTime, hldTime, dcyTime, noteDur, relTime };
	Z curves[kNumADSRStages] = { 0., -1., 0., -5., 0., -5. };

	th.push(new List(new ADSR<kNumADSRStages>(th, levels, durs, curves, rate, 4)));
}




struct K2A : public Gen
{
	int n_;
	int remain_;
	Z slopeFactor_;
	BothIn vals_;
	Z oldval_, newval_, slope_;
	bool once = true;

	K2A(Thread& th, int n, Arg vals) : Gen(th, itemTypeZ, vals.isFinite()), vals_(vals),
		n_(n), remain_(0), slopeFactor_(1./n)
	{
	}

	virtual const char* TypeName() const override { return "K2A"; }

	virtual void pull(Thread& th) override
	{
		if (once) {
			once = false;
			vals_.onez(th, oldval_);
		}
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		
		while (framesToFill) {
			if (remain_ == 0) {
				if (vals_.onez(th, newval_) ) {
					setDone();
					goto leave;
				}
				slope_ = slopeFactor_ * (newval_ - oldval_);
				remain_ = n_;
			}
			int n = std::min(remain_, framesToFill);
			for (int i = 0; i < n; ++i) {
				out[i] = oldval_;
				oldval_ += slope_;
			}
			framesToFill -= n;
			remain_ -= n;
			out += n;
		}
leave:
		produce(framesToFill);
	}
};

struct K2AC : public Gen
{
	BothIn vals_;
	int n_;
	int remain_;
	Z y0, y1, y2, y3;
	Z c0, c1, c2, c3;
	Z phase_;
	Z slope_;
	bool once = true;

	K2AC(Thread& th, int n, Arg vals)
		: Gen(th, itemTypeZ, vals.isFinite()), vals_(vals), n_(n), remain_(0),
		phase_(0), slope_(1./n)
	{
	}

	virtual const char* TypeName() const override { return "K2AC"; }

	virtual void pull(Thread& th) override
	{	
		if (once) {
			once = false;
			y0 = 0.;
			y1 = 0.;
			vals_.onez(th, y2);
			vals_.onez(th, y3);
		}
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		Z x = phase_;

		while (framesToFill) {
			if (remain_ == 0) {
				x = 0.;
				y0 = y1;
				y1 = y2;
				y2 = y3;
				if (vals_.onez(th, y3)) {
					setDone();
					goto leave;
				}
				c0 = y1;
				c1 = .5 * (y2 - y0);
				c2 = y0 - 2.5 * y1 + 2. * y2 - .5 * y3;
				c3 = 1.5 * (y1 - y2) + .5 * (y3 - y0);
				remain_ = n_;
			}
			int n = std::min(remain_, framesToFill);
			for (int i = 0; i < n; ++i) {
				out[i] = ((c3 * x + c2) * x + c1) * x + c0;
				x += slope_;
			}
			framesToFill -= n;
			remain_ -= n;
			out += n;
		}
leave:
		produce(framesToFill);
		phase_ = x;
	}
};


static void k2a_(Thread& th, Prim* prim)
{
	int n = (int)th.popInt("kr : n");
	V a = th.popZIn("kr : signal");
	
	th.push(new List(new K2A(th, n, a)));
}

static void k2ac_(Thread& th, Prim* prim)
{
	int n = (int)th.popInt("krc : n");
	V a = th.popZIn("krc : signal");
	
	th.push(new List(new K2AC(th, n, a)));
}

P<Prim> gK2A;
P<Prim> gK2AC;

static void kr_(Thread& th, Prim* prim)
{
	int n = (int)th.popInt("kr : n");
	V fun = th.pop();
	
	if (n <= 0) {
		post("krc : n <= 0\n");
		throw errOutOfRange;
	}
	if (n > th.rate.blockSize) {
		post("krc : n > block size\n");
		throw errOutOfRange;
	}
	if (th.rate.blockSize % n != 0) {
		post("kr : %d is not a divisor of the current signal block size %d\n", n, th.rate.blockSize);
		throw errFailed;
	}
	
	V result;
	{
		SaveStack ss(th);
		Rate subRate(th.rate, n);
		{
			UseRate ur(th, subRate);
			fun.apply(th);
		}
		result = th.pop();
		{
			SaveStack ss2(th);
			th.push(result);
			th.push(n);
			gK2A->apply_n(th, 2);
			result = th.pop();
		}
	}
	th.push(result);
}

static void krc_(Thread& th, Prim* prim)
{
	int64_t n = th.popInt("kr : n");
	V fun = th.pop();
	
	if (n <= 0) {
		post("krc : n <= 0\n");
		throw errOutOfRange;
	}
	if (n > th.rate.blockSize) {
		post("krc : n > block size\n");
		throw errOutOfRange;
	}
	if (th.rate.blockSize % n != 0) {
		post("krc : %d is not a divisor of the current signal block size %d\n", n, th.rate.blockSize);
		throw errFailed;
	}
	
	V result;
	{
		SaveStack ss(th);
		Rate subRate(th.rate, (int)n);
		{
			UseRate ur(th, subRate);
			fun.apply(th);
		}
		result = th.pop();
		{
			SaveStack ss2(th);
			th.push(result);
			th.push(n);
			gK2AC->apply_n(th, 2);
			result = th.pop();
		}
	}
	th.push(result);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

struct LFNoise0 : public Gen
{
	ZIn rate_;
	Z val_;
	Z phase_;
	Z freqmul_;

	LFNoise0(Thread& th, Arg rate) : Gen(th, itemTypeZ, true), rate_(rate),
		phase_(1.), freqmul_(th.rate.invSampleRate)
	{
	}

	virtual const char* TypeName() const override { return "LFNoise0"; }

	virtual void pull(Thread& th) override
	{	
		RGen& r = th.rgen;
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		Z x = phase_;
		Z freqmul = freqmul_;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}

			for (int i = 0; i < n; ++i) {
				while (x >= 1.) {
					x -= 1.;
					val_ = r.drand2();
				}
				out[i] = val_;

				x += *rate * freqmul;
				rate += rateStride;
			}
			framesToFill -= n;
			out += n;
			rate_.advance(n);
		}
		produce(framesToFill);
		phase_ = x;
	}
};

struct LFNoise1 : public Gen
{
	ZIn rate_;
	Z oldval_, newval_;
	Z slope_;
	Z phase_;
	Z freqmul_;

	LFNoise1(Thread& th, Arg rate) : Gen(th, itemTypeZ, true), rate_(rate),
		phase_(1.), freqmul_(th.rate.invSampleRate)
	{
		RGen& r = th.rgen;
		newval_ = oldval_ = r.drand2();
	}

	virtual const char* TypeName() const override { return "LFNoise1"; }

	virtual void pull(Thread& th) override
	{	
		RGen& r = th.rgen;
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		Z x = phase_;
		Z freqmul = freqmul_;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}

			for (int i = 0; i < n; ++i) {
				while (x >= 1.) {
					x -= 1.;
					oldval_ = newval_;
					newval_ = r.drand2();
					slope_ = newval_ - oldval_;
				}
				out[i] = oldval_ + slope_ * x;

				x += *rate * freqmul;
				rate += rateStride;
			}
			framesToFill -= n;
			out += n;
			rate_.advance(n);
		}
		produce(framesToFill);
		phase_ = x;
	}
};

struct LFNoise3 : public Gen
{
	ZIn rate_;
	Z y0, y1, y2, y3;
	Z c0, c1, c2, c3;
	Z phase_;
	Z freqmul_;

	LFNoise3(Thread& th, Arg rate) : Gen(th, itemTypeZ, true), rate_(rate),
		phase_(1.), freqmul_(th.rate.invSampleRate)
	{
		RGen& r = th.rgen;
		y1 = r.drand2();
		y2 = r.drand2();
		y3 = r.drand2();
	}

	virtual const char* TypeName() const override { return "LFNoise3"; }

	virtual void pull(Thread& th) override
	{	
		RGen& r = th.rgen;
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		Z x = phase_;
		Z freqmul = freqmul_;
		while (framesToFill) {
			Z* rate;
			int n = framesToFill;
			int rateStride;
			if (rate_(th, n, rateStride, rate)) {
				setDone();
				break;
			}

			for (int i = 0; i < n; ++i) {
				while (x >= 1.) {
					x -= 1.;
					y0 = y1;
					y1 = y2;
					y2 = y3;
					y3 = r.drand2() * 0.8; // 0.8 because cubic interpolation can overshoot up to 1.25 if inputs are -1,1,1,-1.
					c0 = y1;
					c1 = .5 * (y2 - y0);
					c2 = y0 - 2.5 * y1 + 2. * y2 - .5 * y3;
					c3 = 1.5 * (y1 - y2) + .5 * (y3 - y0);
				}

				out[i] = ((c3 * x + c2) * x + c1) * x + c0;

				x += *rate * freqmul;
				rate += rateStride;
			}
			framesToFill -= n;
			out += n;
			rate_.advance(n);
		}
		produce(framesToFill);
		phase_ = x;
	}
};


static void lfnoise0_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("lfnoise0 : freq");

	th.push(new List(new LFNoise0(th, rate)));
}

static void lfnoise1_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("lfnoise1 : freq");

	th.push(new List(new LFNoise1(th, rate)));
}

static void lfnoise3_(Thread& th, Prim* prim)
{
	V rate = th.popZIn("lfnoise3 : freq");

	th.push(new List(new LFNoise3(th, rate)));
}


////////////////////////////////////////////////////////////////////////////////////////////////////////


template <class F>
struct SymmetricEnv : public Gen
{
	Z xinc;
	Z x;
	int64_t n_;
	
	SymmetricEnv<F>(Thread& th, Z dur, Z scale) : Gen(th, itemTypeZ, true), x(-scale)
	{
		Z n = std::max(1., floor(dur * th.rate.sampleRate + .5));
		n_ = (int64_t)n;
		xinc = 2. * scale / n;
	}
		
	virtual void pull(Thread& th) override 
	{
		int n = (int)std::min(n_, (int64_t)mBlockSize);
		Z* out = mOut->fulfillz(n);
		static_cast<F*>(this)->F::calc(n, out);
		mOut = mOut->nextp();
		n_ -= n;
		if (n_ == 0) {
			end();
		}
	}
};

template <class F>
struct TriggeredSymmetricEnv : public Gen
{
	ZIn trig_;
	BothIn dur_;
	BothIn amp_;
	Z xinc;
	Z x;
	Z scale_;
	Z ampval_;
	int64_t n_ = 0;
	
	TriggeredSymmetricEnv<F>(Thread& th, Arg trig, Arg dur, Arg amp, Z scale)
		: Gen(th, itemTypeZ, true), trig_(trig), dur_(dur), amp_(amp), scale_(scale)
	{
	}

	virtual void pull(Thread& th) override
	{
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		while (framesToFill) {
			Z* trig;
			int n = framesToFill;
			int trigStride;
			if (trig_(th, n, trigStride, trig)) {
				setDone();
				break;
			}
			if (n_) {
				n = (int)std::min((int64_t)n, n_);
				static_cast<F*>(this)->F::calc(n, ampval_, out);
				n_ -= n;
				trig += n * trigStride;
			} else {
				for (int i = 0; i < n;) {
					if (*trig > 0.) {
						Z dur;
						if (dur_.onez(th, dur)) {
							setDone();
							produce(framesToFill - i);
							return;
						}
						if (amp_.onez(th, ampval_)) {
							setDone();
							produce(framesToFill - i);
							return;
						}
						x = -scale_;
						Z zn = std::max(1., floor(dur * th.rate.sampleRate + .5));
						n_ = (int64_t)zn;
						xinc = 2. * scale_ / zn;
						int n2 = (int)std::min((int64_t)(n-i), n_);
						static_cast<F*>(this)->F::calc(n2, ampval_, out+i);
						n_ -= n2;
						trig += n2 * trigStride;
						i += n2;
					} else {
						out[i] = 0.;
						++i;
						trig += trigStride;
					}
				}
			}
			framesToFill -= n;
			out += n;
			trig_.advance(n);
		}
	//ended:
		produce(framesToFill);
	}
};

struct TriggeredSignal : public Gen
{
	ZIn trig_;
	ZIn list_;
	BothIn amp_;
	V in_;
	Z ampval_;
	bool waiting_ = true;
	Z counter_ = 0.;
	
	TriggeredSignal(Thread& th, Arg trig, Arg in, Arg amp)
		: Gen(th, itemTypeZ, true), trig_(trig), amp_(amp), in_(in)
	{
	}

	virtual const char* TypeName() const override { return "TriggeredSignal"; }

	virtual void pull(Thread& th) override
	{
		Z* out = mOut->fulfillz(mBlockSize);
		int framesToFill = mBlockSize;
		while (framesToFill) {
			Z* trig;
			int n = framesToFill;
			int trigStride;
			if (trig_(th, n, trigStride, trig)) {
				setDone();
				break;
			}
			if (!waiting_) {
				Z* list;
				int listStride;
				if (list_(th, n, listStride, list)) {
					waiting_ = true;
					goto waiting;
				}
				Z amp = ampval_;
				for (int i = 0; i < n; ++i) {
					out[i] = amp * *list;
					list += listStride;
				}
				list_.advance(n);
			} else {
		waiting:
				for (int i = 0; i < n;) {
					if (*trig > 0.) {
						if (amp_.onez(th, ampval_)) {
							setDone();
							produce(framesToFill - i);
							return;
						}
						V in = in_;
						if (in.isFunOrPrim()) {
							SaveStack ss(th);
							try {
								th.push(counter_);
								in.apply(th);
							} catch (...) {
								setDone();
								produce(framesToFill);
								throw;
							}
							in = th.pop();
						}
						counter_ += 1.;
						list_.set(in);
						Z* list;
						int listStride;
						int n2 = n-i;
						if (list_(th, n2, listStride, list)) {
							out[i] = 0.;
							++i;
							trig += trigStride;
						} else {
							Z amp = ampval_;
							for (int j = i; j < i+n2; ++j) {
								out[j] = amp * *list;
								list += listStride;
							}
							trig += n2 * trigStride;
							i += n2;
							list_.advance(n2);
							waiting_ = i < n;
						}
					} else {
						out[i] = 0.;
						++i;
						trig += trigStride;
					}
				}
			}
			framesToFill -= n;
			out += n;
			trig_.advance(n);
		}
	//ended:
		produce(framesToFill);
	}
};


static void tsig_(Thread& th, Prim* prim)
{
	V amp = th.pop();
	V in = th.pop();
	V trig = th.popZIn("tsig : trig");

	th.push(new List(new TriggeredSignal(th, trig, in, amp)));
}

struct ParEnv : public SymmetricEnv<ParEnv>
{
	ParEnv(Thread& th, Z dur) : SymmetricEnv<ParEnv>(th, dur, 1.) {}
	
	virtual const char* TypeName() const override { return "ParEnv"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			Z x2 = x*x;
			out[i] = 1. - x2;
			x += xinc;
		}
	}
};

static void parenv_(Thread& th, Prim* prim)
{
	Z dur = th.popFloat("parenv : dur");

	th.push(new List(new ParEnv(th, dur)));
}

struct TParEnv : public TriggeredSymmetricEnv<TParEnv>
{
	TParEnv(Thread& th, Arg trig, Arg dur, Arg amp) : TriggeredSymmetricEnv<TParEnv>(th, trig, dur, amp, 1.) {}
	
	virtual const char* TypeName() const override { return "TParEnv"; }
	
	virtual void calc(int n, Z amp, Z* out)
	{
		for (int i = 0; i < n; ++i) {
			Z x2 = x*x;
			out[i] = amp * (1. - x2);
			x += xinc;
		}
	}
};

static void tparenv_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("tparenv : amp");
	V dur = th.popZIn("tparenv : dur");
	V trig = th.popZIn("tparenv : trig");

	th.push(new List(new TParEnv(th, trig, dur, amp)));
}

struct QuadEnv : public SymmetricEnv<QuadEnv>
{
	QuadEnv(Thread& th, Z dur) : SymmetricEnv<QuadEnv>(th, dur, 1.) {}
	
	virtual const char* TypeName() const override { return "QuadEnv"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			Z x2 = x*x;
			out[i] = 1. - x2*x2;
			x += xinc;
		}
	}
};

static void quadenv_(Thread& th, Prim* prim)
{
	Z dur = th.popFloat("quadenv : dur");

	th.push(new List(new QuadEnv(th, dur)));
}

struct TQuadEnv : public TriggeredSymmetricEnv<TQuadEnv>
{
	TQuadEnv(Thread& th, Arg trig, Arg dur, Arg amp) : TriggeredSymmetricEnv<TQuadEnv>(th, trig, dur, amp, 1.) {}
	
	virtual const char* TypeName() const override { return "TQuadEnv"; }
	
	virtual void calc(int n, Z amp, Z* out)
	{
		for (int i = 0; i < n; ++i) {
			Z x2 = x*x;
			out[i] = amp * (1. - x2*x2);
			x += xinc;
		}
	}
};

static void tquadenv_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("tquadenv : amp");
	V dur = th.popZIn("tquadenv : dur");
	V trig = th.popZIn("tquadenv : trig");

	th.push(new List(new TQuadEnv(th, trig, dur, amp)));
}


struct OctEnv : public SymmetricEnv<OctEnv>
{
	OctEnv(Thread& th, Z dur) : SymmetricEnv<OctEnv>(th, dur, 1.) {}
	
	virtual const char* TypeName() const override { return "OctEnv"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			Z x2 = x*x;
			Z x4 = x2*x2;
			
			out[i] = 1. - x4*x4;
			
			x += xinc;
		}
	}
};

static void octenv_(Thread& th, Prim* prim)
{
	Z dur = th.popFloat("octenv : dur");

	th.push(new List(new OctEnv(th, dur)));
}

struct TOctEnv : public TriggeredSymmetricEnv<TOctEnv>
{
	TOctEnv(Thread& th, Arg trig, Arg dur, Arg amp) : TriggeredSymmetricEnv<TOctEnv>(th, trig, dur, amp, 1.) {}
	
	virtual const char* TypeName() const override { return "TOctEnv"; }
	
	virtual void calc(int n, Z amp, Z* out)
	{
		for (int i = 0; i < n; ++i) {
			Z x2 = x*x;
			Z x4 = x2*x2;
			out[i] = amp * (1. - x4*x4);
			x += xinc;
		}
	}
};

static void toctenv_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("toctenv : amp");
	V dur = th.popZIn("toctenv : dur");
	V trig = th.popZIn("toctenv : trig");

	th.push(new List(new TOctEnv(th, trig, dur, amp)));
}

struct TriEnv : public SymmetricEnv<TriEnv>
{
	TriEnv(Thread& th, Z dur) : SymmetricEnv<TriEnv>(th, dur, 1.) {}
	
	virtual const char* TypeName() const override { return "TriEnv"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = 1. - fabs(x);
			x += xinc;
		}
	}
};

static void trienv_(Thread& th, Prim* prim)
{
	Z dur = th.popFloat("trienv : dur");

	th.push(new List(new TriEnv(th, dur)));
}

struct TTriEnv : public TriggeredSymmetricEnv<TTriEnv>
{
	TTriEnv(Thread& th, Arg trig, Arg dur, Arg amp) : TriggeredSymmetricEnv<TTriEnv>(th, trig, dur, amp, 1.) {}
	
	virtual const char* TypeName() const override { return "TTriEnv"; }
	
	virtual void calc(int n, Z amp, Z* out)
	{
		for (int i = 0; i < n; ++i) {
			out[i] = amp * (1. - fabs(x));
			x += xinc;
		}
	}
};

static void ttrienv_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("ttrienv : amp");
	V dur = th.popZIn("ttrienv : dur");
	V trig = th.popZIn("ttrienv : trig");

	th.push(new List(new TTriEnv(th, trig, dur, amp)));
}

struct Tri2Env : public SymmetricEnv<Tri2Env>
{
	Tri2Env(Thread& th, Z dur) : SymmetricEnv<Tri2Env>(th, dur, 1.) {}
	
	virtual const char* TypeName() const override { return "Tri2Env"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			Z y = 1. - fabs(x);
			out[i] = y*y;
			x += xinc;
		}
	}
};

static void tri2env_(Thread& th, Prim* prim)
{
	Z dur = th.popFloat("tri2env : dur");

	th.push(new List(new Tri2Env(th, dur)));
}

struct TTri2Env : public TriggeredSymmetricEnv<TTri2Env>
{
	TTri2Env(Thread& th, Arg trig, Arg dur, Arg amp) : TriggeredSymmetricEnv<TTri2Env>(th, trig, dur, amp, 1.) {}
	
	virtual const char* TypeName() const override { return "TTri2Env"; }
	
	virtual void calc(int n, Z amp, Z* out)
	{
		for (int i = 0; i < n; ++i) {
			Z y = 1. - fabs(x);
			out[i] = amp * y*y;
			x += xinc;
		}
	}
};

static void ttri2env_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("ttri2env : amp");
	V dur = th.popZIn("ttri2env : dur");
	V trig = th.popZIn("ttri2env : trig");

	th.push(new List(new TTri2Env(th, trig, dur, amp)));
}


struct TrapezEnv : public SymmetricEnv<TrapezEnv>
{
	TrapezEnv(Thread& th, Z dur) : SymmetricEnv<TrapezEnv>(th, dur, 1.) {}
	
	virtual const char* TypeName() const override { return "TrapezEnv"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = 2. - fabs(x-.5) - fabs(x+.5);
			x += xinc;
		}
	}
};

static void trapezenv_(Thread& th, Prim* prim)
{
	Z dur = th.popFloat("trapezenv : dur");

	th.push(new List(new TrapezEnv(th, dur)));
}


struct TTrapezEnv : public TriggeredSymmetricEnv<TTrapezEnv>
{
	TTrapezEnv(Thread& th, Arg trig, Arg dur, Arg amp) : TriggeredSymmetricEnv<TTrapezEnv>(th, trig, dur, amp, 1.) {}
	
	virtual const char* TypeName() const override { return "TTrapezEnv"; }
	
	virtual void calc(int n, Z amp, Z* out)
	{
		for (int i = 0; i < n; ++i) {
			Z y = 2. - fabs(x-.5) - fabs(x+.5);
			out[i] = amp * y;
			x += xinc;
		}
	}
};

static void ttrapezenv_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("ttrapezenv : amp");
	V dur = th.popZIn("ttrapezenv : dur");
	V trig = th.popZIn("ttrapezenv : trig");

	th.push(new List(new TTrapezEnv(th, trig, dur, amp)));
}


struct Trapez2Env : public SymmetricEnv<Trapez2Env>
{
	Trapez2Env(Thread& th, Z dur) : SymmetricEnv<Trapez2Env>(th, dur, 1.) {}
	
	virtual const char* TypeName() const override { return "Trapez2Env"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			Z y = 2. - fabs(x-.5) - fabs(x+.5);
			out[i] = y*y;
			x += xinc;
		}
	}
};

static void trapez2env_(Thread& th, Prim* prim)
{
	Z dur = th.popFloat("trapez2env : dur");

	th.push(new List(new Trapez2Env(th, dur)));
}

struct TTrapez2Env : public TriggeredSymmetricEnv<TTrapez2Env>
{
	TTrapez2Env(Thread& th, Arg trig, Arg dur, Arg amp) : TriggeredSymmetricEnv<TTrapez2Env>(th, trig, dur, amp, 1.) {}
	
	virtual const char* TypeName() const override { return "TTrapez2Env"; }
	
	virtual void calc(int n, Z amp, Z* out)
	{
		for (int i = 0; i < n; ++i) {
			Z y = 2. - fabs(x-.5) - fabs(x+.5);
			out[i] = amp * y*y;
			x += xinc;
		}
	}
};

static void ttrapez2env_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("ttrapez2env : amp");
	V dur = th.popZIn("ttrapez2env : dur");
	V trig = th.popZIn("ttrapez2env : trig");

	th.push(new List(new TTrapez2Env(th, trig, dur, amp)));
}



struct CosEnv : public SymmetricEnv<CosEnv>
{
	CosEnv(Thread& th, Z dur) : SymmetricEnv<CosEnv>(th, dur, M_PI_2) {}
	
	virtual const char* TypeName() const override { return "CosEnv"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = cos(x);
			x += xinc;
		}
	}
};

static void cosenv_(Thread& th, Prim* prim)
{
	Z dur = th.popFloat("cosenv : dur");

	th.push(new List(new CosEnv(th, dur)));
}


struct TCosEnv : public TriggeredSymmetricEnv<TCosEnv>
{
	TCosEnv(Thread& th, Arg trig, Arg dur, Arg amp) : TriggeredSymmetricEnv<TCosEnv>(th, trig, dur, amp, 1.) {}
	
	virtual const char* TypeName() const override { return "TCosEnv"; }
	
	virtual void calc(int n, Z amp, Z* out)
	{
		for (int i = 0; i < n; ++i) {
			out[i] = amp * cos(x);
			x += xinc;
		}
	}
};

static void tcosenv_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("tcosenv : amp");
	V dur = th.popZIn("tcosenv : dur");
	V trig = th.popZIn("tcosenv : trig");

	th.push(new List(new TCosEnv(th, trig, dur, amp)));
}



struct HanEnv : public SymmetricEnv<HanEnv>
{
	HanEnv(Thread& th, Z dur) : SymmetricEnv<HanEnv>(th, dur, M_PI_2) {}
	
	virtual const char* TypeName() const override { return "HanEnv"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			Z y = cos(x);
			out[i] = y*y;
			x += xinc;
		}
	}
};

static void hanenv_(Thread& th, Prim* prim)
{
	Z dur = th.popFloat("hanenv : dur");

	th.push(new List(new HanEnv(th, dur)));
}


struct THanEnv : public TriggeredSymmetricEnv<THanEnv>
{
	THanEnv(Thread& th, Arg trig, Arg dur, Arg amp) : TriggeredSymmetricEnv<THanEnv>(th, trig, dur, amp, 1.) {}
	
	virtual const char* TypeName() const override { return "THanEnv"; }
	
	virtual void calc(int n, Z amp, Z* out)
	{
		for (int i = 0; i < n; ++i) {
			Z y = cos(x);
			out[i] = amp * y*y;
			x += xinc;
		}
	}
};

static void thanenv_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("thanenv : amp");
	V dur = th.popZIn("thanenv : dur");
	V trig = th.popZIn("thanenv : trig");

	th.push(new List(new THanEnv(th, trig, dur, amp)));
}



struct Han2Env : public SymmetricEnv<Han2Env>
{
	Han2Env(Thread& th, Z dur) : SymmetricEnv<Han2Env>(th, dur, M_PI_2) {}
	
	virtual const char* TypeName() const override { return "Han2Env"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			Z y = cos(x);
			Z y2 = y*y;
			out[i] = y2*y2;
			x += xinc;
		}
	}
};

static void han2env_(Thread& th, Prim* prim)
{
	Z dur = th.popFloat("han2env : dur");

	th.push(new List(new Han2Env(th, dur)));
}


struct THan2Env : public TriggeredSymmetricEnv<THan2Env>
{
	THan2Env(Thread& th, Arg trig, Arg dur, Arg amp) : TriggeredSymmetricEnv<THan2Env>(th, trig, dur, amp, 1.) {}
	
	virtual const char* TypeName() const override { return "THan2Env"; }
	
	virtual void calc(int n, Z amp, Z* out)
	{
		for (int i = 0; i < n; ++i) {
			Z y = cos(x);
			Z y2 = y*y;
			out[i] = amp * y2*y2;
			x += xinc;
		}
	}
};

static void than2env_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("than2env : amp");
	V dur = th.popZIn("than2env : dur");
	V trig = th.popZIn("than2env : trig");

	th.push(new List(new THan2Env(th, trig, dur, amp)));
}



struct GaussEnv : public SymmetricEnv<GaussEnv>
{
	Z widthFactor;
	GaussEnv(Thread& th, Z dur, Z width) : SymmetricEnv<GaussEnv>(th, dur, 1.), widthFactor(-1. / (2. * width * width)) {}
	
	virtual const char* TypeName() const override { return "GaussEnv"; }
	
	virtual void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = exp(x * x * widthFactor);
			x += xinc;
		}
	}
};

static void gaussenv_(Thread& th, Prim* prim)
{
	Z width = th.popFloat("gaussenv : width");
	Z dur = th.popFloat("gaussenv : dur");

	th.push(new List(new GaussEnv(th, dur, width)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Pause : public Gen
{
	ZIn _in;
	ZIn _amp;
	
	Pause(Thread& th, Arg in, Arg amp)
		: Gen(th, itemTypeZ, amp.isFinite()), _in(in), _amp(amp)
	{
	}
	
	virtual const char* TypeName() const override { return "Pause"; }
	
	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			Z *amp;
			int n, ampStride;
			n = framesToFill;
			if (_amp(th, n, ampStride, amp) ) {
				setDone();
				goto leave;
			}
			
			int framesThisTime = n;
			while (framesThisTime) {
				int zerolen = 0;
				for (int i = 0; i < framesThisTime && *amp <= 0.; ++i) {
					*out++ = 0.;
					amp += ampStride;
					++zerolen;
				}
				framesThisTime -= zerolen;
				
				int seglen = 0;
				for (int i = 0; i < framesThisTime && *amp > 0.; ++i) {
					++seglen;
					amp += ampStride;
				}
				amp -= seglen * ampStride;

				int seglenRemain = seglen;
				while (seglenRemain) {
					Z *in;
					int n2, inStride;
					n2 = seglenRemain;
					if (_in(th, n2, inStride, in) ) {
						setDone();
						goto leave;
					}
					for (int i = 0; i < n2; ++i) {
						out[i] = *amp * *in;
						in += inStride;
						amp += ampStride;
					}
					out += n2;
					_in.advance(n2);
					seglenRemain -= n2;
				}
				framesThisTime -= seglen;
				
			}
			framesToFill -= n;
			_amp.advance(n);
		}
leave:
		produce(framesToFill);
	}
	
};

static void pause_(Thread& th, Prim* prim)
{
	V amp   = th.popZIn("pause : amp");
	V in   = th.popZIn("pause : in");

	th.push(new List(new Pause(th, in, amp)));
}



////////////////////////////////////////////////////////////////////////////////////////////////////////

P<String> s_tempo;
P<String> s_dt;
P<String> s_out;

class OverlapAddInputSource;
class OverlapAddOutputChannel;

class OverlapAddBase : public Object
{
protected:
	OverlapAddOutputChannel* mOutputs = nullptr;
	P<OverlapAddInputSource> mActiveSources;
	bool mFinished = false;
	bool mNoMoreSources = false;
	int mNumChannels;
public:
    OverlapAddBase(int numChannels);
    virtual ~OverlapAddBase();

	virtual const char* TypeName() const override { return "OverlapAddBase"; }

	virtual bool pull(Thread& th);
	virtual void addNewSources(Thread& th, int blockSize) = 0;

	P<List> createOutputs(Thread& th);
    void fulfillOutputs(int blockSize);
    void produceOutputs(int shrinkBy);
    int renderActiveSources(Thread& th, int blockSize, bool& anyDone);
    void removeInactiveSources();
};

class OverlapAdd : public OverlapAddBase
{
protected:
	VIn mSounds;
	BothIn mHops;
	ZIn mRate;
	Z mBeatTime;
	Z mNextEventBeatTime;
	Z mEventCounter;
	Z mRateMul;
	int64_t mSampleTime;
	int64_t mPrevChaseTime;
	
	P<Form> mChasedSignals;

public:
	OverlapAdd(Thread& th, Arg sounds, Arg hops, Arg rate, P<Form> const& chasedSignals, int numChannels);
	virtual ~OverlapAdd() {}
	
	virtual const char* TypeName() const override { return "OverlapAdd"; }
		
	virtual void addNewSources(Thread& th, int blockSize) override;
	void chaseToTime(Thread& th, int64_t inSampleTime);
};

class OverlapAddInputSource : public Object
{
public:
	P<OverlapAddInputSource> mNextSource;
	std::vector<ZIn> mInputs;
	int mOffset;
	bool mSourceDone;
	
	OverlapAddInputSource(Thread& th, List* channels, int inOffset, P<OverlapAddInputSource> const& inNextSource) 
		: mNextSource(inNextSource), mOffset(inOffset), mSourceDone(false)
	{
		if (channels->isVList()) {
			P<List> packedChannels = channels->pack(th);
			Array* a = packedChannels->mArray();
			
			// put channels into mInputs
			mInputs.reserve(a->size());
			for (int i = 0; i < a->size(); ++i) {
				ZIn zin(a->v()[i]);
				mInputs.push_back(zin);
			}
		} else {
			ZIn zin(channels);
			mInputs.push_back(zin);
		}
	}

	virtual const char* TypeName() const override { return "OverlapAdd"; }
};

class OverlapAddOutputChannel : public Gen
{
	friend class OverlapAddBase;
	P<OverlapAddBase> mOverlapAddBase;
	OverlapAddOutputChannel* mNextOutput;
	
public:	
	OverlapAddOutputChannel(Thread& th, OverlapAddBase* inOverlapAdd)
        : Gen(th, itemTypeZ, false), mOverlapAddBase(inOverlapAdd), mNextOutput(nullptr)
	{
	}

	
	virtual void norefs() override
	{
		mOut = nullptr;
		mOverlapAddBase = nullptr;
	}
		
	virtual const char* TypeName() const override { return "OverlapAddOutputChannel"; }
	
	virtual void pull(Thread& th) override
	{
		if (mOverlapAddBase->pull(th)) {
			end();
		}
	}
	
};

OverlapAddBase::OverlapAddBase(int numChannels)
	: mNumChannels(numChannels)
{
}

OverlapAddBase::~OverlapAddBase()
{
	OverlapAddOutputChannel* output = mOutputs;
	do {
		OverlapAddOutputChannel* next = output->mNextOutput;
		delete output;
		output = next;
	} while (output);
}

OverlapAdd::OverlapAdd(Thread& th, Arg sounds, Arg hops, Arg rate, P<Form> const& chasedSignals, int numChannels)
	: OverlapAddBase(numChannels),
    mSounds(sounds), mHops(hops), mRate(rate),
	mBeatTime(0.), mNextEventBeatTime(0.), mEventCounter(0.), mRateMul(th.rate.invSampleRate),
	mSampleTime(0), mPrevChaseTime(0),
	mChasedSignals(chasedSignals)
{
}

P<List> OverlapAddBase::createOutputs(Thread& th)
{
	P<List> s = new List(itemTypeV, mNumChannels);
	
	// fill s->mArray with ola's output channels.
    OverlapAddOutputChannel* last = nullptr;
	P<Array> a = s->mArray;
	for (int i = 0; i < mNumChannels; ++i) {
        OverlapAddOutputChannel* c = new OverlapAddOutputChannel(th, this);
        if (last) last->mNextOutput = c;
        else mOutputs = c;
        last = c;
		a->add(new List(c));
	}
	
	return s;
}

void OverlapAdd::addNewSources(Thread& th, int blockSize)
{			
	// integrate tempo and add new sources.
	Z* rate;
	int rateStride;
	if (mRate(th, blockSize, rateStride, rate)) {
		mNoMoreSources = true;
	} else if (!mNoMoreSources) {
		Z beatTime = mBeatTime;
		Z nextEventBeatTime = mNextEventBeatTime;
		Z ratemul = mRateMul;
		for (int i = 0; i < blockSize; ++i) {
			while (beatTime >= nextEventBeatTime) {
			
				chaseToTime(th, mSampleTime + i);
			
				V newSource;
				if (mSounds.one(th, newSource)) {
					mNoMoreSources = true;
					break;
				}
							
				if (newSource.isFun()) {
					SaveStack ss(th);	
					th.push(mEventCounter);
					newSource.apply(th);
					newSource = th.pop();
				}
				
				V out;
				Z deltaTime;
				if (mHops.onez(th, deltaTime)) {
					mNoMoreSources = true;
					break;
				}

				if (newSource.isForm()) {
					if (mChasedSignals()) {
						V parents[2];
						parents[0] = mChasedSignals;
						parents[1] = newSource;
						newSource = linearizeInheritance(th, 2, parents);
					}
					newSource.dot(th, s_out, out);
					V hop;
					if (newSource.dot(th, s_dt, hop) && hop.isReal()) {
						deltaTime = hop.f;
					}

				} else {
					out = newSource;
				}
				
				// must be a finite array with fewer than mNumChannels
				if (out.isZList() || (out.isVList() && out.isFinite())) {
					List* s = (List*)out.o();
					// create an active source:
					P<OverlapAddInputSource> source = new OverlapAddInputSource(th, s, i, mActiveSources);
					mActiveSources = source;
				}
								
				nextEventBeatTime += deltaTime;
				mEventCounter += 1.;
			}
			beatTime += *rate * ratemul;
			rate += rateStride;
		}
		mBeatTime = beatTime;
		mNextEventBeatTime = nextEventBeatTime;
		mSampleTime += blockSize;
		
		mRate.advance(blockSize);

		chaseToTime(th, mSampleTime);
	}
}

void OverlapAddBase::fulfillOutputs(int blockSize)
{
	OverlapAddOutputChannel* output = mOutputs;
	do {
		if (output->mOut) {
			Z* out = output->mOut->fulfillz(blockSize);
			memset(out, 0, output->mBlockSize * sizeof(Z));
		}
		output = output->mNextOutput;
	} while (output);
}

int OverlapAddBase::renderActiveSources(Thread& th, int blockSize, bool& anyDone)
{
	int maxProduced = 0;
	OverlapAddInputSource* source = mActiveSources();
	while (source) {
		int offset = source->mOffset;
		int pullSize = blockSize - offset;
		std::vector<ZIn>& sourceChannels = source->mInputs;
		bool allOutputsDone = true; // initial value for reduction on &&
		OverlapAddOutputChannel* output = mOutputs;
		for (size_t j = 0; j < sourceChannels.size() && output; ++j, output = output->mNextOutput) {
			if (output->mOut) {
				ZIn& zin = sourceChannels[j];
				if (zin.mIsConstant && zin.mConstant.f == 0.)
					continue;

				int n = pullSize;
				Z* out = output->mOut->mArray->z() + offset;
				if (!zin.mix(th, n, out)) {
					allOutputsDone = false;
				}
				maxProduced = std::max(maxProduced, n);
			}
		}
		source->mOffset = 0;
		if (allOutputsDone) {
			// mark for removal from mActiveSources
			source->mSourceDone = true;
            anyDone = true;
		}
		source = source->mNextSource();
	}
	return maxProduced;
}

void OverlapAddBase::removeInactiveSources()
{
	P<OverlapAddInputSource> source = mActiveSources();
	P<OverlapAddInputSource> prevSource = nullptr;
	mActiveSources = nullptr;
	
	while (source) {
		P<OverlapAddInputSource> nextSource = source->mNextSource;
		source->mNextSource = nullptr;
		if (!source->mSourceDone) {
			if (prevSource()) prevSource->mNextSource = source;
			else mActiveSources = source;
            prevSource = source;
		}
		source = nextSource;
	}
}

void OverlapAddBase::produceOutputs(int shrinkBy)
{
	OverlapAddOutputChannel* output = mOutputs;
	do {
		if (output->mOut)
			output->produce(shrinkBy);
		output = output->mNextOutput;
	} while (output);
}

bool OverlapAddBase::pull(Thread& th)
{
	if (mFinished) {
		return mFinished;
    }
	
	OverlapAddOutputChannel* output = mOutputs;
	int blockSize = output->mBlockSize;
	addNewSources(th, blockSize);
	
	fulfillOutputs(blockSize);
	
    bool anyDone = false;
	int maxProduced = renderActiveSources(th, blockSize, anyDone);
			
	mFinished = mNoMoreSources && mActiveSources() == nullptr;
	int shrinkBy = mFinished ? blockSize - maxProduced : 0;
	
	produceOutputs(shrinkBy);

	if (anyDone)
        removeInactiveSources();
	
	return mFinished; 
}

void OverlapAdd::chaseToTime(Thread& th, int64_t inSampleTime)
{
	int64_t n = inSampleTime - mPrevChaseTime;
	mPrevChaseTime = inSampleTime;

	if (mChasedSignals() && n > 0) {
		mChasedSignals = mChasedSignals->chaseForm(th, n);
	}
}


const int64_t kMaxOverlapAddChannels = 10000;

static void ola_(Thread& th, Prim* prim)
{
	int64_t numChannels = th.popInt("ola : numChannels");
	V rate = th.pop();
	V hops = th.popZInList("ola : hops");
	V sounds = th.pop();

	if (numChannels > kMaxOverlapAddChannels) {
		post("ola : too many channels\n");
		throw errFailed;
	}
	
	P<Form> chasedSignals;
	if (rate.isForm()) {
		chasedSignals = (Form*)rate.o();
		rate = 1.;
		chasedSignals->dot(th, s_tempo, rate);
	}

	P<OverlapAdd> ola = new OverlapAdd(th, sounds, hops, rate, chasedSignals, (int)numChannels);
	
	th.push(ola->createOutputs(th));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////


class ITD;

class ITD_OutputChannel : public Gen
{
	friend class ITD;
	P<ITD> mITD;
	
public:	
	ITD_OutputChannel(Thread& th, bool inFinite, ITD* inITD);
	
	virtual void norefs() override
	{
		mOut = nullptr;
		mITD = nullptr;
	}
	
	virtual const char* TypeName() const override { return "ITD_OutputChannel"; }
	
	virtual void pull(Thread& th) override;
};

struct ITD : public Gen
{
	ZIn in_;
	ZIn pan_;
	Z maxdelay_;
	Z half;
	int32_t bufSize;
	int32_t bufMask;
	int32_t bufPos;
	Z* buf;
	Z sr;
	ITD_OutputChannel* mLeft;
	ITD_OutputChannel* mRight;
	
	ITD(Thread& th, Arg in, Arg pan, Z maxdelay) : Gen(th, itemTypeZ, false), in_(in), pan_(pan), maxdelay_(maxdelay)
	{
		sr = th.rate.sampleRate;
		half = (int32_t)ceil(sr * maxdelay * .5 + .5);
		bufSize = NEXTPOWEROFTWO(2 * (int)half + 3);
		bufMask = bufSize - 1;
		bufPos = 0;
		buf = (Z*)calloc(bufSize, sizeof(Z));
	}
	
	~ITD() { delete mLeft; delete mRight; free(buf); }
	
	virtual const char* TypeName() const override { return "ITD"; }
	
	P<List> createOutputs(Thread& th);
    
	virtual void pull(Thread& th) override 
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
			int inStride, panStride;
			Z *in, *pan;
			if (in_(th, n, inStride, in) || pan_(th, n, panStride, pan)) {
				mLeft->setDone();
				mRight->setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					{
						Z fpos = std::max(2., *pan * half + half);
						Z ipos = floor(fpos);
						Z frac = fpos - ipos;
						int32_t offset = bufPos-(int32_t)ipos;
						Z a = buf[(offset+1) & bufMask];
						Z b = buf[(offset  ) & bufMask];
						Z c = buf[(offset-1) & bufMask];
						Z d = buf[(offset-2) & bufMask];
						*Lout = lagrangeInterpolate(frac, a, b, c, d);
						Lout += Loutstride;
					}
					{
						Z fpos = std::max(2., -*pan * half + half);
						Z ipos = floor(fpos);
						Z frac = fpos - ipos;
						int32_t offset = bufPos-(int32_t)ipos;
						Z a = buf[(offset+1) & bufMask];
						Z b = buf[(offset  ) & bufMask];
						Z c = buf[(offset-1) & bufMask];
						Z d = buf[(offset-2) & bufMask];
						*Rout = lagrangeInterpolate(frac, a, b, c, d);
						Rout += Routstride;
					}
					buf[bufPos & bufMask] = *in;
					in += inStride;
					pan += panStride;
					++bufPos;
				}
				in_.advance(n);
				pan_.advance(n);
				framesToFill -= n;
			}
		}
		if (mLeft->mOut)  mLeft->produce(framesToFill);
		if (mRight->mOut) mRight->produce(framesToFill);
	}
};

ITD_OutputChannel::ITD_OutputChannel(Thread& th, bool inFinite, ITD* inITD) : Gen(th, itemTypeZ, inFinite), mITD(inITD)
{
}

void ITD_OutputChannel::pull(Thread& th)
{
	mITD->pull(th);
}

P<List> ITD::createOutputs(Thread& th)
{
	mLeft = new ITD_OutputChannel(th, finite, this);
	mRight = new ITD_OutputChannel(th, finite, this);
	
	P<Gen> left = mLeft;
	P<Gen> right = mRight;
	
	P<List> s = new List(itemTypeV, 2);
	P<Array> a = s->mArray;
	a->add(new List(left));
	a->add(new List(right));
	
	return s;
}

static void itd_(Thread& th, Prim* prim)
{
	Z maxdelay = th.popFloat("itd : maxdelay");
	V pan = th.popZIn("itd : pan");
	V in = th.popZIn("itd : in");
    
	P<ITD> itd = new ITD(th, in, pan, maxdelay);

	P<List> s = itd->createOutputs(th);

	th.push(s);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline Z fast_sin1(Z x)
{
	x = x - floor(x + .5);
	
    Z y = x * (8. - 16. * fabs(x));
	y = 0.225 * (y * fabs(y) - y) + y;

	return y;
}

inline Z fast_cos1(Z x)
{
	return fast_sin1(x + .25);
}

inline Z fast_pan(Z x)
{
	Z y = .75 + x * (.5 - .25 * x);
	y = 0.225 * (y * fabs(y) - y) + y;

	return y;
}

struct Pan2Out;

struct Pan2 : public Object
{
	ZIn _in;
	ZIn _pos;
	
	Pan2Out* mLeft;
	Pan2Out* mRight;
		
	Pan2(Thread& th, Arg inIn, Arg inPos)
		: _in(inIn), _pos(inPos)
	{
		finite = mostFinite(inIn, inPos);
	}

	P<List> createOutputs(Thread& th);

	virtual const char* TypeName() const override { return "Pan2"; }

	virtual void pull(Thread& th);
};

struct Pan2Out : public Gen
{
	P<Pan2> mPan2;
	
	Pan2Out(Thread& th, bool inFinite, P<Pan2> const& inPan2) : Gen(th, itemTypeZ, inFinite), mPan2(inPan2)
	{
	}

	virtual void norefs() override
	{
		mOut = nullptr;
		mPan2 = nullptr;
	}
	
	virtual const char* TypeName() const override { return "Pan2Out"; }
	
	virtual void pull(Thread& th) override
	{
		mPan2->pull(th);
	}
	
};

void Pan2::pull(Thread& th)
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
		Z *a, *b;
		int n, aStride, bStride;
		n = framesToFill;
		if (_in(th, n, aStride, a) || _pos(th, n, bStride, b)) {
			mLeft->setDone();
			mRight->setDone();
			break;
		}
		
		if (bStride == 0) {
			Z x = std::clamp(*b, -1., 1.);
			Z Lpan = fast_pan(-x);
			Z Rpan = fast_pan(x);
			for (int i = 0; i < n; ++i) {
				Z z = *a;
				*Lout = z * Lpan;
				*Rout = z * Rpan;
				a += aStride;
				Lout += Loutstride;
				Rout += Routstride;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z x = std::clamp(*b, -1., 1.);
				Z z = *a;
				*Lout = z * fast_pan(-x);
				*Rout = z * fast_pan(x);
				a += aStride;
				b += bStride;
				Lout += Loutstride;
				Rout += Routstride;
			}
		}
		framesToFill -= n;
		_in.advance(n);
		_pos.advance(n);
	}
	if (mLeft->mOut) mLeft->produce(framesToFill);
	if (mRight->mOut) mRight->produce(framesToFill);
}


P<List> Pan2::createOutputs(Thread& th)
{
	mLeft = new Pan2Out(th, finite, this);
	mRight = new Pan2Out(th, finite, this);
	
	P<Gen> left = mLeft;
	P<Gen> right = mRight;
	
	P<List> s = new List(itemTypeV, 2);
	P<Array> a = s->mArray;
	a->add(new List(left));
	a->add(new List(right));
	
	return s;
}


static void pan2_(Thread& th, Prim* prim)
{
	V pos = th.popZIn("pan2 : pos");
	V in = th.popZIn("pan2 : in");

	P<Pan2> pan = new Pan2(th, in, pos);

	P<List> s = pan->createOutputs(th);

	th.push(s);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Balance2Out;

struct Balance2 : public Object
{
	ZIn _L;
	ZIn _R;
	ZIn _pos;
	
	P<Balance2Out> mLeft;
	P<Balance2Out> mRight;
		
	Balance2(Thread& th, Arg inL, Arg inR, Arg inPos)
		: _L(inL), _R(inR), _pos(inPos)
	{
		finite = mostFinite(inL, inR, inPos);
	}

	P<List> createOutputs(Thread& th);

	virtual const char* TypeName() const override { return "Balance2"; }

	virtual void pull(Thread& th);
};

struct Balance2Out : public Gen
{
	P<Balance2> mBalance2;
	
	Balance2Out(Thread& th, bool inFinite, P<Balance2> const& inBalance2) : Gen(th, itemTypeZ, inFinite), mBalance2(inBalance2)
	{
	}

	virtual void norefs() override
	{
		mOut = nullptr;
		mBalance2 = nullptr;
	}
	
	virtual const char* TypeName() const override { return "Balance2Out"; }
	
	virtual void pull(Thread& th) override
	{
		mBalance2->pull(th);
	}
	
};

void Balance2::pull(Thread& th)
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
		Z *a, *b, *c;
		int n, aStride, bStride, cStride;
		n = framesToFill;
		if (_L(th, n, aStride, a) || _R(th, n, bStride, b) || _pos(th, n, cStride, c)) {
			mLeft->setDone();
			mRight->setDone();
			break;
		}
		
		if (cStride == 0) {
			Z x = std::clamp(*c, -1., 1.);
			Z Lpan = fast_pan(-x);
			Z Rpan = fast_pan(x);
			for (int i = 0; i < n; ++i) {
				*Lout = *a * Lpan;
				*Rout = *b * Rpan;
				a += aStride;
				b += bStride;
				Lout += Loutstride;
				Rout += Routstride;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z x = std::clamp(*c, -1., 1.);
				*Lout = *a * fast_pan(-x);
				*Rout = *b * fast_pan(x);
				a += aStride;
				b += bStride;
				c += cStride;
				Lout += Loutstride;
				Rout += Routstride;
			}
		}
		framesToFill -= n;
		_L.advance(n);
		_R.advance(n);
		_pos.advance(n);
	}
	if (mLeft->mOut) mLeft->produce(framesToFill);
	if (mRight->mOut) mRight->produce(framesToFill);
}


P<List> Balance2::createOutputs(Thread& th)
{
	mLeft = new Balance2Out(th, finite, this);
	mRight = new Balance2Out(th, finite, this);
	
	P<Gen> left = mLeft;
	P<Gen> right = mRight;
	
	P<List> s = new List(itemTypeV, 2);
	P<Array> a = s->mArray;
	a->add(new List(left));
	a->add(new List(right));
	
	return s;
}


static void bal2_(Thread& th, Prim* prim)
{
	V pos = th.popZIn("bal2 : pos");
	V R = th.popZIn("bal2 : right");
	V L = th.popZIn("bal2 : left");

	P<Balance2> bal = new Balance2(th, L, R, pos);

	P<List> s = bal->createOutputs(th);

	th.push(s);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


struct Rot2Out;

struct Rot2 : public Object
{
	ZIn _L;
	ZIn _R;
	ZIn _pos;
	
	P<Rot2Out> mLeft;
	P<Rot2Out> mRight;
		
	Rot2(Thread& th, Arg inL, Arg inR, Arg inPos)
		: _L(inL), _R(inR), _pos(inPos)
	{
		finite = mostFinite(inL, inR, inPos);
	}

	P<List> createOutputs(Thread& th);

	virtual const char* TypeName() const override { return "Rot2"; }

	virtual void pull(Thread& th);
};

struct Rot2Out : public Gen
{
	P<Rot2> mRot2;
	
	Rot2Out(Thread& th, bool inFinite, P<Rot2> const& inRot2) : Gen(th, itemTypeZ, inFinite), mRot2(inRot2)
	{
	}


	virtual void norefs() override
	{
		mOut = nullptr;
		mRot2 = nullptr;
	}
	
	virtual const char* TypeName() const override { return "Rot2Out"; }
	
	virtual void pull(Thread& th) override
	{
		mRot2->pull(th);
	}
	
};

void Rot2::pull(Thread& th)
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
		Z *a, *b, *c;
		int n, aStride, bStride, cStride;
		n = framesToFill;
		if (_L(th, n, aStride, a) || _R(th, n, bStride, b) || _pos(th, n, cStride, c)) {
			mLeft->setDone();
			mRight->setDone();
			break;
		}
		
		if (cStride == 0) {
			if (_L.isZero()) {
				Z pos = .5 * *c;
				Z sn = -fast_sin1(pos);
				Z cs = fast_cos1(pos);
				for (int i = 0; i < n; ++i) {
					Z R = *b;
					*Lout = - R * sn;
					*Rout =   R * cs;
					b += bStride;
					Lout += Loutstride;
					Rout += Routstride;
				}
			} else if (_R.isZero()) {
				Z pos = .5 * *c;
				Z sn = -fast_sin1(pos);
				Z cs = fast_cos1(pos);
				for (int i = 0; i < n; ++i) {
					Z L = *a;
					*Lout = L * cs;
					*Rout = L * sn;
					a += aStride;
					Lout += Loutstride;
					Rout += Routstride;
				}
			} else {
				Z pos = .5 * *c;
				Z sn = -fast_sin1(pos);
				Z cs = fast_cos1(pos);
				for (int i = 0; i < n; ++i) {
					Z L = *a;
					Z R = *b;
					*Lout = L * cs - R * sn;
					*Rout = L * sn + R * cs;
					a += aStride;
					b += bStride;
					Lout += Loutstride;
					Rout += Routstride;
				}
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z L = *a;
				Z R = *b;
				Z pos = .5 * *c;
				Z sn = -fast_sin1(pos);
				Z cs = fast_cos1(pos);
				*Lout = L * cs - R * sn;
				*Rout = L * sn + R * cs;
				a += aStride;
				b += bStride;
				c += cStride;
				Lout += Loutstride;
				Rout += Routstride;
			}
		}
		framesToFill -= n;
		_L.advance(n);
		_R.advance(n);
		_pos.advance(n);
	}
	if (mLeft->mOut) mLeft->produce(framesToFill);
	if (mRight->mOut) mRight->produce(framesToFill);
}


P<List> Rot2::createOutputs(Thread& th)
{
	mLeft = new Rot2Out(th, finite, this);
	mRight = new Rot2Out(th, finite, this);
	
	P<Gen> left = mLeft;
	P<Gen> right = mRight;
	
	P<List> s = new List(itemTypeV, 2);
	P<Array> a = s->mArray;
	a->add(new List(left));
	a->add(new List(right));
	
	return s;
}


static void rot2_(Thread& th, Prim* prim)
{
	V pos = th.popZIn("rot2 : pos");
	V R = th.popZIn("rot2 : right");
	V L = th.popZIn("rot2 : left");

	P<Rot2> rot2 = new Rot2(th, L, R, pos);

	P<List> s = rot2->createOutputs(th);

	th.push(s);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


struct Fade2 : public Gen
{
	ZIn _L;
	ZIn _R;
	ZIn _pos;
		
	Fade2(Thread& th, Arg inL, Arg inR, Arg inPos)
		: Gen(th, itemTypeZ, mostFinite(inL, inR, inPos)), _L(inL), _R(inR), _pos(inPos)
	{
	}
	
	virtual const char* TypeName() const override { return "Fade2"; }
	
	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		
		Z* out = mOut->fulfillz(framesToFill);
		
		while (framesToFill) {
			Z *a, *b, *c;
			int n, aStride, bStride, cStride;
			n = framesToFill;
			if (_L(th, n, aStride, a) || _R(th, n, bStride, b) || _pos(th, n, cStride, c)) {
				setDone();
				break;
			}
			
			if (cStride == 0) {
				Z x = std::clamp(*c, -1., 1.);
				Z Lpan = fast_pan(-x);
				Z Rpan = fast_pan(x);
				for (int i = 0; i < n; ++i) {
					out[i] = *a * Lpan + *b * Rpan;
					a += aStride;
					b += bStride;
				}
			} else {
				for (int i = 0; i < n; ++i) {
					Z x = std::clamp(*c, -1., 1.);
					out[i] = *a * fast_pan(-x) + *b * fast_pan(x);
					a += aStride;
					b += bStride;
					c += cStride;
				}
			}
			framesToFill -= n;
			out += n;
			_L.advance(n);
			_R.advance(n);
			_pos.advance(n);
		}
		produce(framesToFill);
	}
	
};

static void fade2_(Thread& th, Prim* prim)
{
	V pos = th.popZIn("fade2 : pos");
	V R = th.popZIn("fade2 : right");
	V L = th.popZIn("fade2 : left");

	th.push(new List(new Fade2(th, L, R, pos)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Trig : public Gen
{
	ZIn _in;
	Z _prev;
	
	Trig(Thread& th, Arg in)
		: Gen(th, itemTypeZ, in.isFinite()), _in(in), _prev(0.)
	{
	}
	
	virtual const char* TypeName() const override { return "Trig"; }
	
	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		
		Z* out = mOut->fulfillz(framesToFill);
		Z prev = _prev;
		while (framesToFill) {
			Z *in;
			int n, inStride;
			n = framesToFill;
			if (_in(th, n, inStride, in)) {
				setDone();
				break;
			}
			
			for (int i = 0; i < n; ++i) {
				
				Z cur = *in;
				out[i] = cur > 0. && prev <= 0. ? 1. : 0.;
				prev = cur;
				in += inStride;
			}
			
			framesToFill -= n;
			out += n;
			_in.advance(n);
		}
		
		_prev = prev;
		produce(framesToFill);
	}
	
};

struct NegTrig : public Gen
{
	ZIn _in;
	Z _prev;
	
	NegTrig(Thread& th, Arg in)
		: Gen(th, itemTypeZ, in.isFinite()), _in(in), _prev(-1.)
	{
	}
	
	virtual const char* TypeName() const override { return "NegTrig"; }
	
	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		
		Z* out = mOut->fulfillz(framesToFill);
		Z prev = _prev;
		while (framesToFill) {
			Z *in;
			int n, inStride;
			n = framesToFill;
			if (_in(th, n, inStride, in)) {
				setDone();
				break;
			}
			
			for (int i = 0; i < n; ++i) {
				
				Z cur = *in;
				out[i] = cur >= 0. && prev < 0. ? 1. : 0.;
				prev = cur;
				in += inStride;
			}
			
			framesToFill -= n;
			out += n;
			_in.advance(n);
		}
		
		_prev = prev;
		produce(framesToFill);
	}
	
};

static void tr_(Thread& th, Prim* prim)
{
	V in   = th.popZIn("tr : in");

	th.push(new List(new Trig(th, in)));
}

static void ntr_(Thread& th, Prim* prim)
{
	V in   = th.popZIn("ntr : in");

	th.push(new List(new NegTrig(th, in)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Gate : public TwoInputUGen<Gate>
{
	Z phase;
	Z freq;
	Gate(Thread& th, Arg trig, Arg hold)
		: TwoInputUGen<Gate>(th, trig, hold), phase(INFINITY), freq(th.rate.invSampleRate)
	{
	}
	
	virtual const char* TypeName() const override { return "Gate"; }
	
	void calc(int n, Z* out, Z* trig, Z* hold, int trigStride, int holdStride) 
	{
		for (int i = 0; i < n; ++i) {
			Z t = *trig;
			if (t > 0.) {
				phase = 0.;
			}
			out[i] = phase < *hold ? 1. : 0.;
			phase += freq;
			trig += trigStride;
			hold += holdStride;
		}
	}
};

static void gate_(Thread& th, Prim* prim)
{
	V hold = th.popZIn("gate : hold");
	V in   = th.popZIn("gate : in");

	th.push(new List(new Gate(th, in, hold)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampleAndHold : public Gen
{
	ZIn _in;
	ZIn _tr;
	Z _val;
	
	SampleAndHold(Thread& th, Arg in, Arg tr)
		: Gen(th, itemTypeZ, mostFinite(in, tr)), _in(in), _tr(tr), _val(0.)
	{
	}
	
	virtual const char* TypeName() const override { return "SampleAndHold"; }
	
	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		
		Z* out = mOut->fulfillz(framesToFill);
		Z val = _val;
		while (framesToFill) {
			Z *in, *tr;
			int n, inStride, trStride;
			n = framesToFill;
			if (_in(th, n, inStride, in) || _tr(th, n, trStride, tr)) {
				setDone();
				break;
			}
			
			for (int i = 0; i < n; ++i) {				
				if (*tr > 0.) val = *in;
				out[i] = val;
				in += inStride;
				tr += trStride;
			}
			
			framesToFill -= n;
			out += n;
			_in.advance(n);
			_tr.advance(n);
		}
		
		_val = val;
		produce(framesToFill);
	}
	
};

static void sah_(Thread& th, Prim* prim)
{
	V trigger   = th.popZIn("sah : trigger");
	V in   = th.popZIn("sah : in");

	th.push(new List(new SampleAndHold(th, in, trigger)));
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Sequencer : public Gen
{
	BothIn _in;
	ZIn _tr;
	Z _val;
	
	Sequencer(Thread& th, Arg in, Arg tr)
		: Gen(th, itemTypeZ, mostFinite(in, tr)), _in(in), _tr(tr), _val(0.)
	{
	}
	
	virtual const char* TypeName() const override { return "Sequencer"; }
	
	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		
		Z* out = mOut->fulfillz(framesToFill);
		Z val = _val;
		while (framesToFill) {
			Z *tr;
			int n, trStride;
			n = framesToFill;
			if (_tr(th, n, trStride, tr)) {
				setDone();
				break;
			}
			
			for (int i = 0; i < n; ++i) {				
				if (*tr > 0.) {
					Z z;
					if (_in.onez(th, z)) {
						setDone();
						produce(framesToFill - i);
						return;
					}
					val = z;
				}
				out[i] = val;
				tr += trStride;
			}
			
			framesToFill -= n;
			out += n;
			_tr.advance(n);
		}
		
		_val = val;
		produce(framesToFill);
	}
	
};

static void seq_(Thread& th, Prim* prim)
{
	V trigger   = th.popZIn("seq : trigger");
	V in   = th.pop();

	th.push(new List(new Sequencer(th, in, trigger)));
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


struct ImpulseSequencer : public Gen
{
	BothIn _in;
	ZIn _tr;
	
	ImpulseSequencer(Thread& th, Arg in, Arg tr)
		: Gen(th, itemTypeZ, mostFinite(in, tr)), _in(in), _tr(tr)
	{
	}
	
	virtual const char* TypeName() const override { return "ImpulseSequencer"; }
	
	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			Z *tr;
			int n, trStride;
			n = framesToFill;
			if (_tr(th, n, trStride, tr)) {
				setDone();
				break;
			}
			
			for (int i = 0; i < n; ++i) {				
				if (*tr > 0.) {
					Z z;
					if (_in.onez(th, z)) {
						setDone();
						produce(framesToFill - i);
						return;
					}
					out[i] = z;
				} else {
					out[i] = 0.;
				}
				tr += trStride;
			}
			
			framesToFill -= n;
			out += n;
			_tr.advance(n);
		}
		
		produce(framesToFill);
	}
	
};

static void iseq_(Thread& th, Prim* prim)
{
	V trigger   = th.popZIn("iseq : trigger");
	V in   = th.pop();

	th.push(new List(new ImpulseSequencer(th, in, trigger)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


struct PulseDivider : public Gen
{
	ZIn _tr;
	ZIn _div;
	Z _count;
	
	PulseDivider(Thread& th, Arg tr, Arg div, Z start)
		: Gen(th, itemTypeZ, mostFinite(tr, div)), _tr(tr), _div(div), _count(start - 1.)
	{
	}
	
	virtual const char* TypeName() const override { return "PulseDivider"; }
	
	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			Z *tr, *div;
			int n, trStride, divStride;
			n = framesToFill;
			if (_tr(th, n, trStride, tr) || _div(th, n, divStride, div)) {
				setDone();
				break;
			}
			
			for (int i = 0; i < n; ++i) {				
				if (*tr > 0.) {
					_count += 1.;
					Z idiv = floor(*div + .5);
					if (_count >= idiv) {
						_count -= idiv;
					}
					out[i] = _count == 0. ? *tr : 0.;
				} else out[i] = 0.;
				tr += trStride;
				div += divStride;
			}
			
			framesToFill -= n;
			out += n;
			_tr.advance(n);
			_div.advance(n);
		}
		
		produce(framesToFill);
	}
	
};

static void pdiv_(Thread& th, Prim* prim)
{
	Z start  = th.popFloat("pdiv : istart");
	V div  = th.popZIn("pdiv : n");
	V in   = th.popZIn("pdiv : in");

	th.push(new List(new PulseDivider(th, in, div, start)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Clip : public ThreeInputUGen<Clip>
{
	
	Clip(Thread& th, Arg a, Arg b, Arg c) : ThreeInputUGen<Clip>(th, a, b, c)
	{
	}
	
	virtual const char* TypeName() const override { return "Clip"; }
	
	void calc(int n, Z* out, Z* a, Z* b, Z* c, int aStride, int bStride, int cStride) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = std::clamp(*a, *b, *c);
			a += aStride;
			b += bStride;
			c += cStride;
		}
	}
};

struct Wrap : public ThreeInputUGen<Wrap>
{
	
	Wrap(Thread& th, Arg a, Arg b, Arg c) : ThreeInputUGen<Wrap>(th, a, b, c)
	{
	}
	
	virtual const char* TypeName() const override { return "Wrap"; }
	
	void calc(int n, Z* out, Z* a, Z* b, Z* c, int aStride, int bStride, int cStride) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = sc_wrap(*a, *b, *c);
			a += aStride;
			b += bStride;
			c += cStride;
		}
	}
};

struct Fold : public ThreeInputUGen<Fold>
{
	
	Fold(Thread& th, Arg a, Arg b, Arg c) : ThreeInputUGen<Fold>(th, a, b, c)
	{
	}
	
	virtual const char* TypeName() const override { return "Fold"; }
	
	void calc(int n, Z* out, Z* a, Z* b, Z* c, int aStride, int bStride, int cStride) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = sc_fold(*a, *b, *c);
			a += aStride;
			b += bStride;
			c += cStride;
		}
	}
};


struct IWrap : public ThreeInputUGen<IWrap>
{
	
	IWrap(Thread& th, Arg a, Arg b, Arg c) : ThreeInputUGen<IWrap>(th, a, b, c)
	{
	}
	
	virtual const char* TypeName() const override { return "IWrap"; }
	
	void calc(int n, Z* out, Z* a, Z* b, Z* c, int aStride, int bStride, int cStride) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = sc_iwrap(*a, *b, *c);
			a += aStride;
			b += bStride;
			c += cStride;
		}
	}
};

struct IFold : public ThreeInputUGen<IFold>
{
	
	IFold(Thread& th, Arg a, Arg b, Arg c) : ThreeInputUGen<IFold>(th, a, b, c)
	{
	}
	
	virtual const char* TypeName() const override { return "IFold"; }
	
	void calc(int n, Z* out, Z* a, Z* b, Z* c, int aStride, int bStride, int cStride) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = sc_ifold(*a, *b, *c);
			a += aStride;
			b += bStride;
			c += cStride;
		}
	}
};

static void clip_(Thread& th, Prim* prim)
{
	V hi   = th.popZIn("clip : hi");
	V lo   = th.popZIn("clip : lo");
	V in   = th.popZIn("clip : in");

	if (in.isReal() && lo.isReal() && hi.isReal()) {
		th.push(std::clamp(in.f, lo.f, hi.f));
	} else {
		th.push(new List(new Clip(th, in, lo, hi)));
	}
}

static void wrap_(Thread& th, Prim* prim)
{
	V hi   = th.popZIn("wrap : hi");
	V lo   = th.popZIn("wrap : lo");
	V in   = th.popZIn("wrap : in");

	if (in.isReal() && lo.isReal() && hi.isReal()) {
		th.push(sc_wrap(in.f, lo.f, hi.f));
	} else {
		th.push(new List(new Wrap(th, in, lo, hi)));
	}
}

static void fold_(Thread& th, Prim* prim)
{
	V hi   = th.popZIn("fold : hi");
	V lo   = th.popZIn("fold : lo");
	V in   = th.popZIn("fold : in");
	
	if (in.isReal() && lo.isReal() && hi.isReal()) {
		th.push(sc_fold(in.f, lo.f, hi.f));
	} else {
		th.push(new List(new Fold(th, in, lo, hi)));
	}
}

static void iwrap_(Thread& th, Prim* prim)
{
	V hi   = th.popZIn("iwrap : hi");
	V lo   = th.popZIn("iwrap : lo");
	V in   = th.popZIn("iwrap : in");

	if (in.isReal() && lo.isReal() && hi.isReal()) {
		th.push(sc_iwrap(in.f, lo.f, hi.f));
	} else {
		th.push(new List(new IWrap(th, in, lo, hi)));
	}
}

static void ifold_(Thread& th, Prim* prim)
{
	V hi   = th.popZIn("ifold : hi");
	V lo   = th.popZIn("ifold : lo");
	V in   = th.popZIn("ifold : in");
	
	if (in.isReal() && lo.isReal() && hi.isReal()) {
		th.push(sc_ifold(in.f, lo.f, hi.f));
	} else {
		th.push(new List(new IFold(th, in, lo, hi)));
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Carbon/Carbon.h>

struct MouseUGenGlobalState {
	float mouseX, mouseY;
	bool mouseButton;
} gMouseUGenGlobals;

static void* gstate_update_func(void* arg)
{
	MouseUGenGlobalState* gstate = &gMouseUGenGlobals;

	CGDirectDisplayID display = kCGDirectMainDisplay; // to grab the main display ID
	CGRect bounds = CGDisplayBounds(display);
	float rscreenWidth = 1. / bounds.size.width;
	float rscreenHeight = 1. / bounds.size.height;
	for (;;) {
		HIPoint point;
		HICoordinateSpace space = 2;
		HIGetMousePosition(space, nullptr, &point);

		gstate->mouseX = point.x * rscreenWidth; //(float)p.h * rscreenWidth;
		gstate->mouseY = 1. - point.y * rscreenHeight; //(float)p.v * rscreenHeight;
		gstate->mouseButton = GetCurrentButtonState();
		usleep(17000);
	}

	return 0;
}

Z gMouseLagTime = .1;
Z gMouseLagMul = log001 / gMouseLagTime;

struct MouseX : public TwoInputUGen<MouseX>
{
	Z _b1;
	Z _y1;
	bool _once;
	
	MouseX(Thread& th, Arg lo, Arg hi) : TwoInputUGen<MouseX>(th, lo, hi), _b1(1. + gMouseLagMul * th.rate.invSampleRate), _once(true)
	{
	}
	
	virtual const char* TypeName() const override { return "MouseX"; }
	
	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride) 
	{
		if (_once) {
			_once = false;
			_y1 = *lo + gMouseUGenGlobals.mouseX * (*hi - *lo);
		}
		Z y1 = _y1;
		Z b1 = _b1;
		for (int i = 0; i < n; ++i) {
			Z y0 = *lo + gMouseUGenGlobals.mouseX * (*hi - *lo);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};

struct MouseY : public TwoInputUGen<MouseY>
{
	Z _b1;
	Z _y1;
	bool _once;
	
	MouseY(Thread& th, Arg lo, Arg hi) : TwoInputUGen<MouseY>(th, lo, hi), _b1(1. + gMouseLagMul * th.rate.invSampleRate), _once(true)
	{
	}
	
	virtual const char* TypeName() const override { return "MouseY"; }
	
	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride) 
	{
		if (_once) {
			_once = false;
			_y1 = *lo + gMouseUGenGlobals.mouseY * (*hi - *lo);
		}
		Z y1 = _y1;
		Z b1 = _b1;
		for (int i = 0; i < n; ++i) {
			Z y0 = *lo + gMouseUGenGlobals.mouseY * (*hi - *lo);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};

struct ExpMouseX : public TwoInputUGen<ExpMouseX>
{
	Z _b1;
	Z _y1;
	bool _once;
	
	ExpMouseX(Thread& th, Arg lo, Arg hi) : TwoInputUGen<ExpMouseX>(th, lo, hi), _b1(1. + gMouseLagMul * th.rate.invSampleRate), _once(true)
	{
	}
	
	virtual const char* TypeName() const override { return "MouseX"; }
	
	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride) 
	{
		if (_once) {
			_once = false;
			_y1 = *lo * pow(*hi / *lo, gMouseUGenGlobals.mouseX);
		}
		Z y1 = _y1;
		Z b1 = _b1;
		for (int i = 0; i < n; ++i) {
			Z y0 = *lo * pow(*hi / *lo, gMouseUGenGlobals.mouseX);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};

struct ExpMouseY : public TwoInputUGen<ExpMouseY>
{
	Z _b1;
	Z _y1;
	bool _once;
	
	ExpMouseY(Thread& th, Arg lo, Arg hi) : TwoInputUGen<ExpMouseY>(th, lo, hi), _b1(1. + gMouseLagMul * th.rate.invSampleRate), _once(true)
	{
	}
	
	virtual const char* TypeName() const override { return "MouseY"; }
	
	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride) 
	{
		if (_once) {
			_once = false;
			_y1 = *lo * pow(*hi / *lo, gMouseUGenGlobals.mouseY);
		}
		Z y1 = _y1;
		Z b1 = _b1;
		for (int i = 0; i < n; ++i) {
			Z y0 = *lo * pow(*hi / *lo, gMouseUGenGlobals.mouseY);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};

static void mousex_(Thread& th, Prim* prim)
{
	V hi   = th.popZIn("mousex : hi");
	V lo   = th.popZIn("mousex : lo");
	
	th.push(new List(new MouseX(th, lo, hi)));
}

static void mousey_(Thread& th, Prim* prim)
{
	V hi   = th.popZIn("mousey : hi");
	V lo   = th.popZIn("mousey : lo");
	
	th.push(new List(new MouseY(th, lo, hi)));
}

static void xmousex_(Thread& th, Prim* prim)
{
	V hi   = th.popZIn("xmousex : hi");
	V lo   = th.popZIn("xmousex : lo");
	
	th.push(new List(new ExpMouseX(th, lo, hi)));
}

static void xmousey_(Thread& th, Prim* prim)
{
	V hi   = th.popZIn("xmousey : hi");
	V lo   = th.popZIn("xmousey : lo");
	
	th.push(new List(new ExpMouseY(th, lo, hi)));
}

static void mousex1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mousex1 : hi");
	Z lo   = th.popFloat("mousex1 : lo");
	
	Z z = lo + gMouseUGenGlobals.mouseX * (hi - lo);
	th.push(z);
}

static void mousey1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mousey1 : hi");
	Z lo   = th.popFloat("mousey1 : lo");
	
	Z z = lo + gMouseUGenGlobals.mouseY * (hi - lo);
	th.push(z);
}

static void xmousex1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmousex1 : hi");
	Z lo   = th.popFloat("xmousex1 : lo");
	
	Z z = lo * pow(hi / lo, gMouseUGenGlobals.mouseX);
	th.push(z);
}

static void xmousey1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmousey1 : hi");
	Z lo   = th.popFloat("xmousey1 : lo");
	
	Z z = lo * pow(hi / lo, gMouseUGenGlobals.mouseY);
	th.push(z);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define DEF(NAME, N, HELP) 	vm.def(#NAME, N, 1, NAME##_, HELP);
#define DEFMCX(NAME, N, HELP) 	vm.defmcx(#NAME, N, NAME##_, HELP);
#define DEFAM(NAME, MASK, HELP) 	vm.defautomap(#NAME, #MASK, NAME##_, HELP);

void AddFilterUGenOps();
void AddOscilUGenOps();
void AddDelayUGenOps();

void AddUGenOps()
{
	s_tempo = getsym("tempo");
	s_dt = getsym("dt");
	s_out = getsym("out");

	pthread_t mouseListenThread;
	pthread_create (&mouseListenThread, nullptr, gstate_update_func, (void*)0);

	vm.addBifHelp("\n*** unit generators ***");
	vm.defmcx("*+", 3, madd_, "(a b c --> out) multiply add. a b * c +");

    AddOscilUGenOps();
    AddFilterUGenOps();
    AddDelayUGenOps();
	
	vm.addBifHelp("\n*** plugs ***");

	vm.addBifHelp("\n*** control rate subgraphs ***");
	gK2A = automap("zk", 2, new Prim(k2a_, V(0.), 2, 1, "", ""), "", "");
	gK2AC = automap("zk", 2, new Prim(k2ac_, V(0.), 2, 1, "", ""), "", "");
	DEF(kr, 2, "(fun n --> out) evaluates fun with the current sample rate divided by n, then linearly upsamples all returned signals by n.")
	DEF(krc, 2, "(fun n --> out) evaluates fun with the current sample rate divided by n, then cubically upsamples all returned signals by n.")
	
	vm.addBifHelp("\n*** control function unit generators ***");
	DEFAM(imps, aaz, "(values durs rate --> out) single sample impulses.");
	DEFAM(steps, aaz, "(values durs rate --> out) steps");
	DEFAM(gates, aaaz, "(values durs holds rate --> out) gates");
	DEFAM(lines, aaz, "(values durs rate --> out) lines");
	DEFAM(xlines, aaz, "(values durs rate --> out) exponential lines");
	DEFAM(cubics, az, "(values rate --> out) cubic splines");
	DEFAM(curves, aaaz, "(values curvatures durs rate --> out) curves.");
	

	vm.addBifHelp("\n*** random control unit generators ***");
	DEFMCX(lfnoise0, 1, "(freq --> out) step noise source.");
	DEFMCX(lfnoise1, 1, "(freq --> out) ramp noise source.");
	DEFMCX(lfnoise3, 1, "(freq --> out) cubic spline noise source.");
	
	vm.addBifHelp("\n*** tempo unit generators ***");
	DEFAM(tempo, az, "([bps dur bps dur ...] rate --> out) returns a signal of tempo vs time given a list of interleaved tempos (in beats per second) and durations (in beats).");
	DEFAM(beats, z, "(tempo --> beats) integrates a tempo signal to produce a signal of the time in beats.");

	vm.addBifHelp("\n*** envelope unit generators ***");
	vm.addBifHelp("\nFor asr, adsr, dadsr, dahdsr envelopes, the arguments are as follows:");
	vm.addBifHelp("   delay - a time in seconds. a period of time before the attack segment where the amplitude is zero.");
	vm.addBifHelp("   attack - a time in seconds to rise from zero to the level specified by the amp argument.");
	vm.addBifHelp("   hold - a time in seconds to hold at the level specified by the amp argument.");
	vm.addBifHelp("   delay - a time in seconds to fall from amp to the sustain level.");
	vm.addBifHelp("   sustain - a level from zero to one which is multiplied by the amp argument. The envelope holds at this level until released.");
	vm.addBifHelp("   release - a time in seconds to fall from the current level to zero. A release begins whenever the beat time (the integral of tempo), exceeds dur.");
	vm.addBifHelp("   amp - an amplitude that scales the peak and sustain levels of the envelope.");
	vm.addBifHelp("   dur - a time in beats to release the envelope.");
	vm.addBifHelp("   tempo - a signal giving the tempo in beats per second versus time.");
	vm.addBifHelp("");

	DEFAM(adsr, akkz, "([attack decay sustain release] amp dur tempo --> envelope) an envelope generator.")
	DEFAM(dadsr, akkz, "([delay attack decay sustain release] amp dur tempo --> envelope) an envelope generator.")
	DEFAM(dahdsr, akkz, "([delay attack hold decay sustain release] amp dur tempo --> envelope) an envelope generator.")
	vm.addBifHelp("");
    
	DEFAM(endfade, zkkkk, "(in startupTime holdTime fadeTime threshold --> out) after startupTime has elapsed, fade out the sound when peak amplitude has dropped below threshold for more than the holdTime.");
	DEFAM(fadeout, zkk, "(in sustainTime fadeTime --> out) fadeout after sustain.");
	DEFAM(fadein, zk, "(in fadeTime --> out) fade in.");
	DEFAM(parenv, k, "(dur --> out) parabolic envelope. 1-x^2 for x from -1 to 1")
	DEFAM(quadenv, k, "(dur --> out) 4th order envelope. 1-x^4 for x from -1 to 1")
	DEFAM(octenv, k, "(dur --> out) 8th order envelope. 1-x^8 for x from -1 to 1")
	DEFAM(trienv, k, "(dur --> out) triangular envelope. 1-|x| for x from -1 to 1")
	DEFAM(tri2env, k, "(dur --> out) triangle squared envelope. (1-|x|)^2 for x from -1 to 1")
	DEFAM(trapezenv, k, "(dur --> out) trapezoidal envelope. (2 - |x-.5| - |x+.5|) for x from -1 to 1")
	DEFAM(trapez2env, k, "(dur --> out) trapezoid squared envelope. (2 - |x-.5| - |x+.5|)^2 for x from -1 to 1")

	DEFAM(cosenv, k, "(dur --> out) cosine envelope.")
	DEFAM(hanenv, k, "(dur --> out) hanning envelope.")
	DEFAM(han2env, k, "(dur --> out) hanning squared envelope.")
	DEFAM(gaussenv, kk, "(dur width --> out) gaussian envelope. exp(x^2/(-2*width^2)) for x from -1 to 1")

	DEFAM(tsig, zza, "(trig signal amp --> out) trigger a signal.")

	DEFAM(tparenv, zaa, "(trig dur amp --> out) triggered parabolic envelope. 1-x^2 for x from -1 to 1")
	DEFAM(tquadenv, zaa, "(trig dur amp --> out) triggered 4th order envelope. 1-x^4 for x from -1 to 1")
	DEFAM(toctenv, zaa, "(trig dur amp --> out) triggered 8th order envelope. 1-x^8 for x from -1 to 1")
	DEFAM(ttrienv, zaa, "(trig dur amp --> out) triggered triangular envelope. 1-|x| for x from -1 to 1")
	DEFAM(ttri2env, zaa, "(trig dur amp --> out) triggered triangle squared envelope. (1-|x|)^2 for x from -1 to 1")
	DEFAM(ttrapezenv, zaa, "(trig dur amp --> out) triggered trapezoidal envelope. (2 - |x-.5| - |x+.5|) for x from -1 to 1")
	DEFAM(ttrapez2env, zaa, "(trig dur amp --> out) triggered trapezoid squared envelope. (2 - |x-.5| - |x+.5|)^2 for x from -1 to 1")

	DEFAM(tcosenv, zaa, "(trig dur amp --> out) triggered cosine envelope.")
	DEFAM(thanenv, zaa, "(trig dur amp --> out) triggered hanning envelope.")
	DEFAM(than2env, zaa, "(trig dur amp --> out) triggered hanning squared envelope.")
	
	vm.addBifHelp("\n*** spawn unit generators ***");
	DEF(ola, 4, "(sounds hops rate numChannels --> out) overlap add. This is the basic operator for polyphony. ")

	vm.addBifHelp("\n*** pause unit generator ***");
	DEFMCX(pause, 2, "(in amp --> out) pauses the input when amp is <= 0, otherwise in is multiplied by amp.")

	vm.addBifHelp("\n*** panner unit generators ***");
	DEFAM(itd, zzk, "(in pan maxdelay --> out) interaural time delay.");
	DEFMCX(pan2, 2, "(in pos --> [left right]) stereo pan. pos 0 is center. pos -1 is full left, pos +1 is full right.")
	DEFMCX(rot2, 3, "(left right pos --> [left right]) stereo rotation. pos 0 is no rotation, +/-1 is 180 degrees, -.5 is -90 degrees, +.5 is +90 degrees.")
	DEFMCX(bal2, 3, "(left right pos --> [left right]) stereo balance control. pos 0 is center. pos -1 is full left, pos +1 is full right.")
	DEFMCX(fade2, 3, "(left right pos --> out) cross fade between two inputs. pos 0 is equal mix. pos -1 is all left, pos +1 is all right.")

	
	vm.addBifHelp("\n*** trigger unit generators ***");
	DEFMCX(tr, 1, "(in --> out) transitions from nonpositive to positive become single sample impulses.")
	DEFMCX(ntr, 1, "(in --> out) transitions from negative to nonnegative become single sample impulses.")
	DEFMCX(gate, 1, "(in hold --> out) outputs 1 for hold seconds after each trigger, else outputs zero.")
	DEFMCX(sah, 2, "(in trigger --> out) sample and hold")
	DEFAM(seq, az, "(in trigger --> out) pulls one value from the input for each trigger. output sustains at that level until the next trigger.")
	DEFAM(iseq, az, "(in trigger --> out) pulls one value from the input for each trigger. outputs that value for one sample. outputs zero when there is no trigger.")
	DEFMCX(pdiv, 3, "(in n istart --> out) pulse divider. outputs one impulse from the output for each n impulses in the input. istart is an offset. istart = 0 outputs a pulse on the first input pulse.")
	
	
	vm.addBifHelp("\n*** bounds unit generators ***");
	DEFMCX(clip, 3, "(in lo hi --> out) constrain the input to the bounds by clipping.")
	DEFMCX(wrap, 3, "(in lo hi --> out) constrain the input to the bounds by wrapping.")
	DEFMCX(fold, 3, "(in lo hi --> out) constrain the input to the bounds by folding at the edges.")
	DEFMCX(iwrap, 3, "(in lo hi --> out) constrain the input to the bounds by wrapping. all inputs treated as integers.")
	DEFMCX(ifold, 3, "(in lo hi --> out) constrain the input to the bounds by folding at the edges. all inputs treated as integers.")

	vm.addBifHelp("\n*** mouse control unit generators ***");
	DEFMCX(mousex, 2, "(lo hi --> out) returns a signal of the X coordinate of the mouse mapped to the linear range lo to hi.");
	DEFMCX(mousey, 2, "(lo hi --> out) returns a signal of the Y coordinate of the mouse mapped to the linear range lo to hi.");
	DEFMCX(xmousex, 2, "(lo hi --> out) returns a signal of the X coordinate of the mouse mapped to the exponential range lo to hi.");
	DEFMCX(xmousey, 2, "(lo hi --> out) returns a signal of the Y coordinate of the mouse mapped to the exponential range lo to hi.");

	DEFMCX(mousex1, 2, "(lo hi --> out) returns the current value of the X coordinate of the mouse mapped to the linear range lo to hi.");
	DEFMCX(mousey1, 2, "(lo hi --> out) returns the current value of the Y coordinate of the mouse mapped to the linear range lo to hi.");
	DEFMCX(xmousex1, 2, "(lo hi --> out) returns the current value of the X coordinate of the mouse mapped to the exponential range lo to hi.");
	DEFMCX(xmousey1, 2, "(lo hi --> out) returns the current value of the Y coordinate of the mouse mapped to the exponential range lo to hi.");	
}
