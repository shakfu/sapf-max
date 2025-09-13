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

#ifndef __MathFuns_h__
#define __MathFuns_h__

#include <cmath>
#include <stdint.h>
#include <stdio.h>
#include <algorithm>
#include "MathFuns.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////

const double kTwoPi = 2. * M_PI;
const double kDegToRad = M_PI / 180.;
const double kRadToDeg = 180. / M_PI;
const double kMinToSecs = 60.; // beats per minute to beats per second
const double kSecsToMin = 1. / 60.; // beats per second to beats per minute

const double log001 = std::log(0.001);
const double log01  = std::log(0.01);
const double log1   = std::log(0.1);
const double log2o2  = std::log(2.)/2.;
const double rlog2  = 1./std::log(2.);
const double sqrt2  = std::sqrt(2.);
const double rsqrt2 = 1. / sqrt2;

// used to truncate precision
const float truncFloat = (float)(3. * pow(2.0,22));
const double truncDouble = 3. * pow(2.0,51);


// lookup table implementations for sine, cosine and the A parameter for the RBJ biquads.

const int kSineTableSize = 16384; // this size gives more than 144 dB SNR (24 bit accuracy) when linear interpolated.
const int kSineTableSize4 = kSineTableSize >> 2;
const int kSineTableMask = kSineTableSize - 1;
extern double gSineTable[kSineTableSize+1];
const double gInvSineTableSize = 1. / kSineTableSize;
const double gSineTableOmega = kTwoPi * gInvSineTableSize;
const double gInvSineTableOmega = 1. / gSineTableOmega;

const int kDBAmpTableSize = 1500;
extern double gDBAmpTable[kDBAmpTableSize+2];
const double kDBAmpScale = 5.;
const double kInvDBAmpScale = .2;
const double kDBAmpOffset = 150.;

const int kDecayTableSize = 2000;
const double kDecayScale = 1000.;
extern double gDecayTable[kDecayTableSize+1];

const int kFirstOrderCoeffTableSize = 1000;
const double kInvFirstOrderCoeffTableSize = 1. / kFirstOrderCoeffTableSize;
const double kFirstOrderCoeffScale = 1000.;
extern double gFirstOrderCoeffTable[kFirstOrderCoeffTableSize+1];

void fillSineTable();
void fillDBAmpTable();
void fillDecayTable();
void fillFirstOrderCoeffTable();
void fillOddHilbert(int n, double* h);

inline double lut(double* table, int index, double frac)
{
	double a = table[index];
	double b = table[index + 1];
	return a + frac * (b - a);
}

inline double oscilLUT(double* table, int index, int mask, double x)
{
	double y0 = table[(index - 1) & mask];
	double y1 = table[(index    ) & mask];
	double y2 = table[(index + 1) & mask];
	double y3 = table[(index + 2) & mask];

    double c0 = y1;
    double c1 = 0.5f * (y2 - y0);
    double c2 = y0 - 2.5f * y1 + 2. * y2 - 0.5 * y3;
    double c3 = 1.5 * (y1 - y2) + 0.5 * (y3 - y0);
    
    return ((c3 * x + c2) * x + c1) * x + c0;
}

inline double oscilLUT2(double* tableA, double* tableB, int index, int mask, double x, double frac)
{
	double x2 = x*x;
	double x3 = x*x2;
	double x3b = 1.5 * x3;
	
    double c0 = x2 - .5 * (x + x3);
    double c1 = 1. - 2.5*x2 + x3b;
    double c2 = .5 * x + 2. * x2 - x3b;
    double c3 = .5 * (x3 - x2);
	
	double a, b;
	{
		double y0 = tableA[(index - 1) & mask];
		double y1 = tableA[(index    ) & mask];
		double y2 = tableA[(index + 1) & mask];
		double y3 = tableA[(index + 2) & mask];
		a = c0*y0 + c1*y1 + c2*y2 + c3*y3;
	}
	{
		double y0 = tableB[(index - 1) & mask];
		double y1 = tableB[(index    ) & mask];
		double y2 = tableB[(index + 1) & mask];
		double y3 = tableB[(index + 2) & mask];
		b = c0*y0 + c1*y1 + c2*y2 + c3*y3;
	}
	
	return a + frac * (b - a);
}


inline double calcDecay(double ratio)
{
	if (ratio < 0.) {
		ratio = -ratio;
		ratio = std::clamp(ratio, 0., 2.) ;

		double findex = kDecayScale * ratio;
		double iindex = floor(findex);
		return -lut(gDecayTable, (int)iindex, findex - iindex);
	} else {
		ratio = std::clamp(ratio, 0., 2.) ;

		double findex = kDecayScale * ratio;
		double iindex = floor(findex);
		return lut(gDecayTable, (int)iindex, findex - iindex);
	}
}

inline double t_dbamp(double dbgain)
{
	dbgain = std::clamp(dbgain, -150., 150.) ;
	double findex = kDBAmpScale * (dbgain + kDBAmpOffset);
	double iindex = floor(findex);
	return lut(gDBAmpTable, (int)iindex, findex - iindex);
}

inline double t_firstOrderCoeff(double freq)
{
	freq = std::clamp(freq, 0., kFirstOrderCoeffScale) ;

	double findex = freq;
	double iindex = floor(findex);
	return lut(gFirstOrderCoeffTable, (int)iindex, findex - iindex);
}

inline double tsin(double x)
{
	double findex = gInvSineTableOmega * x;
	double iindex = floor(findex);
	int index = (int)iindex & kSineTableMask;
	return lut(gSineTable, index, findex - iindex);
}

inline double tcos(double x)
{
	return tsin(x + M_PI_2);
}

inline double tsin1(double x) // full cycle every multiple of 1.0 instead of every multiple of two pi. 
{
	double findex = gInvSineTableSize * x;
	double iindex = floor(findex);
	int index = (int)iindex & kSineTableMask;
	return lut(gSineTable, index, findex - iindex);
}

inline double tsinx(double x) // full cycle every multiple of 1.0 instead of every multiple of two pi. 
{
	double iindex = floor(x);
	int index = (int)iindex & kSineTableMask;
	return lut(gSineTable, index, x - iindex);
}

inline double tcos1(double x)
{
	return tsin1(x + .25);
}


inline void tsincos(double x, double& sn, double& cs)
{
	double findex = gInvSineTableOmega * x;
	double iindex = floor(findex);
	double frac = findex - iindex;
	int index = (int)iindex & kSineTableMask;
	{
		sn = lut(gSineTable, index, frac);
	}
	{
		index = (index + kSineTableSize4) & kSineTableMask;
		cs = lut(gSineTable, index, frac);
	}
}


inline void tsincosx(double x, double& sn, double& cs)
{
	double iindex = floor(x);
	//double iindex = (x + truncDouble) - truncDouble;
	double frac = x - iindex;
	int index = (int)iindex & kSineTableMask;
	{
		//sn = lut(gSineTable, index, frac);
		sn = oscilLUT(gSineTable, index, kSineTableMask, frac);
	}
	{
		index = (index + kSineTableSize4) & kSineTableMask;
		//cs = lut(gSineTable, index, frac);
		cs = oscilLUT(gSineTable, index, kSineTableMask, frac);
	}
}


inline void tsincos1(double x, double& sn, double& cs)
{
	double findex = gInvSineTableSize * x;
	double iindex = floor(findex);
	double frac = findex - iindex;
	int index = (int)iindex & kSineTableMask;
	{
		sn = lut(gSineTable, index, frac);
	}
	{
		index = (index + kSineTableSize4) & kSineTableMask;
		cs = lut(gSineTable, index, frac);
	}
}

inline double cubicInterpolate(double x, double y0, double y1, double y2, double y3)
{
    // 4-point, 3rd-order Hermite (x-form)
    double c0 = y1;
    double c1 = 0.5 * (y2 - y0);
    double c2 = y0 - 2.5 * y1 + 2. * y2 - 0.5 * y3;
    double c3 = 1.5 * (y1 - y2) + 0.5 * (y3 - y0);
    
    return ((c3 * x + c2) * x + c1) * x + c0;
}


inline double lagrangeInterpolate(double x, double y0, double y1, double y2, double y3)
{
    // 4-point, 3rd-order Lagrange (x-form)
    double c0 = y1;
    double c1 = y2 - 1/3. * y0 - .5 * y1 - 1/6. * y3;
    double c2 = .5 * (y0 + y2) - y1;
    double c3 = 1/6. * (y3 - y0) + .5 * (y1 - y2);
    return ((c3 * x + c2) * x + c1) * x + c0;
}

inline double cubicInterpolate2(double x, double y0, double y1, double y2, double y3)
{
    // 4-point, 3rd-order Hermite (x-form)
	double x2 = x*x;
	double x3 = x*x2;
	double x3b = 1.5 * x3;
    double c0 = x2 - .5 * (x + x3);
    double c1 = 1. - 2.5*x2 + x3b;
    double c2 = .5 * x + 2. * x2 - x3b;
    double c3 = .5 * (x3 - x2);
    
    return c0*y0 + c1*y1 + c2*y2 + c3*y3;
}

inline double CalcFeedback(double delaytime, double decaytime)
{
	if (delaytime == 0.) {
		return 0.;
	} else if (decaytime > 0.) {
		return exp(log001 * delaytime / decaytime);
	} else if (decaytime < 0.) {
		return -exp(log001 * delaytime / -decaytime);
	} else {
		return 0.;
	}
}

inline double CalcDecayRate(double decaytime, double sampleRate)
{
	if (decaytime == 0.) {
		return 0.;
	} else if (decaytime > 0.) {
		return exp(log001 / (decaytime * sampleRate));
	} else if (decaytime < 0.) {
		return -exp(log001 / (-decaytime * sampleRate));
	} else {
		return 0.;
	}
}


inline bool IsInteger(double x)
{
	return floor(x) == x;
}

inline double sc_squared(double a) { return a * a; }
inline double sc_cubed(double a) { return a * a * a; }
inline double sc_fourth(double a) { double a2 = a * a; return a2 * a2; }
inline double sc_fifth(double a) { double a2 = a * a; return a2 * a2 * a; }
inline double sc_sixth(double a) { double a3 = a * a * a; return a3 * a3; }
inline double sc_seventh(double a) { double a3 = a * a * a; return a3 * a3 * a; }
inline double sc_eighth(double a) { double a2 = a * a; double a4 = a2 * a2; return a4 * a4; }
inline double sc_ninth(double a) { double a3 = a * a * a; return a3 * a3 * a3; }

inline double sc_pow(double a, double b) 
{
	if (a < 0.) { // pow can't normally handle negatives.
		if (b == floor(b)) { // exponent is an integer
			double b2 = b * .5;
			if (b2 == floor(b2)) { // is even
				return pow(-a, b);
			} else {	// is odd
				return -pow(-a, b);
			}
		} else {
			return -pow(-a, b); // do something besides returning NaN.
		}
	} else {
		return pow(a, b);
	}
}

inline double sc_log(double a) { return log(fabs(a)); }
inline double sc_log2(double a) { return log2(fabs(a)); }
inline double sc_log10(double a) { return log10(fabs(a)); }

inline double sc_sinc(double a) { return a == 0. ? 1. : sin(a) / a; }

inline double sc_centshz(double cents)
{
	return 440. * exp2((cents - 6900.) * 0.0008333333333333333333333333);
}

inline double sc_hzcents(double freq)
{
	return sc_log2(freq * 0.002272727272727272727272727) * 1200. + 6900.;  // 0.0022727 = 1/440
}

inline double sc_centsratio(double cents)
{
	return exp2(cents * 0.00083333333333);
}

inline double sc_ratiocents(double ratio)
{
	return 1200. * sc_log2(ratio);
}


inline double sc_keyhz(double note)
{
	return 440. * exp2((note - 9.) * 0.08333333333333333333333333);
}

inline double sc_hzkey(double freq)
{
	return sc_log2(freq * 0.002272727272727272727272727) * 12. + 9.;  // 0.0022727 = 1/440
}

inline double sc_nnhz(double note)
{
	return 440. * exp2((note - 69.) * 0.08333333333333333333333333);
}

inline double sc_hznn(double freq)
{
	return sc_log2(freq * 0.002272727272727272727272727) * 12. + 69.;  // 0.0022727 = 1/440
}

inline double sc_moctratio(double moct)
{
	return exp2(moct * .001);
}


inline double sc_semiratio(double key)
{
	return exp2(key * 0.08333333333333333333333333);
}

inline double sc_ratiosemi(double ratio)
{
	return 0.08333333333333333333333333 * sc_log2(ratio);
}

inline double sc_keyratio(double key, double invedo)
{
	return exp2(key * invedo);
}

inline double sc_ratiokey(double ratio, double edo)
{
	return edo * sc_log2(ratio);
}

inline double sc_octhz(double note)
{
	return 440. * exp2(note - .75);
}

inline double sc_mocthz(double note)
{
	return 440. * exp2(note * .001 - .75);
}

inline double sc_hzoct(double freq)
{
	return sc_log2(freq * 0.002272727272727272727272727) + .75;  // 0.0022727 = 1/440
}

inline double sc_hzmoct(double freq)
{
	return sc_log2(freq * 0.002272727272727272727272727) * 1000. + 750;  // 0.0022727 = 1/440
}

inline double sc_ampdb(double amp)
{
	return sc_log10(amp) * 20.;
}

inline double sc_dbamp(double db)
{
	return pow(10., db * .05);
}

inline int64_t sc_isgn(int64_t x)
{
	if (x < 0) return -1;
	if (x > 0) return 1;
	return 0;
}

inline double sc_sgn(double x)
{
	if (x < 0.) return -1.;
	if (x > 0.) return 1.;
	return 0.;
}

inline double sc_sgn(int x)
{
	if (x < 0) return -1;
	if (x > 0) return 1;
	return 0;
}

inline double sc_possgn(double x)
{
	if (x < 0.) return -1.;
	if (x >= 0.) return 1.;
	return 0.;
}

inline double sc_ramp(double x)
{
	if (x <= 0.) return 0.;
	if (x >= 1.) return 1.;
	return x;
}

inline double sc_distort(double x)
{
	return x / (1. + fabs(x));
}

inline double sc_softclip(double x)
{
	double absx = fabs(x);
	if (absx <= 0.5) return x;
	else return (absx - 0.25) / x;
}

inline double sc_tanh_approx(double x)
{
	if (x < -3.) return -1.;
	if (x > 3.) return 1.;
	double x2 = x*x;
	return x * (27. + x2)/(27. + 9*x2);
}

inline double sc_sqrt(double x)
{
	return x < 0. ? -sqrt(-x) : sqrt(x);
}

inline double sc_hanWindow(double x)
{
	if (x < 0. || x > 1.) return 0.;
	return 0.5 - 0.5 * cos(x * kTwoPi);
}

inline double sc_sinWindow(double x)
{
	if (x < 0. || x > 1.) return 0.;
	return sin(x * M_PI);
}

inline double sc_triWindow(double x)
{
	if (x < 0. || x > 1.) return 0.;
	if (x < 0.5) return 2. * x;
	else return -2. * x + 2.;
}

inline double sc_bitriWindow(double x)
{
	double ax = fabs(x);
	if (ax > 1.) return 0.;
	return 1. - ax;
}

inline double sc_rectWindow(double x)
{
	if (x < 0. || x > 1.) return 0.;
	return 1.;
}

inline double sc_scurve(double x)
{
	if (x <= 0.) return 0.;
	if (x >= 1.) return 1.;
	return (x * x) * (3. - 2. * x);
}

inline double sc_scurve0(double x)
{
	// assumes that x is in range
	return (x * x) * (3. - 2. * x);
}

inline double zapgremlins(double x)
{
	double absx = fabs(x);
	// very small numbers fail the first test, eliminating denormalized numbers
	//    (zero also fails the first test, but that is OK since it returns zero.)
	// very large numbers fail the second test, eliminating infinities
	// Not-a-Numbers fail both tests and are eliminated.
	return (absx > 1e-15 && absx < 1e15) ? x : 0.;
}

inline int64_t sc_div(int64_t a, int64_t b) 
{
	int64_t c;
	if (b) {
		if (a<0) c = (a+1)/b - 1;
		else c = a/b;
	} else c = a;
	return c;
}

inline int64_t sc_divz(int64_t a, int64_t b, int64_t z) 
{
	int64_t c;
	if (b) {
		if (a<0) c = (a+1)/b - 1;
		else c = a/b;
	} else c = z;
	return c;
}

inline double sc_fmod(double in, double hi) 
{
	double out = fmod(in, hi);
	if (out < 0.) out = hi + out;
	return out;
}

inline void sc_fdivmod(double in, double hi, double& outDiv, double& outMod)
{
	outMod = sc_fmod(in, hi);
	outDiv = (in - outMod) / hi;
}

inline int64_t sc_imod(int64_t in, int64_t hi) 
{ 
	// avoid the divide if possible
	const int64_t lo = 0;
	if (in >= hi) {
		in -= hi;
		if (in < hi) return in;
	} else if (in < lo) {
		in += hi;
		if (in >= lo) return in;
	} else return in;
	
	if (hi == lo) return lo;
	
	int64_t c = in % hi;
	if (c<0) c += hi;
	return c;
}

inline int64_t sc_gcd(int64_t u, int64_t v) 
{
	int64_t t;
	u = std::abs(u);
	v = std::abs(v);
	if (u <= 1 || v <= 1) return 1;
	while (u>0) {
		if (u<v) { t=u; u=v; v=t; }
		u = u % v;
	}
	return v;
}

inline int64_t sc_lcm(int64_t u, int64_t v) 
{
	return u/sc_gcd(u,v) * v;
}

inline double sc_fold(double in, double lo, double hi) 
{
	double x, c, range, range2;
	x = in - lo;
	
	// avoid the divide if possible
	if (in >= hi) {
		in = hi + hi - in;
		if (in >= lo) return in;
	} else if (in < lo) {
		in = lo + lo - in;
		if (in < hi) return in;
	} else return in;
	
	if (hi == lo) return lo;
	// ok do the divide
	range = hi - lo;
	range2 = range + range;
	c = x - range2 * floor(x / range2);
	if (c>=range) c = range2 - c;
	return c + lo;
}

inline double sc_wrap(double in, double lo, double hi) 
{
	double range;
	// avoid the divide if possible
	if (in >= hi) {
		range = hi - lo;
		in -= range;
		if (in < hi) return in;
	} else if (in < lo) {
		range = hi - lo;
		in += range;
		if (in >= lo) return in;
	} else return in;
	
	if (hi == lo) return lo;
	return in - range * floor((in - lo)/range); 
}



inline int64_t sc_iwrap(int64_t in, int64_t lo, int64_t hi)
{
	return sc_imod(in - lo, hi - lo + 1) + lo;
}

inline int64_t sc_ifold(int64_t in, int64_t lo, int64_t hi)
{
	int64_t b = hi - lo;
	int64_t b2 = b+b;
	int64_t c = sc_imod(in - lo, b2);
	if (c>b) c = b2-c;
	return c + lo;
}


inline double sc_clip2(double a, double b)
{
	return std::clamp(a, -b, b);
}

inline double sc_wrap2(double a, double b)
{
	return sc_wrap(a, -b, b);
}

inline double sc_fold2(double a, double b)
{
	return sc_fold(a, -b, b);
}

inline int sc_excess(int a, int b)
{
	return a - std::clamp(a, -b, b);
}

inline double sc_round(double x, double quant)
{
	return quant==0. ? x : floor(x/quant + .5) * quant;
}

inline double sc_roundUp(double x, double quant)
{
	return quant==0. ? x : ceil(x/quant) * quant;
}

inline double sc_trunc(double x, double quant)
{
	return quant==0. ? x : floor(x/quant) * quant;
}


inline double sc_cmp(double a, double b)
{
	return a < b ? -1. : (a > b ? 1. : 0.);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////



#endif
