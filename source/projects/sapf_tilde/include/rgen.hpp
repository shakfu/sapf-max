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

#ifndef __rgen_h__
#define __rgen_h__

#include <stdint.h>
#include <cmath>
#include <algorithm>
#include "Hash.hpp"

inline uint64_t xorshift64star(uint64_t x) {
	x ^= x >> 12; // a
	x ^= x << 25; // b
	x ^= x >> 27; // c
	return x * UINT64_C(2685821657736338717);
}

inline uint64_t xorshift128plus(uint64_t s[2]) {
	uint64_t x = s[0];
	uint64_t const y = s[1];
	s[0] = y;
	x ^= x << 23; // a
	x ^= x >> 17; // b
	x ^= y ^ (y >> 26); // c
	s[1] = x;
	return x + y;
}

inline uint64_t rotl(const uint64_t x, int k) {
	return (x << k) | (x >> (64 - k));
}

inline uint64_t xoroshiro128(uint64_t s[2]) {
	const uint64_t s0 = s[0];
	uint64_t s1 = s[1];
	const uint64_t result = s0 + s1;

	s1 ^= s0;
	s[0] = rotl(s0, 55) ^ s1 ^ (s1 << 14); // a, b
	s[1] = rotl(s1, 36); // c

	return result;
}

inline double itof1(uint64_t i)
{
	union { uint64_t i; double f; } u;
	u.i = 0x3FF0000000000000LL | (i >> 12);
	return u.f - 1.;
}

const double kScaleR63 = pow(2.,-63);
const double kScaleR31 = pow(2.,-31);

inline double itof2(uint64_t i, double a)
{
	return (double)i * a * kScaleR63 - a;
}

inline double itof2(uint32_t i, double a)
{
	return (double)i * a * kScaleR31 - a;
}

struct RGen
{
	uint64_t s[2];
	
	void init(int64_t seed);
	int64_t trand();
	    
	double drand(); // 0 .. 1
	double drand2(); // -1 .. 1
	double drand8(); // -1/8 .. 1/8
	double drand16(); // -1/16 .. 1/16

	double rand(double lo, double hi);
	double xrand(double lo, double hi);
	double linrand(double lo, double hi);
	double trirand(double lo, double hi);
	double coin(double p);
	
	int64_t irand(int64_t lo, int64_t hi);
	int64_t irand0(int64_t n);
	int64_t irand2(int64_t scale);
	int64_t ilinrand(int64_t lo, int64_t hi);
	int64_t itrirand(int64_t lo, int64_t hi);
	
};


inline void RGen::init(int64_t seed)
{
	s[0] = Hash64(seed + 0x43a68b0d0492ba51LL);
	s[1] = Hash64(seed + 0x56e376c6e7c29504LL);
}

inline int64_t RGen::trand()
{
	return (int64_t)xoroshiro128(s);
}

inline double RGen::drand()
{
	union { uint64_t i; double f; } u;
	u.i = 0x3FF0000000000000LL | ((uint64_t)trand() >> 12);
	return u.f - 1.;
}

inline double RGen::drand2()
{
	union { uint64_t i; double f; } u;
	u.i = 0x4000000000000000LL | ((uint64_t)trand() >> 12);
	return u.f - 3.;
}

inline double RGen::drand8()
{
	union { uint64_t i; double f; } u;
	u.i = 0x3FD0000000000000LL | ((uint64_t)trand() >> 12);
	return u.f - .375;
}

inline double RGen::drand16()
{
	union { uint64_t i; double f; } u;
	u.i = 0x3FC0000000000000LL | ((uint64_t)trand() >> 12);
	return u.f - .1875;
}

inline double RGen::rand(double lo, double hi)
{
	return lo + (hi - lo) * drand();
}

inline double RGen::xrand(double lo, double hi)
{
	return lo * pow(hi / lo,  drand());
}

inline double RGen::linrand(double lo, double hi)
{
	return lo + (hi - lo) * std::min(drand(), drand());
}

inline double RGen::trirand(double lo, double hi)
{
	return lo + (hi - lo) * (.5 + .5 * (drand() - drand()));
}

inline double RGen::coin(double p)
{
	return drand() < p ? 1. : 0.;
}

inline int64_t RGen::irand0(int64_t n)
{
	return (int64_t)floor(n * drand());
}

inline int64_t RGen::irand(int64_t lo, int64_t hi)
{
	return lo + (int64_t)floor((hi - lo + 1) * drand());
}

inline int64_t RGen::irand2(int64_t scale)
{
	double fscale = (double)scale;
	return (int64_t)floor((2. * fscale + 1.) * drand() - fscale);
}

inline int64_t RGen::ilinrand(int64_t lo, int64_t hi)
{
	return lo + (int64_t)floor((hi - lo) * std::min(drand(), drand()));
}

inline int64_t RGen::itrirand(int64_t lo, int64_t hi)
{
	double scale = (double)(hi - lo);
	return lo + (int64_t)floor(scale * (.5 + .5 * (drand() - drand())));
}

#endif

