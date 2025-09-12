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

/*

count leading zeroes function and those that can be derived from it

*/

// TODO FIXME Replace with C++20's <bit>

#ifndef _CLZ_
#define _CLZ_

#include <TargetConditionals.h>


static int32_t CLZ( int32_t arg )
{
	if (arg == 0) return 32;
	return __builtin_clz(arg);
}


static int64_t CLZ( int64_t arg )
{
	if (arg == 0) return 64;
	return __builtin_clzll(arg);
}



// count trailing zeroes
inline int32_t CTZ(int32_t x) 
{
	return 32 - CLZ(~x & (x-1));
}

// count leading ones
inline int32_t CLO(int32_t x) 
{
	return CLZ(~x);
}

// count trailing ones
inline int32_t CTO(int32_t x) 
{
	return 32 - CLZ(x & (~x-1));
}

// number of bits required to represent x. 
inline int32_t NUMBITS(int32_t x) 
{
	return 32 - CLZ(x);
}

// log2 of the next power of two greater than or equal to x. 
inline int32_t LOG2CEIL(int32_t x) 
{
	return 32 - CLZ(x - 1);
}

// log2 of the next power of two greater than or equal to x. 
inline int64_t LOG2CEIL(int64_t x)
{
	return 64 - CLZ(x - 1);
}

// next power of two greater than or equal to x
inline int32_t NEXTPOWEROFTWO(int32_t x)
{
	return int32_t(1) << LOG2CEIL(x);
}

// next power of two greater than or equal to x
inline int64_t NEXTPOWEROFTWO(int64_t x)
{
	return int64_t(1) << LOG2CEIL(x);
}

// is x a power of two
inline bool ISPOWEROFTWO(int32_t x) 
{
	return (x & (x-1)) == 0;
}

inline bool ISPOWEROFTWO64(int64_t x) 
{
	return (x & (x-1)) == 0;
}

// input a series of counting integers, outputs a series of gray codes .
inline int32_t GRAYCODE(int32_t x)
{
	return x ^ (x>>1);
}

// find least significant bit
inline int32_t LSBit(int32_t x)
{
	return x & -x;
}

// find least significant bit position
inline int32_t LSBitPos(int32_t x)
{
	return CTZ(x & -x);
}

// find most significant bit position
inline int32_t MSBitPos(int32_t x)
{
	return 31 - CLZ(x);
}

// find most significant bit
inline int32_t MSBit(int32_t x)
{
	return int32_t(1) << MSBitPos(x);
}

// count number of one bits
inline uint32_t ONES(uint32_t x) 
{
	uint32_t t; 
	x = x - ((x >> 1) & 0x55555555); 
	t = ((x >> 2) & 0x33333333); 
	x = (x & 0x33333333) + t; 
	x = (x + (x >> 4)) & 0x0F0F0F0F; 
	x = x + (x << 8); 
	x = x + (x << 16); 
	return x >> 24; 
}

// count number of zero bits
inline uint32_t ZEROES(uint32_t x)
{
	return ONES(~x);
}


// reverse bits in a word
inline uint32_t BitReverse(uint32_t x)
{
  x = ((x & 0xAAAAAAAA) >>  1) | ((x & 0x55555555) <<  1);
  x = ((x & 0xCCCCCCCC) >>  2) | ((x & 0x33333333) <<  2);
  x = ((x & 0xF0F0F0F0) >>  4) | ((x & 0x0F0F0F0F) <<  4);
  x = ((x & 0xFF00FF00) >>  8) | ((x & 0x00FF00FF) <<  8);
  return (x >> 16) | (x << 16);
}

// barrel shifts
inline uint64_t RotateRight (int64_t ix, int64_t s)
{
	uint64_t x = ix;
	s = s & 63;
	return (x << (64-s)) | (x >> s);
}
   
inline uint64_t RotateLeft (int64_t ix, int64_t s)
{
	uint64_t x = ix;
	s = s & 63;
	return (x >> (64-s)) | (x << s);
}

inline uint32_t RotateRight (int32_t ix, int32_t s)
{
	uint32_t x = ix;
	s = s & 31;
	return (x << (32-s)) | (x >> s);
}
   
inline uint32_t RotateLeft (int32_t ix, int32_t s)
{
	uint32_t x = ix;
	s = s & 31;
	return (x >> (32-s)) | (x << s);
}

inline uint8_t RotateRight (int8_t ix, int8_t s)
{
	uint8_t x = ix;
	s = s & 7;
	return (x << (8-s)) | (x >> s);
}
   
inline uint8_t RotateLeft (int8_t ix, int8_t s)
{
	uint8_t x = ix;
	s = s & 7;
	return (x >> (8-s)) | (x << s);
}

#endif

