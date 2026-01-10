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

#ifndef _Hash_
#define _Hash_

#include <sys/types.h>


// hash function for a string
inline int32_t Hash(const char *inKey)
{
    // the one-at-a-time hash.
    // a very good hash function. ref: a web page by Bob Jenkins.
    // http://www.burtleburtle.net/bob/hash/doobs.html
    int32_t hash = 0;
    while (*inKey) {
        hash += *inKey++;
        hash += hash << 10;
        hash ^= hash >> 6;
    }
    hash += hash << 3;
    hash ^= hash >> 11;
    hash += hash << 15;
    return hash;
}

// hash function for a string that also returns the length
inline int32_t Hash(const char *inKey, size_t *outLength)
{
    // the one-at-a-time hash.
    // a very good hash function. ref: a web page by Bob Jenkins.
    const char *origKey = inKey;
    int32_t hash = 0;
    while (*inKey) {
        hash += *inKey++;
        hash += hash << 10;
        hash ^= hash >> 6;
    }
    hash += hash << 3;
    hash ^= hash >> 11;
    hash += hash << 15;
    *outLength = inKey - origKey;
    return hash;
}

// hash function for an array of char 
inline int32_t Hash(const char *inKey, size_t inLength)
{
    // the one-at-a-time hash.
    // a very good hash function. ref: a web page by Bob Jenkins.
    int32_t hash = 0;
    for (size_t i=0; i<inLength; ++i) {
        hash += *inKey++;
        hash += hash << 10;
        hash ^= hash >> 6;
    }
    hash += hash << 3;
    hash ^= hash >> 11;
    hash += hash << 15;
    return hash;
}

// hash function for integers
inline int32_t Hash(int32_t inKey)
{
    // Thomas Wang's integer hash.
    // http://www.concentric.net/~Ttwang/tech/inthash.htm
    // a faster hash for integers. also very good.
    uint32_t hash = (uint32_t)inKey;
    hash += ~(hash << 15);
    hash ^=   hash >> 10;
    hash +=   hash << 3;
    hash ^=   hash >> 6;
    hash += ~(hash << 11);
    hash ^=   hash >> 16;
    return (int32_t)hash;
}

inline int64_t Hash64(int64_t inKey)
{
    // Thomas Wang's 64 bit integer hash.
	uint64_t hash = (uint64_t)inKey;
	hash ^= ((~hash) >> 31);
	hash += (hash << 28);
	hash ^= (hash >> 21);
	hash += (hash << 3);
	hash ^= ((~hash) >> 5);
	hash += (hash << 13);
	hash ^= (hash >> 27);
	hash += (hash << 32);
	return (int64_t)hash;
}

inline int64_t Hash64bad(int64_t inKey)
{
    // Thomas Wang's 64 bit integer hash.
	uint64_t hash = (uint64_t)inKey;
	hash = (~hash) + (hash << 21); // hash = (hash << 21) - hash - 1;
	hash ^= (hash >> 24);
	hash += (hash << 3) + (hash << 8); // hash * 265
	hash ^= (hash >> 14);
	hash += (hash << 2) + (hash << 4); // hash * 21
	hash ^= (hash >> 28);
	hash += (hash << 31);
	return (int64_t)hash;
}

inline int32_t Hash(const int32_t *inKey, int32_t inLength)
{
    // one-at-a-time hashing of a string of int32_t's.
    // uses Thomas Wang's integer hash for the combining step.
    int32_t hash = 0;
    for (int i=0; i<inLength; ++i) {
        hash = Hash(hash + *inKey++);
    }
    return hash;
}

#ifndef _LASTCHAR_
#define _LASTCHAR_
#if BYTE_ORDER == LITTLE_ENDIAN
const int32_t kLASTCHAR = 0xFF000000;
#else
const int32_t kLASTCHAR = 0x000000FF;
#endif
#endif

inline int32_t Hash(const int32_t *inKey)
{
    // hashing of a string of int32_t's.
    // uses Thomas Wang's integer hash for the combining step.
	int32_t hash = 0;
    int32_t c;
	do {
        c = *inKey++;
		hash = Hash(hash + c);
	} while (c & kLASTCHAR);
    return hash;
}

#endif

