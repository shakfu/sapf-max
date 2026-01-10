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

#ifndef __no_web2__RCObj__
#define __no_web2__RCObj__

#include <stdint.h>
#include <atomic>
#include "rc_ptr.hpp"

class RCObj
{
public:
	mutable std::atomic<int32_t> refcount;

public:
	RCObj();
    RCObj(RCObj const&);
	virtual ~RCObj();
	
	void retain() const;
	void release();
	virtual void norefs();
		
	int32_t getRefcount() const { return refcount; }
	
	void negrefcount();
	void alreadyDead();
	

	virtual const char* TypeName() const = 0;
};

inline void retain(RCObj* o) { o->retain(); }
inline void release(RCObj* o) { o->release(); }

#endif /* defined(__no_web2__RCObj__) */
