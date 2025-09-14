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

#include "RCObj.hpp"
#include "VM.hpp"


RCObj::RCObj()
	: refcount(0)
{
#if COLLECT_MINFO
	++vm.totalObjectsAllocated;
#endif
}

RCObj::RCObj(RCObj const& that)
	: refcount(0)
{
#if COLLECT_MINFO
	++vm.totalObjectsAllocated;
#endif
}

RCObj::~RCObj()
{
#if COLLECT_MINFO
	++vm.totalObjectsFreed;
#endif
}


void RCObj::norefs()
{
	refcount = -999;
	delete this; 
}

	
void RCObj::negrefcount()
{
	post("RELEASING WITH NEGATIVE REFCOUNT %s %p %d\n", TypeName(), this, refcount.load());
}
void RCObj::alreadyDead()
{
	post("RETAINING ALREADY DEAD OBJECT %s %p\n", TypeName(), this);
}
