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

#ifndef __MultichannelExpansion_h__
#define __MultichannelExpansion_h__

#include "VM.hpp"

Prim* mcx(int n, Arg f, const char* name, const char* help);
Prim* automap(const char* mask, int n, Arg f, const char* inName, const char* inHelp);
List* handleEachOps(Thread& th, int numArgs, Arg fun);
void flop_(Thread& th, Prim* prim);
void flops_(Thread& th, Prim* prim);
void flop1_(Thread& th, Prim* prim);
void lace_(Thread& th, Prim* prim);
void sel_(Thread& th, Prim* prim);
void sell_(Thread& th, Prim* prim);

#endif

