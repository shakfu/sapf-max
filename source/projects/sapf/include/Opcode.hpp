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

#ifndef __Opcode_h__
#define __Opcode_h__

#include "VM.hpp"

enum {
	BAD_OPCODE,
	opNone,
	opPushImmediate,
	opPushLocalVar,
	opPushFunVar,
	opPushWorkspaceVar,

	opPushFun,

	opCallImmediate,
	opCallLocalVar,
	opCallFunVar,
	opCallWorkspaceVar,

	
	opDot,
	opComma,
	opBindLocal,
	opBindLocalFromList,
	opBindWorkspaceVar,
	opBindWorkspaceVarFromList,

	opParens,
	opNewVList,
	opNewZList,
	opNewForm,
	opInherit,
	opEach,
	
	opReturn,
	
	kNumOpcodes
};

extern const char* opcode_name[kNumOpcodes];
	
#endif

