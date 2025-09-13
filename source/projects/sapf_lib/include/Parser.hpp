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

#ifndef __Parser_h__
#define __Parser_h__

#include "VM.hpp"

bool parseElems(Thread& th, P<Code>& code);

//////////////////////////////////

#pragma mark PIPER SYNTAX

struct AST : RCObj
{
	virtual const char* TypeName() const { return "AST"; }
	virtual void dump(std::ostream& ost, int indent) = 0;
	virtual void codegen(P<Code>& code) = 0;
};

typedef P<AST> ASTPtr;

ASTPtr parseExpr(const char*& in);

void printi(std::ostream& ost, int indent, const char* fmt, ...);
void prints(std::ostream& ost, const char* fmt, ...);

#endif
