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

#ifndef __ErrorCodes_h__
#define __ErrorCodes_h__

const int errNone = 0;
const int errHalt = -1000;
const int errFailed = -1001;
const int errIndefiniteOperation = -1002;
const int errWrongType = -1003;
const int errOutOfRange = -1004;
const int errSyntax = -1005;
const int errInternalError = -1006;
const int errWrongState = -1007;
const int errNotFound = -1008;
const int errStackOverflow = -1009;
const int errStackUnderflow = -1010;
const int errInconsistentInheritance = -1011;
const int errUndefinedOperation = -1012;
const int errUserQuit = -1013;
const int kNumErrors = 14;

extern const char* errString[kNumErrors];

#endif

