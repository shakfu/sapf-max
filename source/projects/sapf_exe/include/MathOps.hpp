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

#ifndef __MathOps_h__
#define __MathOps_h__

#include "VM.hpp"

struct UnaryOpGen : public Gen
{
	VIn _a;
	UnaryOp* op;

	UnaryOpGen(Thread& th, UnaryOp* inOp, Arg a) : Gen(th, itemTypeV, a.isFinite()), op(inOp), _a(a) {}

	virtual const char* TypeName() const override { return "UnaryOpGen"; }
		
	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			V *a;
			if (_a(th, n,astride, a)) {
				setDone();
				break;
			} else {
				op->loop(th, n, a, astride, out);
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

// element-wise
struct BinaryOpGen : public Gen
{
	VIn _a;
	VIn _b;
	BinaryOp* op;

	BinaryOpGen(Thread& th, BinaryOp* inOp, Arg a, Arg b)	: Gen(th, itemTypeV, mostFinite(a, b)), op(inOp), _a(a), _b(b) {}

	virtual const char* TypeName() const override { return "BinaryOpGen"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride, bstride;
			V *a, *b;
			if (_a(th, n,astride, a) || _b(th, n,bstride, b)) {
				setDone();
				break;
			} else {
				op->loop(th, n, a, astride, b, bstride, out);
				_a.advance(n);
				_b.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

struct BinaryOpLinkGen : public Gen
{
	VIn _a;
	VIn _b;
	BinaryOp* op;

	BinaryOpLinkGen(Thread& th, BinaryOp* inOp, Arg a, Arg b)	: Gen(th, itemTypeV, mostFinite(a, b)), op(inOp), _a(a), _b(b) {}

	virtual const char* TypeName() const override { return "BinaryOpLinkGen"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride, bstride;
			V *a, *b;
			if (_a(th, n,astride, a)) {
				produce(framesToFill);
				_b.link(th, mOut);
				setDone();
				return;
			} else if (_b(th, n,bstride, b)) {
				produce(framesToFill);
				_a.link(th, mOut);
				setDone();
				return;
			} else {
				op->loop(th, n, a, astride, b, bstride, out);
				_a.advance(n);
				_b.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

struct UnaryOpZGen : public Gen
{
	ZIn _a;
	UnaryOp* op;
	
	UnaryOpZGen(Thread& th, UnaryOp* inOp, Arg a) : Gen(th, itemTypeZ, a.isFinite()), _a(a), op(inOp) {}

	virtual const char* TypeName() const override { return "UnaryOpZGen"; }

	virtual int numInputs() const { return 1; }
		
	virtual void pull(Thread& th) override;
};

struct BinaryOpZGen : public Gen
{
	ZIn _a;
	ZIn _b;
	BinaryOp* op;
	
	BinaryOpZGen(Thread& th, BinaryOp* _op, Arg a, Arg b)
					: Gen(th, itemTypeZ, mostFinite(a,b)), _a(a), _b(b), op(_op) {}
	
	virtual const char* TypeName() const override { return "BinaryOpZGen"; }
	
	virtual void pull(Thread& th) override;
};

struct BinaryOpLinkZGen : public Gen
{
	ZIn _a;
	ZIn _b;
	BinaryOp* op;
	
	BinaryOpLinkZGen(Thread& th, BinaryOp* _op, Arg a, Arg b)
					: Gen(th, itemTypeZ, mostFinite(a,b)), _a(a), _b(b), op(_op) {}
	
	virtual const char* TypeName() const override { return "BinaryOpLinkZGen"; }
	
	virtual void pull(Thread& th) override;
};


struct ScanOpZGen : public Gen
{
	ZIn _a;
	BinaryOp* op;
	Z z;
	bool once;
	
	ScanOpZGen(Thread& th, Arg a, BinaryOp* _op)
				: Gen(th, itemTypeZ, a.isFinite()), _a(a), op(_op) , once(true) {}
	
	virtual const char* TypeName() const override { return "ScanOpZGen"; }

	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			Z *a;
			if (_a(th, n,astride, a) ) {
				setDone();
				break;
			} else {
				if (once) {
					once = false;
					out[0] = z = a[0];
					op->scanz(n-1, z, a+1, astride, out+1);
				} else {
					op->scanz(n, z, a, astride, out);
				}
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

struct IScanOpZGen : public Gen
{
	ZIn _a;
	Z z;
	BinaryOp* op;
	bool once;
	
	IScanOpZGen(Thread& th, Arg a, Z b, BinaryOp* _op)
				: Gen(th, itemTypeZ, a.isFinite()), _a(a), z(b), op(_op) , once(true) {}
	
	virtual const char* TypeName() const override { return "IScanOpZGen"; }

	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			Z *a;
			if (_a(th, n,astride, a) ) {
				setDone();
				break;
			} else {
				if (once) {
					once = false;
					out[0] = z;
					op->scanz(n-1, z, a, astride, out+1);
					_a.advance(n-1);
				} else {
					op->scanz(n, z, a, astride, out);
					_a.advance(n);
				}
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

struct PairsOpZGen : public Gen
{
	ZIn _a;
	BinaryOp* op;
	Z z;
	bool once;
	
	PairsOpZGen(Thread& th, Arg a, BinaryOp* _op)
				: Gen(th, itemTypeZ, a.isFinite()), _a(a), op(_op) , once(true) {}
	
	virtual const char* TypeName() const override { return "PairsOpZGen"; }

	virtual void pull(Thread& th) override {
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			Z *a;
			if (_a(th, n,astride, a) ) {
				setDone();
				break;
			} else {
				if (once) {
					once = false;
					out[0] = z = a[0];
					op->pairsz(n-1, z, a+1, astride, out+1);
				} else {
					op->pairsz(n, z, a, astride, out);
				}
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};


struct IPairsOpZGen : public Gen
{
	ZIn _a;
	Z z;
	BinaryOp* op;
	bool once;
	
	IPairsOpZGen(Thread& th, Arg a, Z b, BinaryOp* _op)
				: Gen(th, itemTypeZ, a.isFinite()), _a(a), z(b), op(_op) , once(true) {}
	
	virtual const char* TypeName() const override { return "IPairsOpZGen"; }

	virtual void pull(Thread& th) override {
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			Z *a;
			if (_a(th, n,astride, a) ) {
				setDone();
				break;
			} else {
				if (once) {
					once = false;
					out[0] = z;
					z = a[0];
					op->pairsz(n-1, z, a+1, astride, out+1);
				} else {
					op->pairsz(n, z, a, astride, out);
				}
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};


struct ScanOpGen : public Gen
{
	VIn _a;
	BinaryOp* op;
	V z;
	bool once;
	
	ScanOpGen(Thread& th, Arg a, BinaryOp* _op)
				: Gen(th, itemTypeV, a.isFinite()), _a(a), op(_op) , once(true) {}
	
	virtual const char* TypeName() const override { return "ScanOpGen"; }

	virtual void pull(Thread& th) override {
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			V *a;
			if (_a(th, n,astride, a) ) {
				setDone();
				break;
			} else {
				if (once) {
					once = false;
					out[0] = z = a[0];
					op->scan(th, n-1, z, a+1, astride, out+1);
				} else {
					op->scan(th, n, z, a, astride, out);
				}
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

struct IScanOpGen : public Gen
{
	VIn _a;
	BinaryOp* op;
	V z;
	bool once;
	
	IScanOpGen(Thread& th, Arg a, Arg b, BinaryOp* _op)
				: Gen(th, itemTypeV, a.isFinite()), _a(a), z(b), op(_op) , once(true) {}
	
	virtual const char* TypeName() const override { return "IScanOpGen"; }

	virtual void pull(Thread& th) override {
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			V *a;
			if (_a(th, n,astride, a) ) {
				setDone();
				break;
			} else {
				if (once) {
					once = false;
					out[0] = z;
					op->scan(th, n-1, z, a, astride, out+1);
					_a.advance(n-1);
				} else {
					op->scan(th, n, z, a, astride, out);
					_a.advance(n);
				}
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

struct PairsOpGen : public Gen
{
	VIn _a;
	BinaryOp* op;
	V z;
	bool once;
	
	PairsOpGen(Thread& th, Arg a, BinaryOp* _op)
				: Gen(th, itemTypeV, a.isFinite()), _a(a), op(_op) , once(true) {}
	
	virtual const char* TypeName() const override { return "PairsOpGen"; }

	virtual void pull(Thread& th) override {
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			V *a;
			if (_a(th, n,astride, a) ) {
				setDone();
				break;
			} else {
				if (once) {
					once = false;
					out[0] = z = a[0];
					op->pairs(th, n-1, z, a+1, astride, out+1);
				} else {
					op->pairs(th, n, z, a, astride, out);
				}
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

struct IPairsOpGen : public Gen
{
	VIn _a;
	V z;
	BinaryOp* op;
	bool once;
	
	IPairsOpGen(Thread& th, Arg a, Arg b, BinaryOp* _op)
				: Gen(th, itemTypeV, a.isFinite()), _a(a), z(b), op(_op) , once(true) {}
	
	virtual const char* TypeName() const override { return "IPairsOpGen"; }

	virtual void pull(Thread& th) override {
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			V *a;
			if (_a(th, n,astride, a) ) {
				setDone();
				break;
			} else {
				if (once) {
					once = false;
					out[0] = z;
					z = a[0];
					op->pairs(th, n-1, z, a+1, astride, out+1);
				} else {
					op->pairs(th, n, z, a, astride, out);
				}
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};


#endif
