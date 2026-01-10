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

#include "MultichannelExpansion.hpp"
#include "clz.hpp"
#include <algorithm>
#include <vector>

// multi channel mapping is a special case of auto mapping where the mask is all z's.

class MultichannelMapper : public Gen
{
	V fun;
	int numArgs;
	VIn args[kMaxArgs];
public:
	
	MultichannelMapper(Thread& th, bool inFinite, int n, V* inArgs, Arg inFun)
		: Gen(th, itemTypeV, inFinite), fun(inFun), numArgs(n)
	{
		for (int i = 0; i < numArgs; ++i) {
			args[i].set(inArgs[i]);
		}
	}
	
	const char* TypeName() const override { return "MultichannelMapper"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			SaveStack ss(th);
			for (int j = 0; j < numArgs; ++j) {
				V v;
				if (args[j].one(th, v)) {
					setDone();
					produce(framesToFill);
					return;
				}
				th.push(v);
			}
			try {
				fun.apply(th);
			} catch (...) {
				setDone();
				produce(framesToFill);
				throw;
			}
			out[i] = th.pop();
			--framesToFill;
		}
		produce(framesToFill);
	}
};


template <int N>
void mcx_(Thread& th, Prim* prim)
{
	if (th.stackDepth() < N)
		throw errStackUnderflow;
		
	V& fun = prim->v;
	V* args = &th.top() - (N - 1);
	
	bool hasVList = false;
	bool isFinite = false;
	for (int k = 0; k < N; ++k) {
		if (args[k].isVList()) {
			hasVList = true;
			if (args[k].isFinite()) 
				isFinite = true;
		}
	}
	
	if (hasVList) {
		List* s = new List(new MultichannelMapper(th, isFinite, N, args, prim));
		th.popn(N);
		th.push(s);
	} else {
		fun.apply(th);
	}

}


const char* kAaa = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const size_t kAaaLength = strlen(kAaa);

const char* kZzz = "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz";
const size_t kZzzLength = strlen(kZzz);

const char* Prim::GetAutoMapMask() const
{
	//return NoEachOps() || mNumArgs == 0 ? nullptr : kAaa + kAaaLength - mNumArgs;
	return nullptr;
}

class MultichannelMapPrim : public Prim
{
public:
	MultichannelMapPrim(PrimFun _primFun, Arg _v, int n, const char* inName, const char* inHelp) 
		: Prim(_primFun, _v, n, 1, inName, inHelp)
	{
	}
	
	virtual const char* GetAutoMapMask() const { return kZzz + kZzzLength - mTakes; }
	
};

Prim* mcx(int n, Arg f, const char* name, const char* help)
{
	PrimFun pf = nullptr;
	switch (n) {
		case  1 : pf = mcx_< 1>; break;
		case  2 : pf = mcx_< 2>; break;
		case  3 : pf = mcx_< 3>; break;
		case  4 : pf = mcx_< 4>; break;
		case  5 : pf = mcx_< 5>; break;
		case  6 : pf = mcx_< 6>; break;
		case  7 : pf = mcx_< 7>; break;
		case  8 : pf = mcx_< 8>; break;
		case  9 : pf = mcx_< 9>; break;
		case 10 : pf = mcx_<10>; break;
		case 11 : pf = mcx_<11>; break;
		case 12 : pf = mcx_<12>; break;
		default : throw errFailed;
	}
		
	return new MultichannelMapPrim(pf, f, n, name, help);
}

class AutoMapPrim : public Prim
{
public:
	const char* mask;
	AutoMapPrim(PrimFun _primFun, Arg _v, int n, const char* inMask, const char* inName, const char* inHelp) 
		: Prim(_primFun, _v, n, 1, inName, inHelp), mask(inMask)
	{
	}
	
	virtual const char* GetAutoMapMask() const { return mask; }
	
};

class AutoMapper : public Gen
{
	V fun;
	int numArgs;
	BothIn args[kMaxArgs];
public:
	
	AutoMapper(Thread& th, bool inFinite, const char* inMask, int n, V* inArgs, Arg inFun)
		: Gen(th, itemTypeV, inFinite), fun(inFun), numArgs(n)
	{
		for (int i = 0; i < numArgs; ++i) {
			switch (inMask[i]) {
				case 'a' :
					args[i].setConstant(inArgs[i]);
					break;
				case 'z' : // auto map over V lists, but not Z lists.
					args[i].setv(inArgs[i]);
					break;
				default :
					post("unrecognized AutoMap char '%c'\n", inMask[i]);
				case 'k' :
					args[i].set(inArgs[i]);
					
			}
		}
	}
	
	const char* TypeName() const override { return "AutoMapper"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			SaveStack ss(th);
			for (int j = 0; j < numArgs; ++j) {
				V v;
				if (args[j].one(th, v)) {
					setDone();
					produce(framesToFill);
					return;
				}
				th.push(v);
			}
			try {
				fun.apply(th);
			} catch (...) {
				setDone();
				produce(framesToFill);
				throw;
			}
			out[i] = th.pop();
			--framesToFill;
		}
		produce(framesToFill);
	}
};

/*
	a - as is. argument is not automapped.
	z - argument is expected to be a signal or scalar, streams are auto mapped.
	k - argument is expected to be a scalar, signals and streams are automapped.
*/

template <int N>
void automap_(Thread& th, Prim* prim)
{

	const char* mask = ((AutoMapPrim*)prim)->mask;

	if (th.stackDepth() < N)
		throw errStackUnderflow;
		
	V* args = &th.top() - (N - 1);
	
	bool canMap = false;
	bool isFinite = false;
	for (int k = 0; k < N; ++k) {
		switch (mask[k]) {
			case 'a' :
				break;
			case 'z' :
				if (args[k].isVList()) {
					canMap = true;
					if (args[k].isFinite()) 
						isFinite = true;
				}
				break;
			default :
				post("unrecognized AutoMap char '%c'\n", mask[k]);
				throw errFailed;
				break;
			case 'k' :
				if (args[k].isList()) {
					canMap = true;
					if (args[k].isFinite()) 
						isFinite = true;
				}
				break;
		}
	}
	
	if (canMap) {
		List* s = new List(new AutoMapper(th, isFinite, mask, N, args, prim));
		th.popn(N);
		th.push(s);
	} else {
		prim->v.apply(th);
	}

}

Prim* automap(const char* mask, int n, Arg f, const char* inName, const char* inHelp)
{
	PrimFun pf = nullptr;
	switch (n) {
		case  1 : pf = automap_< 1>; break;
		case  2 : pf = automap_< 2>; break;
		case  3 : pf = automap_< 3>; break;
		case  4 : pf = automap_< 4>; break;
		case  5 : pf = automap_< 5>; break;
		case  6 : pf = automap_< 6>; break;
		case  7 : pf = automap_< 7>; break;
		case  8 : pf = automap_< 8>; break;
		case  9 : pf = automap_< 9>; break;
		case 10 : pf = automap_<10>; break;
		case 11 : pf = automap_<11>; break;
		case 12 : pf = automap_<12>; break;
		default : throw errFailed;
	}
		
	return new AutoMapPrim(pf, f, n, mask, inName, inHelp);
}





class EachMapper : public Gen
{
	const int level;
	const int numLevels;
	V fun;
	ArgInfo args;
public:
	
	EachMapper(Thread& th, bool inFinite, int inLevel, int inNumLevels, const ArgInfo& inArgs, Arg inFun)
		: Gen(th, itemTypeV, inFinite), level(inLevel), numLevels(inNumLevels), args(inArgs), fun(inFun)
	{
	}
	
	const char* TypeName() const override { return "EachMapper"; }
	
	void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		if (level == 0) {
			for (int i = 0; i < mBlockSize; ++i) {
				SaveStack ss(th);
				for (int j = 0; j < args.numArgs; ++j) {
					V v;
					if (args.arg[j].in.one(th, v)) {
						setDone();
						produce(framesToFill);
						return;
					}
					th.push(v);
				}
				try {
					fun.apply(th);
				} catch (...) {
					setDone();
					produce(framesToFill);
					throw;
				}
				out[i] = th.pop();
				
				--framesToFill;
			}
		} else {
			ArgInfo subargs;
			subargs.numArgs = args.numArgs;
			for (int j = 0; j < args.numArgs; ++j) {
				subargs.arg[j].mask = args.arg[j].mask;
			}
			
			int bit = 1 << (numLevels - level);
			
			bool mmIsFinite = true;
						
			for (int i = 0; i < mBlockSize; ++i) {
				V argv[kMaxArgs];
				bool allConstant = true;
				for (int j = 0; j < args.numArgs; ++j) {
					V v;
					if (args.arg[j].in.one(th, argv[j])) {
						setDone();
						produce(framesToFill);
						return;
					}
					if (argv[j].isList() && (args.arg[j].mask & bit))
						allConstant = false;
				}
				
				if (allConstant) {
					SaveStack ss(th);
					for (int j = 0; j < args.numArgs; ++j) {
						th.push(argv[j]);
					}
					try {
						fun.apply(th);
					} catch (...) {
						setDone();
						produce(framesToFill);
						throw;
					}
					out[i] = th.pop();
				} else {
					for (int j = 0; j < args.numArgs; ++j) {
						V v = argv[j];
						if (args.arg[j].mask & bit) {
							if (v.isList() && !v.isFinite())
								mmIsFinite = false;
							subargs.arg[j].in.set(v);
						} else {
							subargs.arg[j].in.setConstant(v);
						}
					}
				
					out[i] = new List(new EachMapper(th, mmIsFinite, level - 1, numLevels, subargs, fun));
				}
				--framesToFill;
			}
		}
		produce(framesToFill);
	}	
};

List* handleEachOps(Thread& th, int numArgs, Arg fun)
{
	ArgInfo args;
	V argv[kMaxArgs];
	
	args.numArgs = numArgs;
	int32_t maxMask = 0;

	bool mmIsFinite = true;

	for (int i = numArgs-1; i >= 0; --i) {
		V v = th.pop();
		argv[i] = v;
		if (v.isEachOp()) {
			
			EachOp* adv = (EachOp*)v.o();
			args.arg[i].mask = adv->mask;
			maxMask |= adv->mask;
			if (adv->mask & 1) {
				if (!adv->v.isFinite()) 
					mmIsFinite = false;
				args.arg[i].in.set(adv->v);
			} else {
				args.arg[i].in.setConstant(adv->v);
			}
		} else {
			args.arg[i].in.setConstant(v);
			args.arg[i].mask = 0;
		}
	}
	if (maxMask > 1 && maxMask != NEXTPOWEROFTWO(maxMask) - 1) {
		post("there are empty levels of iteration. mask: %x\n", maxMask);
		throw errFailed;
	}
	
	int numLevels = maxMask <= 1 ? 1 : LOG2CEIL(maxMask);
	return new List(new EachMapper(th, mmIsFinite, numLevels-1, numLevels, args, fun));
}


class Flop : public Gen
{
	size_t numArgs;
	std::vector<BothIn> args;
public:
	
	Flop(Thread& th, bool inFinite, size_t n, V* inArgs)
		: Gen(th, itemTypeV, inFinite), numArgs(n)
	{
		args.reserve(numArgs);
		for (size_t i = 0; i < numArgs; ++i) {
			BothIn in;
			in.set(inArgs[i]);
			args.push_back(in);
		}
	}
	
	const char* TypeName() const override { return "Flop"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			P<List> s = new List(itemTypeV, numArgs);
			P<Array> a = s->mArray;
			for (size_t j = 0; j < numArgs; ++j) {
				V v;
				if (args[j].one(th, v)) {
					setDone();
					produce(framesToFill);
					return;
				}
				a->add(v);
			}
			out[i] = s;
			--framesToFill;
		}
		produce(framesToFill);
	}
};


class Flops : public Gen
{
	size_t numArgs;
	std::vector<VIn> args;
public:
	
	Flops(Thread& th, bool inFinite, size_t n, V* inArgs)
		: Gen(th, itemTypeV, inFinite), numArgs(n)
	{
		args.reserve(numArgs);
		for (size_t i = 0; i < numArgs; ++i) {
			VIn in;
			in.set(inArgs[i]);
			args.push_back(in);
		}
	}
	
	const char* TypeName() const override { return "Flops"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			P<List> s = new List(itemTypeV, numArgs);
			P<Array> a = s->mArray;
			for (size_t j = 0; j < numArgs; ++j) {
				V v;
				if (args[j].one(th, v)) {
					setDone();
					produce(framesToFill);
					return;
				}
				a->add(v);
			}
			out[i] = s;
			--framesToFill;
		}
		produce(framesToFill);
	}
};

class Flopz : public Gen
{
	size_t numArgs;
	std::vector<ZIn> args;
public:
	
	Flopz(Thread& th, bool inFinite, size_t n, V* inArgs)
		: Gen(th, itemTypeV, inFinite), numArgs(n)
	{
		args.reserve(numArgs);
		for (size_t i = 0; i < numArgs; ++i) {
			ZIn in;
			in.set(inArgs[i]);
			args.push_back(in);
		}
	}
	
	const char* TypeName() const override { return "Flopz"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			P<List> s = new List(itemTypeZ, numArgs);
			P<Array> a = s->mArray;
			for (size_t j = 0; j < numArgs; ++j) {
				Z z;
				if (args[j].onez(th, z)) {
					setDone();
					produce(framesToFill);
					return;
				}
				a->addz(z);
			}
			out[i] = s;
			--framesToFill;
		}
		produce(framesToFill);
	}
};

class FlopNth : public Gen
{
	VIn _in;
	size_t _nth;
public:
	
	FlopNth(Thread& th, size_t nth, Arg in)
		: Gen(th, itemTypeV, false), _nth(nth), _in(in)
	{
	}
	
	const char* TypeName() const override { return "FlopNth"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			V v;
			if (_in.one(th, v)) {
				setDone();
				produce(framesToFill);
				return;
			}
			if (v.isList()) {
				if (!v.isFinite()) {
					setDone();
					produce(framesToFill);
					return;
				}
				P<List> u = (List*)v.o();
				u = u->pack(th);
				P<Array> b = u->mArray;
				
				out[i] = u->wrapAt(_nth);
			} else {
				out[i] = v;
			}
			
			
			--framesToFill;
		}
		produce(framesToFill);
	}
};

class FlopsNth : public Gen
{
	VIn _in;
	size_t _nth;
public:
	
	FlopsNth(Thread& th, size_t nth, Arg in)
		: Gen(th, itemTypeV, false), _nth(nth), _in(in)
	{
	}
	
	const char* TypeName() const override { return "FlopsNth"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			V v;
			if (_in.one(th, v)) {
				setDone();
				produce(framesToFill);
				return;
			}
			if (v.isVList()) {
				if (!v.isFinite()) {
					setDone();
					produce(framesToFill);
					return;
				}
				P<List> u = (List*)v.o();
				u = u->pack(th);
				P<Array> b = u->mArray;
				
				out[i] = u->wrapAt(_nth);
			} else {
				out[i] = v;
			}
			
			--framesToFill;
		}
		produce(framesToFill);
	}
};


void flop_(Thread& th, Prim* prim)
{

	P<List> s = th.popVList("flop : list");
		
	if (s->isFinite()) {
	
		s = s->pack(th);
		
		V* args = s->mArray->v();
		size_t N = s->mArray->size();
		
		bool hasList = false;
		bool isFinite = false;
		bool allZ = true;
		for (size_t k = 0; k < N; ++k) {
			if (args[k].isList()) {
				hasList = true;
				if (args[k].isFinite()) 
					isFinite = true;
				if (!args[k].isZList())
					allZ = false;
			}
		}
		
		if (hasList) {
			if (allZ) {
				List* result = new List(new Flopz(th, isFinite, N, args));
				th.push(result);
			} else {
				List* result = new List(new Flop(th, isFinite, N, args));
				th.push(result);
			}
		} else {
			th.push(s);
		}
	} else {
		VIn in(s);
		V first;
		if (in.one(th, first)) {
			post("flop : can't flop an empty list.");
			throw errFailed;
		}
		if (!first.isList()) {
			wrongType("flop : first item in list", "List", first);
		}
		if (!first.isFinite()) {
			post("flop : can't flop an infinite list of infinite lists.");
			throw errFailed;
		}
		
		size_t n = first.length(th);
		
		List* result = new List(itemTypeV, n);
		for (size_t i = 0; i < n; ++i) {
			result->add(new List(new FlopNth(th, i, s)));
		}
		th.push(result);
	}

}

void flops_(Thread& th, Prim* prim)
{

	P<List> s = th.popVList("flops : list");
		
	if (s->isFinite()) {
	
		s = s->pack(th);
		
		V* args = s->mArray->v();
		size_t N = s->mArray->size();
		
		bool hasList = false;
		bool isFinite = false;
		bool allZ = true;
		for (size_t k = 0; k < N; ++k) {
			if (args[k].isList()) {
				hasList = true;
				if (args[k].isFinite()) 
					isFinite = true;
				if (!args[k].isZList())
					allZ = false;
			}
		}
		
		if (hasList) {
			if (allZ) {
				List* result = new List(new Flops(th, isFinite, N, args));
				th.push(result);
			} else {
				List* result = new List(new Flops(th, isFinite, N, args));
				th.push(result);
			}
		} else {
			th.push(s);
		}
	} else {
		VIn in(s);
		V first;
		if (in.one(th, first)) {
			post("flops : can't flop an empty list.");
			throw errFailed;
		}
		if (!first.isList()) {
			wrongType("flops : first item in list", "List", first);
		}
		if (!first.isFinite()) {
			post("flops : can't flop an infinite list of infinite lists.");
			throw errFailed;
		}
		
		size_t n = first.length(th);
		
		List* result = new List(itemTypeV, n);
		for (size_t i = 0; i < n; ++i) {
			result->add(new List(new FlopsNth(th, i, s)));
		}
		th.push(result);
	}

}


void flop1_(Thread& th, Prim* prim)
{

	P<List> s = th.popVList("flop1 : list");
		
	if (s->isFinite()) {
	
		s = s->pack(th);
		
		V* args = s->mArray->v();
		size_t N = s->mArray->size();
		
		bool hasList = false;
		bool isFinite = false;
		bool allZ = true;
		for (size_t k = 0; k < N; ++k) {
			if (args[k].isList()) {
				hasList = true;
				if (args[k].isFinite()) 
					isFinite = true;
				if (!args[k].isZList())
					allZ = false;
			}
		}
		
		if (hasList) {
			if (allZ) {
				List* result = new List(new Flopz(th, isFinite, N, args));
				th.push(result);
			} else {
				List* result = new List(new Flop(th, isFinite, N, args));
				th.push(result);
			}
		} else {
			List* result = new List(itemTypeV, 1);
			result->add(s);
			th.push(result);
		}
	} else {
		VIn in(s);
		V first;
		if (in.one(th, first)) {
			post("flop1 : can't flop an empty list.");
			throw errFailed;
		}
		if (!first.isList()) {
			wrongType("flop1 : first item in list", "List", first);
		}
		if (!first.isFinite()) {
			post("flop1 : can't flop an infinite list of infinite lists.");
			throw errFailed;
		}
		
		size_t n = first.length(th);
		
		List* result = new List(itemTypeV, n);
		for (size_t i = 0; i < n; ++i) {
			result->add(new List(new FlopNth(th, i, s)));
		}
		th.push(result);
	}

}


class Lace : public Gen
{
	size_t numArgs;
	size_t argPos;
	std::vector<BothIn> args;
public:
	
	Lace(Thread& th, bool inFinite, size_t n, V* inArgs)
		: Gen(th, itemTypeV, inFinite), numArgs(n), argPos(0)
	{
		args.reserve(numArgs);
		for (size_t i = 0; i < numArgs; ++i) {
			BothIn in;
			in.set(inArgs[i]);
			args.push_back(in);
		}
	}
	
	const char* TypeName() const override { return "Lace"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			V v;
			if (args[argPos].one(th, v)) {
				setDone();
				produce(framesToFill);
				return;
			}
			out[i] = v;
			--framesToFill;
			if (++argPos >= numArgs) argPos = 0;
		}
		produce(framesToFill);
	}
};

class Lacez : public Gen
{
	size_t numArgs;
	size_t argPos;
	std::vector<ZIn> args;
public:
	
	Lacez(Thread& th, bool inFinite, size_t n, V* inArgs)
		: Gen(th, itemTypeZ, inFinite), numArgs(n), argPos(0)
	{
		post("Lacez\n");
		args.reserve(numArgs);
		for (size_t i = 0; i < numArgs; ++i) {
			ZIn in;
			in.set(inArgs[i]);
			args.push_back(in);
		}
	}
	
	const char* TypeName() const override { return "Lacez"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			Z z;
			if (args[argPos].onez(th, z)) {
				setDone();
				produce(framesToFill);
				return;
			}
			out[i] = z;
			--framesToFill;
			if (++argPos >= numArgs) argPos = 0;
		}
		produce(framesToFill);
	}
};

void lace_(Thread& th, Prim* prim)
{

	P<List> s = th.popList("lace : list");
	if (!s->isVList())
		wrongType("lace : list", "VList", s);
		
	if (!s->isFinite())
		indefiniteOp("lace : list", "");
	
	s = s->pack(th);
	
	V* args = s->mArray->v();
	size_t N = s->mArray->size();
	
	bool hasList = false;
	bool isFinite = false;
	bool allZ = true;
	for (size_t k = 0; k < N; ++k) {
		if (args[k].isList()) {
			hasList = true;
			if (args[k].isFinite()) 
				isFinite = true;
			if (!args[k].isZList())
				allZ = false;
		}
	}
	
	if (hasList) {
		if (allZ) {
			List* result = new List(new Lacez(th, isFinite, N, args));
			th.push(result);
		} else {
			List* result = new List(new Lace(th, isFinite, N, args));
			th.push(result);
		}
	} else {
		th.push(s);
	}

}


class Sel : public Gen
{
	int64_t numArgs;
	std::vector<BothIn> args;
	BothIn sel;
public:
	
	Sel(Thread& th, bool inFinite, int64_t n, V* inArgs, Arg inSel)
		: Gen(th, itemTypeV, inFinite), numArgs(n), sel(inSel)
	{
		args.reserve(numArgs);
		for (int64_t i = 0; i < numArgs; ++i) {
			BothIn in;
			in.set(inArgs[i]);
			args.push_back(in);
		}
	}
	
	const char* TypeName() const override { return "Sel"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			int64_t k;
			if (sel.onei(th, k)) {
				setDone();
				break;
			}
			k = sc_imod(k, numArgs);
			for (int64_t j = 0; j < numArgs; ++j) {
				V v;
				if (args[j].one(th, v)) {
					setDone();
					produce(framesToFill);
					return;
				}
				if (j == k) {
					out[i] = v;
					--framesToFill;
				}
			}
		}
		produce(framesToFill);
	}
};

class Selz : public Gen
{
	int64_t numArgs;
	std::vector<ZIn> args;
	BothIn sel;
public:
	
	Selz(Thread& th, bool inFinite, int64_t n, V* inArgs, Arg inSel)
		: Gen(th, itemTypeZ, inFinite), numArgs(n), sel(inSel)
	{
		args.reserve(numArgs);
		for (int64_t i = 0; i < numArgs; ++i) {
			ZIn in;
			in.set(inArgs[i]);
			args.push_back(in);
		}
	}
	
	const char* TypeName() const override { return "Selz"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			int64_t k;
			if (sel.onei(th, k)) {
				setDone();
				break;
			}
			k = sc_imod(k, numArgs);
			for (int64_t j = 0; j < numArgs; ++j) {
				Z z;
				if (args[j].onez(th, z)) {
					setDone();
					produce(framesToFill);
					return;
				}
				if (j == k) {
					out[i] = z;
					--framesToFill;
				}
			}
		}
		produce(framesToFill);
	}
};

class Sell : public Gen
{
	int64_t numArgs;
	std::vector<BothIn> args;
	BothIn sel;
public:
	
	Sell(Thread& th, bool inFinite, int64_t n, V* inArgs, Arg inSel)
		: Gen(th, itemTypeV, inFinite), numArgs(n), sel(inSel)
	{
		args.reserve(numArgs);
		for (int64_t i = 0; i < numArgs; ++i) {
			BothIn in;
			in.set(inArgs[i]);
			args.push_back(in);
		}
	}
	
	const char* TypeName() const override { return "Sell"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			int64_t k;
			if (sel.onei(th, k)) {
				setDone();
				break;
			}
			k = sc_imod(k, numArgs);
			V v;
			if (args[k].one(th, v)) {
				setDone();
				produce(framesToFill);
				return;
			}
			out[i] = v;
			--framesToFill;
		}
		produce(framesToFill);
	}
};

class Sellz : public Gen
{
	int64_t numArgs;
	std::vector<ZIn> args;
	BothIn sel;
public:
	
	Sellz(Thread& th, bool inFinite, int64_t n, V* inArgs, Arg inSel)
		: Gen(th, itemTypeZ, inFinite), numArgs(n), sel(inSel)
	{
		args.reserve(numArgs);
		for (int64_t i = 0; i < numArgs; ++i) {
			ZIn in;
			in.set(inArgs[i]);
			args.push_back(in);
		}
	}
	
	const char* TypeName() const override { return "Sellz"; }

	virtual void pull(Thread& th) override
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		
		for (int i = 0; i < mBlockSize; ++i) {
			int64_t k;
			if (sel.onei(th, k)) {
				setDone();
				break;
			}
			k = sc_imod(k, numArgs);
			Z z;
			if (args[k].onez(th, z)) {
				setDone();
				produce(framesToFill);
				return;
			}
			out[i] = z;
			--framesToFill;
		}
		produce(framesToFill);
	}
};


void sel_(Thread& th, Prim* prim)
{
	P<List> indices = th.popList("sel : indices");

	P<List> s = th.popList("sel : list");
	if (!s->isVList())
		wrongType("sel : list", "VList", s);
		
	if (!s->isFinite())
		indefiniteOp("sel : list", "");
	
	s = s->pack(th);
	
	V* args = s->mArray->v();
	size_t N = s->mArray->size();
	
	bool hasList = false;
	bool isFinite = false;
	bool allZ = true;
	for (size_t k = 0; k < N; ++k) {
		if (args[k].isList()) {
			hasList = true;
			if (args[k].isFinite()) 
				isFinite = true;
			if (!args[k].isZList())
				allZ = false;
		}
	}
	
	if (hasList) {
		if (allZ) {
			List* result = new List(new Selz(th, isFinite, N, args, indices));
			th.push(result);
		} else {
			List* result = new List(new Sel(th, isFinite, N, args, indices));
			th.push(result);
		}
	} else {
		th.push(s);
	}

}

void sell_(Thread& th, Prim* prim)
{
	P<List> indices = th.popList("sell : indices");

	P<List> s = th.popList("sell : list");
	if (!s->isVList())
		wrongType("sell : list", "VList", s);
		
	if (!s->isFinite())
		indefiniteOp("sell : list", "");
	
	s = s->pack(th);
	
	V* args = s->mArray->v();
	size_t N = s->mArray->size();
	
	bool hasList = false;
	bool isFinite = false;
	bool allZ = true;
	for (size_t k = 0; k < N; ++k) {
		if (args[k].isList()) {
			hasList = true;
			if (args[k].isFinite()) 
				isFinite = true;
			if (!args[k].isZList())
				allZ = false;
		}
	}
	
	if (hasList) {
		if (allZ) {
			List* result = new List(new Sellz(th, isFinite, N, args, indices));
			th.push(result);
		} else {
			List* result = new List(new Sell(th, isFinite, N, args, indices));
			th.push(result);
		}
	} else {
		th.push(s);
	}

}







