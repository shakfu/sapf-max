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

#include "VM.hpp"
#include "clz.hpp"

struct SetPair
{
	V mValue;
	int mIndex;
};

class Set : public Object
{
	int mSize;
	int mCap;
	int* mIndices;
	SetPair* mPairs;
	
	void grow(Thread& th);
	void alloc(int cap);
    
	Set(const Set& that) {}
public:
	
	Set(int capacity) { alloc(capacity); }
    Set(Thread& th, P<List> list) { alloc(32); putAll(th, list); }
    
	virtual ~Set();
	
	virtual const char* TypeName() const override { return "Set"; }
    
	virtual bool isSet() const override { return true; }
	virtual bool Equals(Thread& th, Arg v) override;
	
	int size() { return mSize; }
	
	bool has(Thread& th, V& value);
    int indexOf(Thread& th, V& value);
	
	void put(Thread& th, V& inValue, int inIndex);
    
    void putAll(Thread& th, P<List>& list);
	
	virtual V at(int64_t i) override { return mPairs[i].mValue; }
    
    P<List> asVList(Thread& th);
    P<List> asZList(Thread& th);
};


bool Set::Equals(Thread& th, Arg v) 
{
	if (v.Identical(this)) return true;
	if (!v.isSet()) return false;
	if (this == v.o()) return true;
	Set* that = (Set*)v.o();
	if (mSize != that->size()) return false;
    
	for (int64_t i = 0; i < mSize; ++i) {
		V& value = mPairs[i].mValue;
		V u;
		if (!that->has(th, value)) return false;
	}
    
	return true;
}

bool Set::has(Thread& th, V& value) 
{	
	int hash = value.Hash();
	int mask = mCap * 2 - 1;
	int index = hash & mask;
	int* indices = mIndices;
	SetPair* pairs = mPairs;
	
	while(1) {
		int index2 = indices[index]-1;
		if (index2 == -1) {
			return false;
		}
		V& testVal = pairs[index2].mValue;
		if (value.Equals(th, testVal)) {
			return true;
		}
		index = (index + 1) & mask;
	}	
	
	return false;
}

int Set::indexOf(Thread& th, V& value)
{	
	int hash = value.Hash();
	int mask = mCap * 2 - 1;
	int index = hash & mask;
	int* indices = mIndices;
	SetPair* pairs = mPairs;
	
	while(1) {
		int index2 = indices[index]-1;
		if (index2 == -1) {
			return -1;
		}
		V& testVal = pairs[index2].mValue;
		if (value.Equals(th, testVal)) {
			return pairs[index2].mIndex;
		}
		index = (index + 1) & mask;
	}	
	
	return -1;
}


Set::~Set()
{
	delete [] mPairs;
	free(mIndices);
}

void Set::alloc(int cap)
{
	cap = NEXTPOWEROFTWO(cap);
	mPairs = new SetPair[cap];
	mIndices = (int*)calloc(2 * cap, sizeof(int));
	mCap = cap;
	mSize = 0;
}

void Set::grow(Thread& th)
{
	free(mIndices);
    
	SetPair* oldPairs = mPairs;
	int oldSize = mSize;
    
	alloc(mCap * 2);
	
	for (int i = 0; i < oldSize; ++i) {
		SetPair& pair = oldPairs[i];
		put(th, pair.mValue, pair.mIndex);
	}
	
	delete [] oldPairs;
}

void Set::put(Thread& th, V& inValue, int inIndex)
{
	if (mSize == mCap) {
		grow(th);
	}
    
	int hash = inValue.Hash();
	int mask = mCap * 2 - 1;
	int index = hash & mask;
	int* indices = mIndices;
	SetPair* pairs = mPairs;
    
	while(1) {
		int index2 = indices[index]-1;
		if (index2 == -1) {
			index2 = mSize++;
			indices[index] = index2+1;
			pairs[index2].mValue = inValue;
			pairs[index2].mIndex = inIndex;
			return;
		}
		V& testVal = pairs[index2].mValue;
		if (inValue.Equals(th, testVal)) {
			return;
		}
		index = (index + 1) & mask;
	}
}

void Set::putAll(Thread& th, P<List>& in)
{
    // caller must ensure that in is finite.
    int64_t insize = in->length(th);
    in = in->pack(th);
    for (int i = 0; i < insize; ++i) {
        V val = in->at(i);
        put(th, val, i);
    }
}


P<List> Set::asVList(Thread& th)
{
    int64_t outsize = size();
    
    P<List> out = new List(itemTypeV, outsize);
    
    for (int i = 0; i < outsize; ++i) {
        out->add(at(i));
    }
    
    return out;
}

P<List> Set::asZList(Thread& th)
{
    int64_t outsize = size();
    
    P<List> out = new List(itemTypeZ, outsize);
    
    for (int i = 0; i < outsize; ++i) {
        out->add(at(i));
    }
    
    return out;
}

static P<List> nub(Thread& th, P<List> in)
{
    P<Set> set = new Set(th, in);
    
    return set->asVList(th);
}


static P<List> set_or(Thread& th, P<List> a, P<List> b)
{
    P<Set> set = new Set(32);

    set->putAll(th, a);
    set->putAll(th, b);
    return a->isZ() && b->isZ() ? set->asZList(th) : set->asVList(th);
}

static P<List> set_and(Thread& th, P<List> a, P<List> b)
{
    P<Set> setA = new Set(th, a);
    P<Set> setB = new Set(th, b);
    P<List> out = new List(a->isZ() && b->isZ() ? itemTypeZ : itemTypeV, 32);
    
    for (int64_t i = 0; i < setA->size(); ++i) {
        V v = setA->at(i);
        if (setB->has(th, v)) out->add(v);
    }
    
    return out;
}

static P<List> set_minus(Thread& th, P<List> a, P<List> b)
{
    P<Set> setA = new Set(th, a);
    P<Set> setB = new Set(th, b);
    P<List> out = new List(a->isZ() && b->isZ() ? itemTypeZ : itemTypeV, 32);
    
    for (int64_t i = 0; i < setA->size(); ++i) {
        V v = setA->at(i);
        if (!setB->has(th, v)) out->add(v);
    }
    
    return out;
}

static P<List> set_xor(Thread& th, P<List> a, P<List> b)
{
    P<Set> setA = new Set(th, a);
    P<Set> setB = new Set(th, b);
    P<List> out = new List(a->isZ() && b->isZ() ? itemTypeZ : itemTypeV, 32);
    
    for (int64_t i = 0; i < setA->size(); ++i) {
        V v = setA->at(i);
        if (!setB->has(th, v)) out->add(v);
    }
    for (int64_t i = 0; i < setB->size(); ++i) {
        V v = setB->at(i);
        if (!setA->has(th, v)) out->add(v);
    }
    
    return out;
}

static bool subset(Thread& th, P<List> a, P<List> b)
{
    P<Set> setA = new Set(th, a);
    P<Set> setB = new Set(th, b);

    for (int64_t i = 0; i < setA->size(); ++i) {
        V v = setA->at(i);
        if (!setB->has(th, v)) return false;
    }
	
	return true;
}

static bool set_equals(Thread& th, P<List> a, P<List> b)
{
    P<Set> setA = new Set(th, a);
    P<Set> setB = new Set(th, b);

	return setA->Equals(th, setB);
}

/* 
 
 list minus values from set.
 
 
 

 
 
 
*/


static void nub_(Thread& th, Prim* prim)
{
    P<List> a = th.popList("nub : a");
    
    if (!a->isFinite())
        indefiniteOp("nub : a", "");
    
    th.push(nub(th, a));
}

static void set_or_(Thread& th, Prim* prim)
{
    P<List> b = th.popList("|| : b");
    if (!b->isFinite())
        indefiniteOp("|| : b", "");
    
    P<List> a = th.popList("|| : a");
    
    if (!a->isFinite())
        indefiniteOp("|| : a", "");
    
    th.push(set_or(th, a, b));
}

static void set_and_(Thread& th, Prim* prim)
{
    P<List> b = th.popList("&& : b");
    if (!b->isFinite())
        indefiniteOp("&& : b", "");
    
    P<List> a = th.popList("&& : a");
    
    if (!a->isFinite())
        indefiniteOp("&& : a", "");
    
    th.push(set_and(th, a, b));
}


static void set_xor_(Thread& th, Prim* prim)
{
    P<List> b = th.popList("set_xor : b");
    if (!b->isFinite())
        indefiniteOp("set_xor : b", "");
    
    P<List> a = th.popList("set_xor : a");
    
    if (!a->isFinite())
        indefiniteOp("set_xor : a", "");
    
    th.push(set_xor(th, a, b));
}

static void set_minus_(Thread& th, Prim* prim)
{
    P<List> b = th.popList("set_minus : b");
    if (!b->isFinite())
        indefiniteOp("set_minus : b", "");
    
    P<List> a = th.popList("set_minus : a");
    
    if (!a->isFinite())
        indefiniteOp("set_minus : a", "");
    
    th.push(set_minus(th, a, b));
}

static void subset_(Thread& th, Prim* prim)
{
    P<List> b = th.popList("subset : b");
    if (!b->isFinite())
        indefiniteOp("subset : b", "");
    
    P<List> a = th.popList("subset : a");
    
    if (!a->isFinite())
        indefiniteOp("subset : a", "");
    
    th.pushBool(subset(th, a, b));
}


static void set_equals_(Thread& th, Prim* prim)
{
    P<List> b = th.popList("set_equals : b");
    if (!b->isFinite())
        indefiniteOp("set_equals : b", "");
    
    P<List> a = th.popList("set_equals : a");
    
    if (!a->isFinite())
        indefiniteOp("set_equals : a", "");
    
    th.push(set_equals(th, a, b));
}

struct FindV : Gen
{
	P<Set> mSet;
	VIn items;
	
	FindV(Thread& th, Arg inItems, P<Set> const& inSet)
		: Gen(th, itemTypeV, inItems.isFinite()), mSet(inSet), items(inItems) {}
		
	const char* TypeName() const override { return "FindV"; }
	
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			V *a;
			if (items(th, n, astride, a)) {
				setDone();
				break;
			}
			for (int i = 0; i < n; ++i) {
				out[i] = mSet->indexOf(th, *a);
				a += astride;
			}
			items.advance(n);
			framesToFill -= n;
		}
		produce(framesToFill);
	}
};

struct FindZ : Gen
{
	P<Set> mSet;
	ZIn items;
	
	FindZ(Thread& th, Arg inItems, P<Set> const& inSet)
		: Gen(th, itemTypeZ, inItems.isFinite()), mSet(inSet), items(inItems) {}
		
	const char* TypeName() const override { return "FindZ"; }
	
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			Z *a;
			if (items(th, n, astride, a)) {
				setDone();
				break;
			}
			for (int i = 0; i < n; ++i) {
				V va = *a;
				out[i] = mSet->indexOf(th, va);
				a += astride;
			}
			items.advance(n);
			framesToFill -= n;
		}
		produce(framesToFill);
	}
};


struct SetHasV : Gen
{
	P<Set> mSet;
	VIn items;
	
	SetHasV(Thread& th, Arg inItems, P<Set> const& inSet)
		: Gen(th, itemTypeV, inItems.isFinite()), mSet(inSet), items(inItems) {}
		
	const char* TypeName() const override { return "SetHasV"; }
	
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			V *a;
			if (items(th, n, astride, a)) {
				setDone();
				break;
			}
			for (int i = 0; i < n; ++i) {
				out[i] = mSet->has(th, *a);
				a += astride;
			}
			items.advance(n);
			framesToFill -= n;
		}
		produce(framesToFill);
	}
};

struct SetHasZ : Gen
{
	P<Set> mSet;
	ZIn items;
	
	SetHasZ(Thread& th, Arg inItems, P<Set> const& inSet)
		: Gen(th, itemTypeV, inItems.isFinite()), mSet(inSet), items(inItems) {}
		
	const char* TypeName() const override { return "SetHasZ"; }
	
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			Z *a;
			if (items(th, n, astride, a)) {
				setDone();
				break;
			}
			for (int i = 0; i < n; ++i) {
				V va = *a;
				out[i] = mSet->has(th, va);
				a += astride;
			}
			items.advance(n);
			framesToFill -= n;
		}
		produce(framesToFill);
	}
};

static V findBase(Thread& th, V& a, P<Set> const& inSet)
{
	V result;
	if (a.isList()) {
		if (a.isZList()) {
			result = new List(new FindZ(th, a, inSet));
		} else {
			result = new List(new FindV(th, a, inSet));
		}
	} else {
		result = inSet->indexOf(th, a);
	}
	return result;
}

static void find_(Thread& th, Prim* prim)
{
    P<List> b = th.popList("find : list");
    if (!b->isFinite())
        indefiniteOp("find : list", "");

	V a = th.pop();

    P<Set> setB = new Set(th, b);
	
	th.push(findBase(th, a, setB));
}

static V hasBase(Thread& th, V& a, P<Set> const& inSet)
{
	V result;
	if (a.isList()) {
		if (a.isZList()) {
			result = new List(new SetHasZ(th, a, inSet));
		} else {
			result = new List(new SetHasV(th, a, inSet));
		}
	} else {
		result = inSet->has(th, a);
	}
	return result;
}

static void sethas_(Thread& th, Prim* prim)
{
    P<List> b = th.popList("Shas : list");
    if (!b->isFinite())
        indefiniteOp("Shas : list", "");

	V a = th.pop();

    P<Set> setB = new Set(th, b);
	
	th.push(hasBase(th, a, setB));
}

#pragma mark ADD STREAM OPS

#define DEF(NAME, N, HELP) 	vm.def(#NAME, N, NAME##_, HELP);
#define DEFMCX(NAME, N, HELP) 	vm.defmcx(#NAME, N, NAME##_, HELP);
#define DEFAM(NAME, MASK, HELP) 	vm.defautomap(#NAME, #MASK, NAME##_, HELP);

void AddSetOps();
void AddSetOps()
{
	vm.addBifHelp("\n*** set operations ***");
    vm.def("S", 1, 1, nub_, "(list --> set) removes all duplicates from a finite list.");
    vm.def("S|", 2, 1, set_or_, "(listA listB --> set) returns the set union of the elements of lists A and B.");
    vm.def("S&", 2, 1, set_and_, "(listA listB --> set) returns the set intersection of the elements of lists A and B.");
    vm.def("Sx", 2, 1, set_xor_, "(listA listB --> set) returns the set of the elements which occur in list A or B, but not both.");
    vm.def("S-", 2, 1, set_minus_, "(listA listB --> set) returns the set of the elements of listA which do not occur in listB.");
    vm.def("S=", 2, 1, set_equals_, "(listA listB --> set) returns 1 if the set of elements in listA is equal to the set of elements in listB.");
    vm.def("subset?", 2, 1, subset_, "(listA listB --> set) returns 1 if the set of elements of listA is a subset of the set of elements of listB. else 0.");
    vm.def("find", 2, 1, find_, "(item(s) list --> set) returns index of item in finite list, or -1 if not in list.");
    vm.def("Shas", 2, 1, sethas_, "(item(s) list --> set) returns 1 if finite list contains item(s), else 0.");
}






