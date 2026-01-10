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

#include "Object.hpp"
#include "VM.hpp"
#include "clz.hpp"
#include "MathOps.hpp"
#include "Opcode.hpp"
#include <algorithm>
#include <cstdarg>

void post(const char* fmt, ...)
{
    va_list vargs;
    va_start(vargs, fmt);
    vprintf(fmt, vargs);
}

void zprintf(std::string& out, const char* fmt, ...)
{
	char s[1024];
	va_list args;
	va_start(args, fmt);
	vsnprintf(s, 1024, fmt, args);
	out += s;
}

#pragma mark ERROR HANDLING


Thread gDummyThread;

[[noreturn]] void notFound(Arg key)
{
	post("notFound ");
	key.print(gDummyThread); // keys are either symbols or numbers neither of which will use the thread argument to print.
	post("\n");
	throw errNotFound;
}

[[noreturn]] void wrongType(const char* msg, const char* expected, Arg got)
{
	post("error: wrong type for  %s .  expected %s. got %s.\n", msg, expected, got.TypeName());
	throw errWrongType;
}

[[noreturn]] void syntaxError(const char* msg)
{
	post("syntax error: %s\n", msg);
	throw errSyntax;
}

[[noreturn]] void indefiniteOp(const char* msg1, const char* msg2)
{
	post("error: operation on indefinite object  %s%s\n", msg1, msg2);
	throw errIndefiniteOperation;
}

#pragma mark OBJECT


Object::Object()
	: scratch(0), elemType(0), finite(false), flags(0)
{
#if COLLECT_MINFO
	++vm.totalObjectsAllocated;
#endif
}

Object::~Object()
{
#if COLLECT_MINFO
	++vm.totalObjectsFreed;
#endif
}

void Ref::set(Arg inV)
{
	O oldval = nullptr;
	if (inV.isObject()) {
		O newval = inV.o();
	
		newval->retain();
		{
			SpinLocker lock(mSpinLock);
			oldval = o;
			o = newval;
		}
	} else {
		Z newval = inV.f;
		{
			SpinLocker lock(mSpinLock);
			oldval = o;
			o = nullptr;
			z = newval;
		}
	}
	if (oldval)
		oldval->release();
}

void ZRef::set(Z inZ)
{
	z = inZ;
}

V Ref::deref() const
{
	V out;
	{
		SpinLocker lock(mSpinLock);
		if (o) {
			out = o;
			if (o) o->retain();
		}
		else out = z;
	}
	return out;
}

V ZRef::deref() const
{
	return z;
}

Z Object::derefz() const
{
	return asFloat();
}

Z ZRef::derefz() const
{
	return z;
}


Form::Form(P<Table> const& inTable, P<Form> const& inNext)
	: Object(), mTable(inTable), mNextForm(inNext)
{
}

volatile int64_t gTreeNodeSerialNumber;

GForm::GForm(P<GTable> const& inTable, P<GForm> const& inNext)
	: Object(), mTable(inTable), mNextForm(inNext)
{
}

GForm::GForm(P<GForm> const& inNext)
	: Object(), mNextForm(inNext)
{ 
	mTable = new GTable();
}

P<GForm> consForm(P<GTable> const& inTable, P<GForm> const& inNext) { return new GForm(inTable, inNext); }
P<Form> consForm(P<Table> const& inTable, P<Form> const& inNext) { return new Form(inTable, inNext); }

void Object::apply(Thread& th) {
	th.push(this);
}

class PushFunContext
{
	Thread& th;
	P<Fun> fun;
	size_t stackBase, localBase;
public:
	PushFunContext(Thread& inThread, P<Fun> const& inFun)
		: th(inThread), fun(th.fun),
		stackBase(th.stackBase), localBase(th.localBase)
	{
	}
	~PushFunContext()
	{
		th.popLocals();
		th.fun = fun;
		th.setStackBaseTo(stackBase);
		th.setLocalBase(localBase);
	}
};

class PushREPLFunContext
{
	Thread& th;
	P<Fun> fun;
	size_t stackBase, localBase;
public:
	PushREPLFunContext(Thread& inThread, P<Fun> const& inFun)
		: th(inThread), fun(th.fun),
		stackBase(th.stackBase), localBase(th.localBase)
	{
	}
	~PushREPLFunContext()
	{
		th.popLocals();
		th.fun = fun;
		th.setStackBaseTo(stackBase);
		th.setLocalBase(localBase);
	}
};

void Fun::runREPL(Thread& th)
{
	if (th.stackDepth() < NumArgs()) {
		post("expected %qd args on stack. Only have %qd\n", (int64_t)NumArgs(), (int64_t)th.stack.size());
		throw errStackUnderflow;
	}

	PushREPLFunContext pfc(th, this);

	th.setLocalBase();

	if (NumArgs()) {
		th.local.insert(th.local.end(), th.stack.end() - NumArgs(), th.stack.end());
		th.stack.erase(th.stack.end() - NumArgs(), th.stack.end());
	}
	size_t numLocalVars = NumLocals() - NumArgs();
	if (numLocalVars) {
		V v;
		th.local.insert(th.local.end(), numLocalVars, v);
	}
	
	th.fun = this;
	
	th.run(mDef->mCode->getOps());
}

void Fun::run(Thread& th)
{	
	if (th.stackDepth() < NumArgs()) {
		post("expected %qd args on stack. Only have %qd\n", (int64_t)NumArgs(), (int64_t)th.stack.size());
		throw errStackUnderflow;
	}

	PushFunContext pfc(th, this);

	th.setLocalBase();

	if (NumArgs()) {
		th.local.insert(th.local.end(), th.stack.end() - NumArgs(), th.stack.end());
		th.stack.erase(th.stack.end() - NumArgs(), th.stack.end());
	}
	size_t numLocalVars = NumLocals() - NumArgs();
	if (numLocalVars) {
		V v;
		th.local.insert(th.local.end(), numLocalVars, v);
	}
	
	th.setStackBase();

	th.fun = this;
	
	th.run(mDef->mCode->getOps());
}

void Fun::apply(Thread& th) 
{ 
	int numArgs = NumArgs();

	if (th.stackDepth() < (size_t)numArgs) {
		throw errStackUnderflow;
	}

	if (NoEachOps()) {
		run(th);
	} else {
		if (th.stackDepth()) {
			bool haveEachOps = false;
			V* args = &th.top() - numArgs + 1;

			for (int i = 0; i < numArgs; ++i) {
				if (args[i].isEachOp()) {
					haveEachOps = true;
					break;
				}
			}

			if (haveEachOps) {
				List* s = handleEachOps(th, numArgs, this);
				th.push(s);
			} else {
				run(th);
			}
		} else {
			run(th);
		}
	}
}

Fun::Fun(Thread& th, FunDef* def)
	: mDef(def), mWorkspace(def->Workspace())
{
	if (NumVars()) {
		mVars.insert(mVars.end(), th.stack.end() - NumVars(), th.stack.end());
		th.stack.erase(th.stack.end() - NumVars(), th.stack.end());
	}
}

FunDef::FunDef(Thread& th, P<Code> const& inCode, uint16_t inNumArgs, uint16_t inNumLocals, uint16_t inNumVars, P<String> const& inHelp) 
	: mCode(inCode), mNumArgs(inNumArgs), mNumLocals(inNumLocals),  mNumVars(inNumVars), 
    mWorkspace(th.mWorkspace),
    mHelp(inHelp)
{
}

Fun::~Fun()
{
}

void Prim::apply(Thread& th) 
{
	apply_n(th, mTakes);
}

void Prim::apply_n(Thread& th, size_t n)
{ 
	if (th.stackDepth() < n)
		throw errStackUnderflow;
	
	if (NoEachOps()) {
		prim(th, this); 
	} else {
	
		if (n) {
			V* args = &th.top() - n + 1;

			bool haveEachOps = false;
			for (size_t i = 0; i < n; ++i) {
				if (args[i].isEachOp()) {
					haveEachOps = true;
					break;
				}
			}
			
			if (haveEachOps) {
				List* s = handleEachOps(th, (int)n, this);
				th.push(s);
			} else {
				prim(th, this); 
			}
		} else {
			prim(th, this); 
		}
	}
}

bool Form::get(Thread& th, Arg key, V& value) const
{
    const Form* e = this;
	int64_t hash = key.Hash();
    do {
       if (e->mTable->getWithHash(th, key, hash, value)) {
           return true;
        }
        e = (Form*)e->mNextForm();
    } while (e);
    return false;
}


V Form::mustGet(Thread& th, Arg key) const
{
	V value;
	if (!get(th, key, value)) {
		post("not found: ");
		throw errNotFound; 
	}
	return value;
}

bool GForm::get(Thread& th, Arg key, V& value) const
{
    const GForm* e = this;
    do {
        if (e->mTable->get(th, key, value)) {
            return true;
        }
        e = (GForm*)e->mNextForm();
    } while (e);
    return false;
}

GForm* GForm::putImpure(Arg key, Arg value)
{
	if (mTable->putImpure(key, value)) return this;
	return putPure(key, value);
}

GForm* GForm::putPure(Arg inKey, Arg inValue)
{
	int64_t inKeyHash = inKey.Hash();
	return new GForm(mTable->putPure(inKey, inKeyHash, inValue), mNextForm);
}

V GForm::mustGet(Thread& th, Arg key) const
{
	V value;
	if (!get(th, key, value)) {
		post("not found: ");
		throw errNotFound; 
	}
	return value;
}


GTable* GTable::putPure(Arg inKey, int64_t inKeyHash, Arg inValue)
{
    auto tree = mTree.load();
	if (tree) {
		return new GTable(tree->putPure(inKey, inKeyHash, inValue));
	} else {
		int64_t serialNo = ++gTreeNodeSerialNumber;
		return new GTable(new TreeNode(inKey, inKeyHash, inValue, serialNo, nullptr, nullptr));
	}
}

TreeNode* TreeNode::putPure(Arg inKey, int64_t inKeyHash, Arg inValue)
{
	if (inKeyHash == mHash && inKey.Identical(mKey)) {
		return new TreeNode(mKey, mHash, inValue, mSerialNumber, mLeft, mRight);
	} 
    auto left = mLeft.load();
    auto right = mRight.load();
    if (inKeyHash < mHash) {
        if (left) {
            return new TreeNode(mKey, mHash, mValue, mSerialNumber, left->putPure(inKey, inKeyHash, inValue), right);
        } else {
            int64_t serialNo = ++gTreeNodeSerialNumber;
            return new TreeNode(mKey, mHash, mValue, mSerialNumber, new TreeNode(inKey, inKeyHash, inValue, serialNo, nullptr, nullptr), right);
        }
	} else {
        if (right) {
            return new TreeNode(mKey, mHash, mValue, mSerialNumber, left, right->putPure(inKey, inKeyHash, inValue));
        } else {
            int64_t serialNo = ++gTreeNodeSerialNumber;
            return new TreeNode(mKey, mHash, mValue, mSerialNumber, left, new TreeNode(inKey, inKeyHash, inValue, serialNo, nullptr, nullptr));
        }
	}
}

static bool TreeNodeEquals(Thread& th, TreeNode* a, TreeNode* b)
{
    while (1) {
        if (!a) return !b;
        if (!b) return false;
        
        if (!a->mKey.Equals(th, b->mKey)) return false;
        if (!a->mValue.Equals(th, b->mValue)) return false;
        if (a->mLeft == 0) {
            if (b->mLeft != 0) return false;
        } else {
            if (b->mLeft == 0) return false;
            if (!TreeNodeEquals(th, a->mLeft, b->mLeft)) return false;
        }
        a = a->mRight;
        b = b->mRight;
    }
}

bool GTable::Equals(Thread& th, Arg v)
{
	if (v.Identical(this)) return true;
	if (!v.isGTable()) return false;
	if (this == v.o()) return true;
	GTable* that = (GTable*)v.o();
    return TreeNodeEquals(th, mTree.load(), that->mTree.load());
}

void GTable::print(Thread& th, std::string& out, int depth)
{
	std::vector<P<TreeNode> > vec = sorted();
	for (size_t i = 0; i < vec.size(); ++i) {
		P<TreeNode>& p = vec[i];
		zprintf(out, "   ");
		p->mValue.print(th, out);
		zprintf(out, " :");
		p->mKey.print(th, out);
		zprintf(out, "\n");
	}
}

void GTable::printSomethingIWant(Thread& th, std::string& out, int depth)
{
	std::vector<P<TreeNode> > vec = sorted();
	for (size_t i = 0; i < vec.size(); ++i) {
		P<TreeNode>& p = vec[i];
		if (p->mValue.leaves() != 0 && p->mValue.leaves() != 1) {
			zprintf(out, "   ");
			p->mKey.print(th, out);
			zprintf(out, " : ");
			p->mValue.print(th, out);
			zprintf(out, "\n");
		}
	}
}

bool GTable::get(Thread& th, Arg inKey, V& outValue) const
{
	int32_t inKeyHash = inKey.Hash();
	TreeNode* tree = mTree.load();
	while (1) {
		if (tree == nullptr) return false;
		int32_t treeKeyHash = tree->mKey.Hash();
		if (inKeyHash == treeKeyHash) {
			outValue = tree->mValue;
			return true;
		} else if (inKeyHash < treeKeyHash) {
			tree = tree->mLeft.load();
		} else {
			tree = tree->mRight.load();
		}
	}
}

bool GTable::getInner(Arg inKey, V& outValue) const
{
	int32_t inKeyHash = inKey.Hash();
	TreeNode* tree = mTree.load();
	while (1) {
		if (tree == nullptr) return false;
		int32_t treeKeyHash = tree->mKey.Hash();
		if (inKeyHash == treeKeyHash) {
			outValue = tree->mValue;
			return true;
		} else if (inKeyHash < treeKeyHash) {
			tree = tree->mLeft.load();
		} else {
			tree = tree->mRight.load();
		}
	}
}

V GTable::mustGet(Thread& th, Arg inKey) const
{
	V value;
	if (get(th, inKey, value)) return value;
	
	throw errNotFound;
}

bool GTable::putImpure(Arg inKey, Arg inValue)
{
	int32_t inKeyHash = inKey.Hash();
	volatile std::atomic<TreeNode*>* treeNodePtr = &mTree;
	while (1) {
		TreeNode* tree = treeNodePtr->load();
		if (tree == nullptr) {
            int64_t serialNo = ++gTreeNodeSerialNumber;
			TreeNode* newNode = new TreeNode(inKey, inKeyHash, inValue, serialNo, nullptr, nullptr);
			newNode->retain();
            TreeNode* nullNode = nullptr;
            if (treeNodePtr->compare_exchange_weak(nullNode, newNode)) {
                break;
            }
            newNode->release();
		} else {
			int32_t treeKeyHash = tree->mKey.Hash();
			if (treeKeyHash == inKeyHash) {
				return false; // cannot rebind an existing value.
			} else if (inKeyHash < treeKeyHash) {
				treeNodePtr = &tree->mLeft;
			} else {
				treeNodePtr = &tree->mRight;
			}
		}
	}
	return true;
}


void TreeNode::getAll(std::vector<P<TreeNode> >& vec)
{
	P<TreeNode> node = this;
	do {
		vec.push_back(node);
        auto left = node->mLeft.load();
		if (left) left->getAll(vec);
		node = node->mRight.load();
	} while (node());
}

static bool compareTreeNodes(P<TreeNode> const& a, P<TreeNode> const& b)
{
	return a->mSerialNumber < b->mSerialNumber;
}

std::vector<P<TreeNode> > GTable::sorted() const
{
	std::vector<P<TreeNode> > vec;
    auto tree = mTree.load();
	if (tree) {
		tree->getAll(vec);
		sort(vec.begin(), vec.end(), compareTreeNodes);
	}
	return vec;
}

/////////


V List::unaryOp(Thread& th, UnaryOp* op)
{
	if (isVList())
		return new List(new UnaryOpGen(th, op, this));
	else
		return new List(new UnaryOpZGen(th, op, this));
		
}

V List::binaryOpWithReal(Thread& th, BinaryOp* op, Z _a)
{
	if (isVList())
		return op->makeVList(th, _a, this);
	else
		return op->makeZList(th, _a, this);
}

V List::binaryOpWithVList(Thread& th, BinaryOp* op, List* _a)
{
	return op->makeVList(th, _a, this);
}

V List::binaryOpWithZList(Thread& th, BinaryOp* op, List* _a)
{
	if (isVList()) 
		return op->makeVList(th, _a, this);
	else
		return op->makeZList(th, _a, this);
}

void V::apply(Thread& th)
{
	if (o) {
		o->apply(th);
	} else {
		th.push(*this);
	}
}

V V::deref()
{
	if (o) {
		return o->deref();
	} else {
		return *this;
	}
}

V V::mustGet(Thread& th, Arg key) const
{
	if (o) {
		return o->mustGet(th, key);
	} else {
		notFound(key);
		throw errNotFound;
	}
}

bool V::get(Thread& th, Arg key, V& value) const
{
	if (o) {
		return o->get(th, key, value);
	} else {
		notFound(key);
		throw errNotFound;
	}
}

int V::Hash() const 
{
	if (o) {
		return o->Hash();
	} else {
        union {
            double f;
            uint64_t i;
        } u;
        u.f = f;
		return (int)::Hash64(u.i);
	}
}

V V::unaryOp(Thread& th, UnaryOp* op) const
{
	return !o ? V(op->op(f)) : o->unaryOp(th, op);
}

V V::binaryOp(Thread& th, BinaryOp* op, Arg _b) const
{
	return !o ? _b.binaryOpWithReal(th, op, f) : o->binaryOp(th, op, _b);
}

V V::binaryOpWithReal(Thread& th, BinaryOp* op, Z _a) const
{
	return !o ? V(op->op(_a, f)) : o->binaryOpWithReal(th, op, _a);
}

V V::binaryOpWithVList(Thread& th, BinaryOp* op, List* _a) const
{
	return !o ? op->makeVList(th, _a, *this) : o->binaryOpWithVList(th, op, _a);
}

V V::binaryOpWithZList(Thread& th, BinaryOp* op, List* _a) const
{
	return !o ? op->makeZList(th, _a, *this) : o->binaryOpWithZList(th, op, _a);
}

void Object::printDebug(Thread& th, int depth)
{ 
	std::string s;
	printDebug(th, s, depth);
	post("%s", s.c_str());
}

void Object::print(Thread& th, int depth)
{ 
	std::string s;
	print(th, s, depth);
	post("%s", s.c_str());
}

void Object::printShort(Thread& th, int depth)
{ 
	std::string s;
	printShort(th, s, depth);
	post("%s", s.c_str());
}

void Object::print(Thread& th, std::string& out, int depth)
{ 
	char s[64];
	snprintf(s, 64, "#%s", TypeName());
	out += s; 
}

void Object::printDebug(Thread& th, std::string& out, int depth)
{ 
	char s[64];
	snprintf(s, 64, "{%s, %p}", TypeName(), this);
	out += s; 
}

void Prim::print(Thread& th, std::string& out, int depth)
{ 
	char s[64];
	snprintf(s, 64, "{%s, %s}", TypeName(), mName);
	out += s; 
}

void Prim::printDebug(Thread& th, std::string& out, int depth)
{ 
	char s[64];
	snprintf(s, 64, "{%s, %s}", TypeName(), mName);
	out += s; 
}


void Ref::print(Thread& th, std::string& out, int depth)
{
	V v = deref();
	v.print(th, out, depth);
	zprintf(out, " R");
	
}

void ZRef::print(Thread& th, std::string& out, int depth)
{
	zprintf(out, "%g ZR", z);	
}

void String::print(Thread& th, std::string& out, int depth)
{
	zprintf(out, "%s", (char*)s);
}

void String::printDebug(Thread& th, std::string& out, int depth)
{
	zprintf(out, "\"%s\"", (char*)s);
}

void GForm::print(Thread& th, std::string& out, int depth)
{
	if (mNextForm) {
		mNextForm->print(th, out, depth);
        zprintf(out, "new\n");
	} else {
		zprintf(out, "New\n");
	}
	
	mTable->print(th, out, depth+1);
}

void Form::print(Thread& th, std::string& out, int depth)
{
	if (depth >= vm.printDepth) {
		zprintf(out, "{...} ");
		return;
	}
	
	zprintf(out, "{");
	if (mNextForm) {
		mNextForm->print(th, out, depth);
		zprintf(out, "  ");
	}
	
	mTable->print(th, out, depth+1);
	
	zprintf(out, "}");
}


void EachOp::print(Thread& th, std::string& out, int inDepth)
{
	v.print(th, out, inDepth);
	zprintf(out, " ");
	
	// try to print as concisely as possible.
	if (mask == 1) {
		zprintf(out, "@");
	} else if (ONES(mask) == 1 && 1 + CTZ(mask) <= 9) {
		zprintf(out, "@%d", 1 + CTZ(mask));
	} else if (CTZ(mask) == 0) {
		int n = CTZ(~mask);
		while (n--) 
			zprintf(out, "@");
	} else {
		zprintf(out, "@");
		int32_t m = mask;
		while (m) {
			zprintf(out, "%c", '0' + (m&1));
			m >>= 1;
		}
	}
}


void Code::print(Thread& th, std::string& out, int depth)
{
	if (depth >= vm.printDepth) {
		zprintf(out, "#Code{...}");
		return;
	}

	zprintf(out, "#Code %d{\n", size());
	Opcode* items = getOps();
	for (int i = 0; i < size(); ++i) {
		zprintf(out, "%4d   %s ", i, opcode_name[items[i].op]);
		items[i].v.printShort(th, out, depth+1);
		out += "\n";
	}
	zprintf(out, "}\n");
}

void List::print(Thread& th, std::string& out, int depth)
{
	if (isV()) {
		zprintf(out, "[");
	} else {
		zprintf(out, "#[");
	}
	
	if (depth >= vm.printDepth) {
		zprintf(out, "...]");
		return;
	}
	
	bool once = true;
	List* list = this;
	
	for (int i = 0; list && i < vm.printLength;) {
		list->force(th);

		Array* a = list->mArray();
		for (int j = 0; j < a->size() && i < vm.printLength; ++j, ++i) {
			if (!once) zprintf(out, " ");
			once = false;
			if (a->isV()) {
				a->v()[j].print(th, out, depth+1);
			} else {
			
				zprintf(out, "%g", a->z()[j]);
			}
		}
		if (i >= vm.printLength) {
			zprintf(out, " ...]");
			return;
		}
		list = list->nextp();
	}
	if (list && list->mNext) {
		zprintf(out, " ...]");
	} else {
		zprintf(out, "]");
	}
}

void V::print(Thread& th, std::string& out, int depth) const
{
	if (!o) zprintf(out, "%g", f);
	else o->print(th, out, depth);
}

void V::printShort(Thread& th, std::string& out, int depth) const
{
	if (!o) zprintf(out, "%g", f);
	else o->printShort(th, out, depth);
}

void V::printDebug(Thread& th, std::string& out, int depth) const
{
	if (!o) zprintf(out, "%g", f);
	else o->printDebug(th, out, depth);
}


void V::print(Thread& th, int depth) const
{
	std::string s;
	print(th, s, depth);
	post("%s", s.c_str());
}

void V::printShort(Thread& th, int depth) const
{
	std::string s;
	printShort(th, s, depth);
	post("%s", s.c_str());
}

void V::printDebug(Thread& th, int depth) const
{
	std::string s;
	printDebug(th, s, depth);
	post("%s", s.c_str());
}

Gen::Gen(Thread& th, int inItemType, bool inFinite)
	: mDone(false), mOut(0), mBlockSize(inItemType == itemTypeV ? vm.VblockSize : th.rate.blockSize)
{
	elemType = inItemType;
	setFinite(inFinite);
#if COLLECT_MINFO
	if (elemType == itemTypeV)
		++vm.totalStreamGenerators;
	else
		++vm.totalSignalGenerators;
#endif
}

Gen::~Gen()
{
#if COLLECT_MINFO
	if (elemType == itemTypeV)
		--vm.totalStreamGenerators;
	else
		--vm.totalSignalGenerators;
#endif
}

void Gen::end() 
{
	setDone();
	mOut->end();
}


void Gen::produce(int shrinkBy)
{
	mOut->mArray->addSize(-shrinkBy);
	mOut = mOut->nextp();
}

void List::end()
{
	assert(mGen);
	mNext = nullptr;
	mGen = nullptr;
	mArray = vm.getNilArray(elemType);
}

V* List::fulfill(int n)
{
	assert(mGen);
	mArray = new Array(elemType, n);
	mArray->setSize(n);
	mNext = new List(mGen);
	mGen = nullptr;
	return mArray->v();
}

V* List::fulfill_link(int n, P<List> const& next)
{
	mArray = new Array(elemType, n);
	mArray->setSize(n);
	mNext = next();
	mGen = nullptr;
	return mArray->v();
}

V* List::fulfill(P<Array> const& inArray)
{
	assert(mGen);
	assert(elemType == inArray->elemType);
	mArray = inArray;
	mNext = new List(mGen);
	mGen = nullptr;
	return mArray->v();
}

Z* List::fulfillz(int n)
{
	assert(mGen);
	mArray = new Array(elemType, n);
	mArray->setSize(n);
	mNext = new List(mGen);
	mGen = nullptr;
	return mArray->z();
}

Z* List::fulfillz_link(int n, P<List> const& next)
{
	mArray = new Array(elemType, n);
	mArray->setSize(n);
	mNext = next;
	mGen = nullptr;
	return mArray->z();
}

Z* List::fulfillz(P<Array> const& inArray)
{
	assert(mGen);
	assert(elemType == inArray->elemType);
	mArray = inArray;
	mNext = new List(mGen);
	mGen = nullptr;
	return mArray->z();
}

void List::link(Thread& th, List* inList)
{
	assert(mGen);
	if (!inList) return;
	inList->force(th);
	mNext = inList->mNext;
	mArray = inList->mArray;
	mGen = nullptr;
}

void List::force(Thread& th)
{	
	SpinLocker lock(mSpinLock);
	if (mGen) {
		P<Gen> gen = mGen; // keep the gen from being destroyed out from under pull().
		if (gen->done()) {
			gen->end();
		} else {
			gen->pull(th);
		}
		// mGen should be NULL at this point because one of the following should have been called: fulfill, link, end.
	}
}

int64_t List::length(Thread& th)
{
	if (!isFinite())
		indefiniteOp("size", "");
	
	List* list = this;
	
	int64_t sum = 0;
	while (list) {
		list->force(th);
		
		sum += list->mArray->size();
		list = list->nextp();
	}
	
	return sum;
}

Array::~Array()
{
	if (isV()) {
		delete [] vv;
	} else {
		free(p);
	}
}

void Array::alloc(int64_t inCap)
{
	if (mCap >= inCap) return;
	mCap = inCap;
	if (isV()) {
		V* oldv = vv;
		vv = new V[mCap];
		for (int64_t i = 0; i < size(); ++i) 
			vv[i] = oldv[i];
		delete [] oldv;
	} else {
		p = realloc(p, mCap * elemSize());
	}
}

void Array::add(Arg inItem)
{
	if (mSize >= mCap)
		alloc(2 * mCap);
	if (isV()) vv[mSize++] = inItem;
	else zz[mSize++] = inItem.asFloat();
}

void Array::addAll(Array* a)
{
	if (!a->mSize)
		return;
		
	int64_t newSize = mSize + a->size();
	if (newSize > mCap)
		alloc(NEXTPOWEROFTWO(newSize));
	
	if (isV()) {
		if (a->isV()) {
			V* x = vv + size();
			V* y = a->vv;
			for (int64_t i = 0; i < a->size(); ++i) x[i] = y[i];
		} else {
			V* x = vv + size();
			Z* y = a->zz;
			for (int64_t i = 0; i < a->size(); ++i) x[i] = y[i];
		}
	} else {
		if (a->isV()) {
			Z* x = zz + size();
			V* y = a->vv;
			for (int64_t i = 0; i < a->size(); ++i) x[i] = y[i].asFloat();
		} else {
			memcpy(zz + mSize, a->zz, a->mSize * sizeof(Z));
		}
	}
	mSize = newSize;
}

void Array::addz(Z inItem)
{
	if (mSize >= mCap)
		alloc(2 * mCap);
	if (isV()) vv[mSize++] = V(inItem);
	else zz[mSize++] = inItem;
}

void Array::put(int64_t inIndex, Arg inItem)
{
	if (isV()) vv[inIndex] = inItem;
	else zz[inIndex] = inItem.asFloat();
}

void Array::putz(int64_t inIndex, Z inItem)
{
	if (isV()) vv[inIndex] = V(inItem);
	else zz[inIndex] = inItem;
}

In::In()
	: mList(nullptr), mOffset(0), mConstant(0.), mIsConstant(true)
{
}

VIn::VIn()
{
	set(0.);
}

VIn::VIn(Arg inValue)
{
	set(inValue);
}

ZIn::ZIn()
{
	set(0.);
}

ZIn::ZIn(Arg inValue)
{
	set(inValue);
}

BothIn::BothIn()
{
	set(0.);
}

BothIn::BothIn(Arg inValue)
{
	set(inValue);
}

void VIn::set(Arg inValue)
{
	if (inValue.isVList()) {
		mList = (List*)inValue.o();
		mOffset = 0;
		mIsConstant = false;
	} else {
		mList = nullptr;
		mConstant = inValue;
		mIsConstant = true;
	}
}

void VIn::setConstant(Arg inValue)
{
	mList = nullptr;
	mConstant = inValue;
	mIsConstant = true;
}


void BothIn::set(Arg inValue)
{
	if (inValue.isList()) {
		mList = (List*)inValue.o();
		mOffset = 0;
		mIsConstant = false;
	} else {
		mList = nullptr;
		mConstant = inValue;
		mIsConstant = true;
	}
}

void BothIn::setv(Arg inValue)
{
	if (inValue.isVList()) {
		mList = (List*)inValue.o();
		mOffset = 0;
		mIsConstant = false;
	} else {
		mList = nullptr;
		mConstant = inValue;
		mIsConstant = true;
	}
}

void BothIn::setConstant(Arg inValue)
{
	mList = nullptr;
	mConstant = inValue;
	mIsConstant = true;
}


void ZIn::set(Arg inValue)
{
	if (inValue.isZList()) {
		mList = (List*)inValue.o();
		mOffset = 0;
		mIsConstant = false;
	} else {
		mList = nullptr;
		mConstant = inValue;
		mIsConstant = true;
	}
}

bool VIn::operator()(Thread& th, int& ioNum, int& outStride, V*& outBuffer)
{
	if (mIsConstant) {
		outStride = 0;
		outBuffer = &mConstant;
		return false;
	}
	
	if (mList) {
        while (1) {
            mList->force(th);
            assert(mList->mArray);
            int num = (int)(mList->mArray->size() - mOffset);
            if (num) {
                ioNum = std::min(ioNum, num);
                outBuffer = mList->mArray->v() + mOffset;
                outStride = 1;
                return false;
            } else if (mList->next()) {
                mList = mList->next();
            } else break;
        }
    }
	mConstant = 0.;
	outStride = 0;
	outBuffer = &mConstant;
	mDone = true;
	return true;
}

bool ZIn::operator()(Thread& th, int& ioNum, int& outStride, Z*& outBuffer)
{
	if (mIsConstant) {
		outStride = 0;
		outBuffer = &mConstant.f;
		return false;
	}
	if (mList) {
		if (mOnce) {
			mOnce = false;
		}
        while (1) {
            mList->force(th);
            assert(mList->mArray);
			int num = (int)(mList->mArray->size() - mOffset);
            if (num) {
                ioNum = std::min(ioNum, num);
                outBuffer = mList->mArray->z() + mOffset;
                outStride = 1;
                return false;
            } else if (mList->next()) {
                mList = mList->next();
            } else break;
        }
	}
	mConstant = 0.;
	outStride = 0;
	outBuffer = &mConstant.f;
    ioNum = 0;
	mDone = true;
	return true;
}

void dumpList(List const* list)
{
	for (int i = 0; list; ++i, list = list->nextp()) {
		printf("  List %d %p mGen %p\n", i, list, list->mGen());
		printf("  List %d %p mNext %p\n", i, list, list->nextp());
		printf("  List %d %p mArray %p %d\n", i, list, list->mArray(), (int)(list->mArray() ? list->mArray->size() : 0));
	}
}

bool VIn::link(Thread& th, List* inList)
{
	if (!mList) return false;
	while (1) {
		mList->force(th);
		assert(mList->mArray);
		if (mOffset) {
			int n = (int)(mList->mArray->size() - mOffset);
			if (n) {
				V* out = inList->fulfill_link(n, mList->next());
				V* in = mList->mArray->v() + mOffset;
				for (int i = 0; i < n; ++i) {
					out[i] = in[i];
				}
				mList = nullptr;
				mDone = true;
				return true;
			} else {
				mList = mList->next();
			}
		} else {
			inList->link(th, mList());
			mList = nullptr;
			mDone = true;
			return true;
		}
	}
}

bool ZIn::link(Thread& th, List* inList)
{	
	if (!mList) return false;

	while (1) {
		mList->force(th);
		assert(mList->mArray);
		if (mOffset) {
			int n = (int)(mList->mArray->size() - mOffset);
			if (n) {
				Z* out = inList->fulfillz_link(n, mList->next());
				Z* in = mList->mArray->z() + mOffset;
				memcpy(out, in, n * sizeof(Z));

				mList = nullptr;
				mDone = true;
				return true;
			} else {
				mList = mList->next();
			}
		} else {
			inList->link(th, mList());
			mList = nullptr;
			mDone = true;
			return true;
		}
	}
}

void In::advance(int inNum)
{
	if (mList) {
		mOffset += inNum;
		if (mOffset == mList->mArray->size()) {
			mList = mList->next();
			mOffset = 0;
		}
	}
}

bool VIn::one(Thread& th, V& v)
{
    if (mIsConstant) {
		v = mConstant;
		return false;
    }
    while (mList) {
        if (mOffset == 0) {
            mList->force(th);
		}
        if (mOffset == mList->mArray->size()) {
            mList = mList->next();
            mOffset = 0;
        } else {
			v = mList->mArray->v()[mOffset++];
			if (mOffset == mList->mArray->size()) {
				mList = mList->next();
				mOffset = 0;
			}
			return false;
		}
    }
	mDone = true;
    return true;
}

bool ZIn::onez(Thread& th, Z& z)
{
    if (mIsConstant) {
		z = mConstant.f;
		return false;
    }
    while (mList) {
        if (mOffset == 0) {
            mList->force(th);
		}
        if (mOffset == mList->mArray->size()) {
            mList = mList->next();
            mOffset = 0;
        } else {
			z = mList->mArray->z()[mOffset++];
			if (mOffset == mList->mArray->size()) {
				mList = mList->next();
				mOffset = 0;
			}
			return false;
		}
    }
	mDone = true;
    return true;
}

bool ZIn::peek(Thread& th, Z& z)
{
    if (mIsConstant) {
		z = mConstant.f;
		return false;
    }
    while (mList) {
        if (mOffset == 0) {
            mList->force(th);
		}
        if (mOffset == mList->mArray->size()) {
            mList = mList->next();
            mOffset = 0;
        } else {
			z = mList->mArray->z()[mOffset];
			return false;
		}
    }
	mDone = true;
    return true;
}

bool BothIn::one(Thread& th, V& v)
{
    if (mIsConstant) {
		v = mConstant;
		return false;
    }
    while (mList) {
        if (mOffset == 0) {
            mList->force(th);
		}
        if (mOffset == mList->mArray->size()) {
			mList = mList->next();
            mOffset = 0;
        } else {
			if (mList->isV())
				v = mList->mArray->v()[mOffset++];
			else
				v = mList->mArray->z()[mOffset++];
				
			if (mOffset == mList->mArray->size()) {
				mList = mList->next();
				mOffset = 0;
			}
			return false;
		}
    }
	mDone = true;
    return true;
}

bool BothIn::onez(Thread& th, Z& z)
{
    if (mIsConstant) {
		z = mConstant.asFloat();
		return false;
    }
    while (mList) {
        if (mOffset == 0) {
            mList->force(th);
		}
        if (mOffset == mList->mArray->size()) {
			mList = mList->next();
            mOffset = 0;
        } else {
			if (mList->isV())
				z = mList->mArray->v()[mOffset++].asFloat();
			else
				z = mList->mArray->z()[mOffset++];
				
			if (mOffset == mList->mArray->size()) {
				mList = mList->next();
				mOffset = 0;
			}
			return false;
		}
    }
	mDone = true;
    return true;
}

bool BothIn::onei(Thread& th, int64_t& i)
{
	Z z = 0.;
	bool result = onez(th, z);
	i = (int64_t)floor(z);
	return result;
}

bool ZIn::bench(Thread& th, int& ioNum)
{
	int framesToFill = ioNum;
	int framesFilled = 0;
	while (framesToFill) {
		int n = framesToFill;
		int astride;
		Z* a;
		if (operator()(th, n, astride, a)) {
			ioNum = framesFilled;
			return true;
		}
		framesToFill -= n;
		framesFilled += n;
		advance(n);
	}
	ioNum = framesFilled;
	return false;
}

bool ZIn::fill(Thread& th, int& ioNum, Z* outBuffer, int outStride)
{
	int framesToFill = ioNum;
	int framesFilled = 0;
	while (framesToFill) {
		int n = framesToFill;
		int astride;
		Z* a;
		if (operator()(th, n, astride, a)) {
			for (int i = 0, k = 0; i < framesToFill; ++i)	{
				outBuffer[k] = 0.;
				k += outStride;
			}
			ioNum = framesFilled;
			return true;
		}
		for (int i = 0, j = 0, k = 0; i < n; ++i)	{
			outBuffer[k] = a[j];
			j += astride;
			k += outStride;
		}
		framesToFill -= n;
		framesFilled += n;
		advance(n);
		outBuffer += n * outStride;
	}
	ioNum = framesFilled;
	return false;
}

bool ZIn::fill(Thread& th, int& ioNum, float* outBuffer, int outStride)
{
	int framesToFill = ioNum;
	int framesFilled = 0;
	while (framesToFill) {
		int n = framesToFill;
		int astride;
		Z* a;
		if (operator()(th, n, astride, a)) {
			for (int i = 0, k = 0; i < framesToFill; ++i)	{
				outBuffer[k] = 0.;
				k += outStride;
			}
			ioNum = framesFilled;
			return true;
		}
		for (int i = 0, j = 0, k = 0; i < n; ++i)	{
			outBuffer[k] = a[j];
			j += astride;
			k += outStride;
		}
		framesToFill -= n;
		framesFilled += n;
		advance(n);
		outBuffer += n * outStride;
	}
	ioNum = framesFilled;
	return false;
}

void ZIn::hop(Thread& th, int framesToAdvance)
{
	P<List> list = mList;
	int offset = mOffset;
	while (list && framesToAdvance) {
		list->force(th);
		int avail = (int)(list->mArray->size() - offset);
		if (avail >= framesToAdvance) {
			offset += framesToAdvance;
			mList = list;
			mOffset = offset;
			return;
		}
		framesToAdvance -= avail;
		offset = 0;
		list = list->next();
		if (!list) {
			mList = nullptr;
			mOffset = 0;
			return;
		}
	}
}

bool ZIn::fillSegment(Thread& th, int inNum, Z* outBuffer)
{
	int framesToFill = inNum;
	if (mIsConstant) {
		Z z = mConstant.f;
		for (int i = 0; i < framesToFill; ++i) outBuffer[i] = z;
		return false;
	}

	P<List> list = mList;
	int offset = mOffset;
	Z* out = outBuffer;
	while (list) {
		list->force(th);
		assert(list->mArray);
		
		int avail = (int)(list->mArray->size() - offset);
		int numToFill = std::min(framesToFill, avail);

		// copy
		Z* in = list->mArray->z() + offset;
		memcpy(out, in, numToFill * sizeof(Z));
		out += numToFill;
		framesToFill -= numToFill;
		
		if (framesToFill == 0)
			return false;
		
		list = list->next();
		offset = 0;
	}
	
	Z z = mConstant.f;
	for (int i = 0; i < framesToFill; ++i) outBuffer[i] = z;
	
	mDone = true;
	return true;
}


bool ZIn::mix(Thread& th, int& ioNum, Z* outBuffer)
{
	int framesToFill = ioNum;
	int framesFilled = 0;
	while (framesToFill) {
		int n = framesToFill;
		int astride;
		Z* a;
		if (operator()(th, n, astride, a)) {
			ioNum = framesFilled;
			return true;
		}
		for (int i = 0; i < n; ++i)	{
			outBuffer[i] += *a;
			a += astride;
		}
		framesToFill -= n;
		framesFilled += n;
		advance(n);
		outBuffer += n;
	}
	ioNum = framesFilled;
	return false;
}

class Comma : public Gen
{
	VIn _a;
	V key;
public:
	Comma(Thread& th, Arg a, Arg inKey) : Gen(th, itemTypeV, a.isFinite()), _a(a), key(inKey) {} 
	
	virtual const char* TypeName() const override { return "Comma"; }

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
				for (int i = 0; i < n; ++i) {
					out[i] = a->comma(th, key);
					a += astride;
				}
				_a.advance(n);
			}
			framesToFill -= n;
			out += n;
		}
		produce(framesToFill);
		
	}
	
};

class Dot : public Gen
{
	VIn _a;
	V key;
	V defaultValue;
public:
	Dot(Thread& th, Arg a, Arg inKey, Arg inDefaultValue)
					: Gen(th, itemTypeV, a.isFinite()), _a(a), key(inKey) {}
	
	virtual const char* TypeName() const override { return "Dot"; }

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
					for (int i = 0; i < n; ++i) {
						V v = defaultValue;
						a->dot(th, key, v);
						out[i] = v;
						a += astride;
					}
				_a.advance(n);
			}
			framesToFill -= n;
			out += n;
		}
		produce(framesToFill);
		
	}
	
};


V List::comma(Thread& th, Arg key)
{
	return new List(new Comma(th, this, key));
}

bool List::dot(Thread& th, Arg key, V& ioValue)
{
	ioValue = new List(new Dot(th, this, key, ioValue));
	return true;
}

bool List::Equals(Thread& th, Arg v)
{
	if (v.Identical(this)) return true;
	if (!v.isList()) return false;
	List* that = (List*)v.o();
	if (!isFinite())
		indefiniteOp("", "equals : a");
	if (!that->isFinite())
		indefiniteOp("", "equals : b");
	
	if (elemType != that->elemType) return false;
	
	if (isVList()) {
		VIn _a(this);
		VIn _b(that);
		
		V *a, *b;
		int astride, bstride;

		while(1) {
			int n = kDefaultVBlockSize;
			bool aend = _a(th, n,astride, a);
			bool bend = _b(th, n,bstride, b);
                        
			if (aend != bend) return false;
			if (aend && bend) return true;
			
			for (int i = 0; i < n; ++i) {
				if (!a->Equals(th, *b)) return false;
				a += astride;
				b += bstride;
			}
			
			_a.advance(n);
			_b.advance(n);
		}	
		
	} else {
		ZIn _a(this);
		ZIn _b(that);
		
		Z *a, *b;
		int astride, bstride;

		while(1) {
			int n = th.rate.blockSize;
			bool aend = _a(th, n,astride, a);
			bool bend = _b(th, n,bstride, b);
			if (aend != bend) return false;
			if (aend && bend) return true;
			
			for (int i = 0; i < n; ++i) {
				if (*a != *b) return false;
				a += astride;
				b += bstride;
			}
			
			_a.advance(n);
			_b.advance(n);
		}	
	}

}

List::List(int inItemType) // construct nil
	: mNext(nullptr), mGen(nullptr), mArray(new Array(inItemType, 0))
{
	elemType = inItemType;
	setFinite(true);
}

List::List(int inItemType, int64_t inCap) // construct nil
	: mNext(nullptr), mGen(nullptr), mArray(new Array(inItemType, inCap))
{
	elemType = inItemType;
	setFinite(true);
}



List::List(P<Gen> const& inGen) 
	: mNext(nullptr), mGen(inGen), mArray(0)
{
	elemType = inGen->elemType;
	setFinite(inGen->isFinite());
	inGen->setOut(this);
}

List::List(P<Array> const& inArray) 
	: mNext(nullptr), mGen(nullptr), mArray(inArray)
{
	elemType = inArray->elemType;
	setFinite(true);
}

List::List(P<Array> const& inArray, P<List> const& inNext) 
	: mNext(inNext), mGen(0), mArray(inArray)
{
	assert(!mNext || mArray->elemType == mNext->elemType);
	elemType = inArray->elemType;
	setFinite(!mNext || mNext->isFinite());
}

List::~List()
{
	// free as much tail as possible at once in order to prevent stack overflow.
	P<List> list = mNext;
	mNext = nullptr;
	while (list) {
		if (list->getRefcount() > 1) break;
		P<List> next = list->mNext;
		list->mNext = nullptr;
		list = next;
	}
}

int64_t List::fillz(Thread& th, int64_t n, Z* z)
{
	int64_t k = 0;
	P<List> list = this;
	while(list && k < n) {
		list->force(th);
		
		int64_t m = std::min((n-k), list->mArray->size());
		for (int64_t i = 0; i < m; ++i) {
			z[k++] = list->mArray->_atz(i);
		}
		list = list->mNext;
	}
	return k;
}


List* List::pack(Thread& th)
{
    force(th);
	if (isPacked())
		return this;
		
	int cap = 0;
	P<List> list = this;
	while(list) {
		list->force(th);
			
		cap += list->mArray->size();	
		
		list = list->mNext;
	}

	P<Array> a = new Array(elemType, cap);
	
	list = this;
	while(list) {
		a->addAll(list->mArray());
		list = list->mNext;
	}
	
	return new List(a);
}

List* List::packz(Thread& th)
{
    force(th);
	if (isPacked() && isZ())
		return this;
		
	int cap = 0;
	P<List> list = this;
	while(list) {
		list->force(th);
			
		cap += list->mArray->size();	
		
		list = list->mNext;
	}

	P<Array> a = new Array(itemTypeZ, cap);
	
	list = this;
	while(list) {
		a->addAll(list->mArray());
		list = list->mNext;
	}
	
	return new List(a);
}

List* List::pack(Thread& th, int limit)
{
    force(th);
	if (isPacked())
		return this;
		
	int cap = 0;
	P<List> list = this;
	while(list) {
		list->force(th);
			
		cap += list->mArray->size();	
				
		if (cap > limit) return nullptr;
		
		list = list->mNext;
	}

	P<Array> a = new Array(elemType, cap);
	
	list = this;
	while(list) {
		a->addAll(list->mArray());
		list = list->mNext;
	}
	
	return new List(a);
}

List* List::packSome(Thread& th, int64_t& limit)
{
    force(th);
	if (isPacked()) {
		limit = std::min(limit, length(th));
		return this;
	}
		
	P<List> list = this;
	int64_t count = 0;
	while(list && count < limit) {
		list->force(th);
			
		count += list->mArray->size();	
						
		list = list->mNext;
	}

	P<Array> a = new Array(elemType, count);
	
	list = this;
	count = 0;
	while(list && count < limit) {
		a->addAll(list->mArray());
		count += list->mArray->size();	
		list = list->mNext;
	}
	limit = std::min(limit, count);
	
	return new List(a);
}

void List::forceAll(Thread& th)
{
	List* list = this;
	while(list) {
		list->force(th);
		list = list->nextp();
	}
}

V V::msgSend(Thread& th, Arg receiver)
{
	if (!o) {
		return *this;
	} else {
		return o->msgSend(th, receiver);
	}
}

V Prim::msgSend(Thread& th, Arg receiver)
{
	SaveStack ss(th, Takes());
	th.push(receiver);
	apply(th);
	if (th.stackDepth() >= 1) {
		return th.pop();
	} else {
		return V(0.);
	}
}

V Fun::msgSend(Thread& th, Arg receiver)
{
	th.push(receiver);
	SaveStack ss(th, NumArgs());
	apply(th);
	if (th.stackDepth() >= 1) {
		return th.pop();
	} else {
		return V(0.);
	}
}
	
TableMap::TableMap(size_t inSize)
	: mSize(inSize)
{
	if (inSize == 0) {
		mMask = 0;
		mIndices = nullptr;
		mKeys = nullptr;
	} else {
		size_t n = 2*NEXTPOWEROFTWO((int64_t)mSize);
		mMask = n-1;
		mIndices = new size_t[n]();
		mKeys = new V[mSize];
	}
}

TableMap::TableMap(Arg inKey)
	: mSize(1)
{
	mMask = 1;
	mIndices = new size_t[2]();
	mKeys = new V[mSize];
	
	mKeys[0] = inKey;
	mIndices[inKey.Hash() & 1] = 1;
}


TableMap::~TableMap()
{
	delete [] mKeys;
	delete [] mIndices;
}

bool TableMap::getIndex(Arg inKey, int64_t inKeyHash, size_t& outIndex)
{
	size_t mask = mMask;
	size_t i = inKeyHash & mask;
	size_t* indices = mIndices;
	V* keys = mKeys;
	while(1) {
		size_t index = indices[i];
		if (index == 0)
			return false;
		size_t index2 = index - 1;
		if (inKey.Identical(keys[index2])) {
			outIndex = index2;
			return true;
		}
		i = (i + 1) & mask;
	}		
}

void TableMap::put(size_t inIndex, Arg inKey, int64_t inKeyHash)
{
	mKeys[inIndex] = inKey;
	
	size_t mask = mMask;
	size_t i = inKeyHash & mask;
	size_t* indices = mIndices;
	
	while(1) {
		if (indices[i] == 0) {
			indices[i] = inIndex + 1;
			return;
		}
		i = (i + 1) & mask;
	}		
}

Table::Table(P<TableMap> const& inMap)
	: mMap(inMap), mValues(new V[mMap->mSize])
{
}

Table::~Table()
{
	delete [] mValues;
}

bool Table::Equals(Thread& th, Arg v)
{
	if (v.Identical(this)) return true;
	if (!v.isTable()) return false;
	if (this == v.o()) return true;
	Table* that = (Table*)v.o();
	size_t size = mMap->mSize;
	if (size != that->mMap->mSize)
		return false;
	for (size_t i = 0; i < size; ++i) {
		V key = mMap->mKeys[i];
		V thatValue;
		if (!that->getWithHash(th, key, key.Hash(), thatValue))
			return false;
		if (!mValues[i].Equals(th, thatValue))
			return false;
	}
    return true;
}

bool Table::getWithHash(Thread& th, Arg key, int64_t hash, V& value) const
{
	size_t index;
	if (!mMap->getIndex(key, hash, index))
		return false;
	value = mValues[index];
	return true;
}

void Table::put(size_t inIndex, Arg inValue)
{
	mValues[inIndex] = inValue;
}

void Table::print(Thread& th, std::string& out, int depth)
{
	for (size_t i = 0; i < mMap->mSize; ++i) {
		if (i == 0) zprintf(out, ":");
		else zprintf(out, "  :");
		mMap->mKeys[i].print(th, out, depth+1);
		zprintf(out, " ");
		mValues[i].print(th, out, depth+1);
	}
}

void TableMap::print(Thread& th, std::string& out, int depth)
{
	zprintf(out, "{");
	for (size_t i = 0; i < mSize; ++i) {
		if (i == 0) zprintf(out, ":");
		else zprintf(out, "  :");
		mKeys[i].print(th, out, depth+1);
	}
	zprintf(out, "}");
}


static P<List> chase_z(Thread& th, P<List> list, int64_t n)
{
	if (n <= 0) return list;
	
	while (list && n > 0) {
		list->force(th);

		Array* a = list->mArray();
		int64_t asize = a->size();
		if (asize > n) {
			int64_t remain = asize - n;
			Array* a2 = new Array(list->elemType, remain);
			a2->setSize(remain);

			memcpy(a2->z(), a->z() + n, remain * a->elemSize());

			return new List(a2, list->next());
		}
		n -= asize;
		list = list->next();
	}
	
	if (!list) {
		list = vm._nilz;
	}
	
	return list;
}

static P<List> chase_v(Thread& th, P<List> list, int64_t n)
{
	if (n <= 0) return list;
	
	if (!list->isFinite()) {
		indefiniteOp("chase : list", "");
	}
	
	int64_t length = list->length(th);
	
	P<List> result = new List(itemTypeV, length);
	P<Array> array = result->mArray;
	V* out = array->v();
	
	int64_t i = 0;
	
	while (list) {
		list->force(th);

		Array* a = list->mArray();
		int64_t asize = a->size();
		V* in = a->v();
		
		for (int64_t j = 0; j < asize; ++j, ++i) {
			out[i] = in[j].chase(th, n);
		}

		list = list->next();
	}
	
	return list;
}

V List::chase(Thread& th, int64_t n)
{
	if (isVList()) {
		return chase_v(th, this, n);
	} else {
		return chase_z(th, this, n);
	}
}

P<Form> Form::chaseForm(Thread& th, int64_t n)
{
	P<Form> nextForm = mNextForm() ? mNextForm->chaseForm(th, n) : nullptr;
	return new Form(mTable->chaseTable(th, n), nextForm);
}

P<Table> Table::chaseTable(Thread& th, int64_t n)
{
	P<Table> result = new Table(mMap);
	
	size_t size = mMap->mSize;

	for (size_t i = 0; i < size; ++i) {
		result->mValues[i] = mValues[i].chase(th, n);
	}
	
	return result;
}



void Plug::setPlug(Arg inV)
{
	SpinLocker lock(mSpinLock);
	in.set(inV);
	++mChangeCount;
}

void Plug::setPlug(const VIn& inVIn, int inChangeCount)
{
	SpinLocker lock(mSpinLock);
	if (inChangeCount == mChangeCount) {
		in = inVIn;
	}
}

void Plug::getPlug(VIn& outVIn, int& outChangeCount)
{
	SpinLocker lock(mSpinLock);
	outVIn = in;
	outChangeCount = mChangeCount;
}


void ZPlug::setPlug(Arg inV) {
	SpinLocker lock(mSpinLock);
	in.set(inV);
	++mChangeCount;
}

void ZPlug::setPlug(const ZIn& inZIn, int inChangeCount) {
	SpinLocker lock(mSpinLock);
	if (inChangeCount == mChangeCount) {
		in = inZIn;
	}
}

void ZPlug::getPlug(ZIn& outZIn, int& outChangeCount) {
	SpinLocker lock(mSpinLock);
	outZIn = in;
	outChangeCount = mChangeCount;
}
