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
#include "elapsedTime.hpp"
#include <cmath>
#include <float.h>
#include <vector>
#include "MultichannelExpansion.hpp"
#include "UGen.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma mark RANDOM STREAMS


template <typename T>
void swapifgt(T& a, T& b) { 
	if (a > b) {
		T t = a;
		a = b;
		b = t;
	}
}


struct URand : ZeroInputGen<URand>
{	
    RGen r;
    
	URand(Thread& th) : ZeroInputGen<URand>(th, false) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "URand"; }
    
	void calc(int n, V* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = r.drand();
		}
	}
};

struct URandz : ZeroInputUGen<URandz>
{	
    RGen r;
    
	URandz(Thread& th) : ZeroInputUGen<URandz>(th, false) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "URandz"; }
    
	void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = r.drand();
		}
	}
};


struct NURand : NZeroInputGen<NURand>
{	
    RGen r;
    
	NURand(Thread& th, int64_t n) : NZeroInputGen<NURand>(th, n) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NURand"; }
    
	void calc(int n, V* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = r.drand();
		}
	}
};

struct NURandz : NZeroInputUGen<NURandz>
{	
    RGen r;
    
	NURandz(Thread& th, int64_t n) : NZeroInputUGen<NURandz>(th, n) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NURandz"; }
    
	void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = r.drand();
		}
	}
};

struct BRand : ZeroInputGen<BRand>
{	
    RGen r;
    
	BRand(Thread& th) : ZeroInputGen<BRand>(th, false) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "BRand"; }
    
	void calc(int n, V* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = r.drand2();
		}
	}
};

struct BRandz : ZeroInputUGen<BRandz>
{	
    RGen r;
    
	BRandz(Thread& th) : ZeroInputUGen<BRandz>(th, false) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "BRandz"; }
    
	void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = r.drand2();
		}
	}
};


struct NBRand : NZeroInputGen<NBRand>
{	
    RGen r;
    
	NBRand(Thread& th, int64_t n) : NZeroInputGen<NBRand>(th, n) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NBRand"; }
    
	void calc(int n, V* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = r.drand2();
		}
	}
};

struct NBRandz : NZeroInputUGen<NBRandz>
{	
    RGen r;
    
	NBRandz(Thread& th, int64_t n) : NZeroInputUGen<NBRandz>(th, n) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NBRandz"; }
    
	void calc(int n, Z* out) 
	{
		for (int i = 0; i < n; ++i) {
			out[i] = r.drand2();
		}
	}
};


struct Rand : TwoInputGen<Rand>
{	
    RGen r;
    
	Rand(Thread& th,  Arg a, Arg b) : TwoInputGen<Rand>(th, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "Rand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = aa->asFloat();
			Z b = bb->asFloat(); 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = a + (b - a) * r.drand();
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				Z b = bb->asFloat(); bb += bStride; 
				swapifgt(a, b);
				out[i] = a + (b - a) * r.drand();
			}
		}
	}
};

struct Randz : TwoInputUGen<Randz>
{	
    RGen r;
    
	Randz(Thread& th,  Arg a, Arg b) : TwoInputUGen<Randz>(th, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "Randz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = *aa;
			Z b = *bb; 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = a + (b - a) * r.drand();
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				Z b = *bb; bb += bStride; 
				swapifgt(a, b);
				out[i] = a + (b - a) * r.drand();
			}
		}
	}
};


struct NRand : NTwoInputGen<NRand>
{	
    RGen r;
    
	NRand(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputGen<NRand>(th, n, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = aa->asFloat();
			Z b = bb->asFloat(); 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = a + (b - a) * r.drand();
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				Z b = bb->asFloat(); bb += bStride; 
				swapifgt(a, b);
				out[i] = a + (b - a) * r.drand();
			}
		}
	}
};

struct NRandz : NTwoInputUGen<NRandz>
{	
    RGen r;
    
	NRandz(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputUGen<NRandz>(th, n, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = *aa;
			Z b = *bb; 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = a + (b - a) * r.drand();
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				Z b = *bb; bb += bStride; 
				swapifgt(a, b);
				out[i] = a + (b - a) * r.drand();
			}
		}
	}
};


static void urands_(Thread& th, Prim* prim)
{
	Gen* g = new URand(th);
	th.push(new List(g));
}

static void urandz_(Thread& th, Prim* prim)
{
	Gen* g = new URandz(th);
	th.push(new List(g));
}

static void brands_(Thread& th, Prim* prim)
{
	Gen* g = new BRand(th);
	th.push(new List(g));
}

static void brandz_(Thread& th, Prim* prim)
{
	Gen* g = new BRandz(th);
	th.push(new List(g));
}

static void nurands_(Thread& th, Prim* prim)
{
	int64_t n = th.popInt("nurands : n");
	Gen* g = new NURand(th, n);
	th.push(new List(g));
}

static void nurandz_(Thread& th, Prim* prim)
{
	int64_t n = th.popInt("nurandz : n");
	Gen* g = new NURandz(th, n);
	th.push(new List(g));
}

static void nbrands_(Thread& th, Prim* prim)
{
	int64_t n = th.popInt("nbrands : n");
	Gen* g = new NBRand(th, n);
	th.push(new List(g));
}

static void nbrandz_(Thread& th, Prim* prim)
{
	int64_t n = th.popInt("nbrandz : n");
	Gen* g = new NBRandz(th, n);
	th.push(new List(g));
}

static void rands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	
	Gen* g = new Rand(th, a, b);
	th.push(new List(g));
}

static void randz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("randz : hi");
	V a = th.popZIn("randz : lo");
	
	Gen* g = new Randz(th, a, b);
	th.push(new List(g));
}

static void nrands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	int64_t n = th.popInt("nrands : n");
	
	Gen* g = new NRand(th, n, a, b);
	th.push(new List(g));
}

static void nrandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("nrandz : hi");
	V a = th.popZIn("nrandz : lo");
	int64_t n = th.popInt("nrandz : n");
	
	Gen* g = new NRandz(th, n, a, b);
	th.push(new List(g));
}

static void urand_(Thread& th, Prim* prim)
{
	Z z = th.rgen.drand();
	th.push(z);
}

static void brand_(Thread& th, Prim* prim)
{
	Z z = th.rgen.drand2();
	th.push(z);
}


static void newseed_(Thread& th, Prim* prim)
{
	V v;
	v.i = timeseed();
	th.push(v);
}

static void setseed_(Thread& th, Prim* prim)
{
	V v = th.pop();
	if (!v.isReal()) wrongType("setseed : seed", "Float", v);
	th.rgen.init(v.i);
}

static void rand_(Thread& th, Prim* prim)
{
	Z b = th.popFloat("rand : hi");
	Z a = th.popFloat("rand : lo");
	
	swapifgt(a, b);
	Z z = th.rgen.rand(a, b);
	th.push(z);
}

struct Coin : OneInputGen<Coin>
{	
    RGen r;
    
	Coin(Thread& th,  Arg a) : OneInputGen<Coin>(th, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "Coin"; }
    
	void calc(int n, V* out, V* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = aa->asFloat();
			for (int i = 0; i < n; ++i) {
				out[i] = r.coin(a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				out[i] = r.coin(a);
			}
		}
	}
};

struct Coinz : OneInputUGen<Coinz>
{	
    RGen r;
    
	Coinz(Thread& th,  Arg a) : OneInputUGen<Coinz>(th, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "Coinz"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			for (int i = 0; i < n; ++i) {
				out[i] = r.coin(a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				out[i] = r.coin(a);
			}
		}
	}
};

struct NCoin : NOneInputGen<NCoin>
{	
    RGen r;
    
	NCoin(Thread& th, int64_t n, Arg a) : NOneInputGen<NCoin>(th, n, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NCoin"; }
    
	void calc(int n, V* out, V* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = aa->asFloat();
			for (int i = 0; i < n; ++i) {
				out[i] = r.coin(a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				out[i] = r.coin(a);
			}
		}
	}
};

struct NCoinz : NOneInputUGen<NCoinz>
{	
    RGen r;
    
	NCoinz(Thread& th, int64_t n, Arg a) : NOneInputUGen<NCoinz>(th, n, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NCoinz"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			for (int i = 0; i < n; ++i) {
				out[i] = r.coin(a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				out[i] = r.coin(a);
			}
		}
	}
};


static void coin_(Thread& th, Prim* prim)
{
	Z p = th.popFloat("coin : p");
	
	Z z = th.rgen.coin(p);
	th.push(z);
}

static void coins_(Thread& th, Prim* prim)
{
	V a = th.pop();
	
	Gen* g = new Coin(th, a);
	th.push(new List(g));
}

static void coinz_(Thread& th, Prim* prim)
{
	V a = th.pop();
	
	Gen* g = new Coinz(th, a);
	th.push(new List(g));
}

static void ncoins_(Thread& th, Prim* prim)
{
	V a = th.pop();
	int64_t n = th.popInt("ncoins : n");
	
	Gen* g = new NCoin(th, n, a);
	th.push(new List(g));
}

static void ncoinz_(Thread& th, Prim* prim)
{
	V a = th.pop();
	int64_t n = th.popInt("ncoinz : n");
	
	Gen* g = new NCoinz(th, n, a);
	th.push(new List(g));
}


struct IRand : TwoInputGen<IRand>
{	
    RGen r;
    
	IRand(Thread& th,  Arg a, Arg b) : TwoInputGen<IRand>(th, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "IRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int32_t a = (int32_t)aa->asInt();
			int32_t b = (int32_t)bb->asInt();
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = (Z)r.irand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int32_t a = (int32_t)aa->asInt(); aa += astride;
				int32_t b = (int32_t)bb->asInt(); bb += bstride;
				swapifgt(a, b);
				out[i] = (Z)r.irand(a, b);
			}
		}
	}
};

struct IRandz : TwoInputUGen<IRandz>
{	
    RGen r;
    
	IRandz(Thread& th,  Arg a, Arg b) : TwoInputUGen<IRandz>(th, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "IRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int32_t a = (int32_t)*aa;
			int32_t b = (int32_t)*bb;
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = (Z)r.irand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int32_t a = (int32_t)*aa; aa += astride;
				int32_t b = (int32_t)*bb; bb += bstride;
				swapifgt(a, b);
				out[i] = (Z)r.irand(a, b);
			}
		}
	}
};


struct NIRand : NTwoInputGen<NIRand>
{	
    RGen r;
    
	NIRand(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputGen<NIRand>(th, n, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NIRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int32_t a = (int32_t)aa->asInt();
			int32_t b = (int32_t)bb->asInt();
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = (Z)r.irand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int32_t a = (int32_t)aa->asInt(); aa += astride;
				int32_t b = (int32_t)bb->asInt(); bb += bstride;
				swapifgt(a, b);
				out[i] = (Z)r.irand(a, b);
			}
		}
	}
};

struct NIRandz : NTwoInputUGen<NIRandz>
{	
    RGen r;
    
	NIRandz(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputUGen<NIRandz>(th, n, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NIRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int32_t a = (int32_t)*aa;
			int32_t b = (int32_t)*bb;
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = (Z)r.irand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int32_t a = (int32_t)*aa; aa += astride;
				int32_t b = (int32_t)*bb; bb += bstride;
				swapifgt(a, b);
				out[i] = (Z)r.irand(a, b);
			}
		}
	}
};


static void irands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	
	Gen* g = new IRand(th, a, b);
	th.push(new List(g));
}

static void irandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("irandz : hi");
	V a = th.popZIn("irandz : lo");
	
	Gen* g = new IRandz(th, a, b);
	th.push(new List(g));
}

static void nirands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	int64_t n = th.popInt("nirands : n");
	
	Gen* g = new NIRand(th, n, a, b);
	th.push(new List(g));
}

static void nirandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("nirandz : hi");
	V a = th.popZIn("nirandz : lo");
	int64_t n = th.popInt("nirandz : n");
	
	Gen* g = new NIRandz(th, n, a, b);
	th.push(new List(g));
}

static void irand_(Thread& th, Prim* prim)
{
	int64_t b = th.popInt("irand : hi");
	int64_t a = th.popInt("irand : lo");
	RGen& r = th.rgen;
	
	swapifgt(a, b);
	Z z = (Z)r.irand(a, b);
	th.push(z);
}


struct ExcRand : TwoInputGen<ExcRand>
{	
    RGen r;
	int64_t prev;
    
	ExcRand(Thread& th,  Arg a, Arg b) : TwoInputGen<ExcRand>(th, a, b), prev(INT32_MIN)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "ExcRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int64_t a = (int64_t)aa->asInt();
			int64_t b = (int64_t)bb->asInt();
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				int64_t x = r.irand(a, b);
				if (x == prev) x = b;
				prev = x;
				out[i] = x;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int64_t a = (int64_t)aa->asInt(); aa += astride;
				int64_t b = (int64_t)bb->asInt(); bb += bstride;
				swapifgt(a, b);
                
				int64_t x = r.irand(a, b);
				if (x == prev) x = b;
				prev = x;
				out[i] = (Z)x;
			}
		}
	}
};

struct ExcRandz : TwoInputUGen<ExcRandz>
{	
    RGen r;
	int64_t prev;
    
	ExcRandz(Thread& th,  Arg a, Arg b) : TwoInputUGen<ExcRandz>(th, a, b), prev(INT32_MIN)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "ExcRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int64_t a = (int64_t)*aa;
			int64_t b = (int64_t)*bb;
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				int64_t x = r.irand(a, b);
				if (x == prev) x = b;
				prev = x;
				out[i] = (Z)x;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int64_t a = (int64_t)*aa; aa += astride;
				int64_t b = (int64_t)*bb; bb += bstride;
				swapifgt(a, b);
				
				int64_t x = r.irand(a, b);
				if (x == prev) x = b;
				prev = x;
				out[i] = (Z)x;
			}
		}
	}
};


struct NExcRand : NTwoInputGen<NExcRand>
{	
    RGen r;
 	int64_t prev;
    
	NExcRand(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputGen<NExcRand>(th, n, a, b), prev(INT32_MIN)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NExcRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int64_t a = (int64_t)aa->asInt();
			int64_t b = (int64_t)bb->asInt();
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				int64_t x = r.irand(a, b);
				if (x == prev) x = b;
				prev = x;
				out[i] = x;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int64_t a = (int64_t)aa->asInt(); aa += astride;
				int64_t b = (int64_t)bb->asInt(); bb += bstride;
				swapifgt(a, b);
                
				int64_t x = r.irand(a, b);
				if (x == prev) x = b;
				prev = x;
				out[i] = (Z)x;
			}
		}
	}
};

struct NExcRandz : NTwoInputUGen<NExcRandz>
{	
    RGen r;
	int64_t prev;
    
	NExcRandz(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputUGen<NExcRandz>(th, n, a, b), prev(INT32_MIN)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NIRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int64_t a = (int64_t)*aa;
			int64_t b = (int64_t)*bb;
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = (Z)r.irand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int64_t a = (int64_t)*aa; aa += astride;
				int64_t b = (int64_t)*bb; bb += bstride;
				swapifgt(a, b);
				out[i] = (Z)r.irand(a, b);
			}
		}
	}
};


static void eprands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	
	Gen* g = new ExcRand(th, a, b);
	th.push(new List(g));
}

static void eprandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("eprandz : hi");
	V a = th.popZIn("eprandz : lo");
	
	Gen* g = new ExcRandz(th, a, b);
	th.push(new List(g));
}

static void neprands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	int64_t n = th.popInt("neprands : n");
	
	Gen* g = new NExcRand(th, n, a, b);
	th.push(new List(g));
}

static void neprandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("eprands : hi");
	V a = th.popZIn("eprands : lo");
	int64_t n = th.popInt("neprandz : n");
	
	Gen* g = new NExcRandz(th, n, a, b);
	th.push(new List(g));
}

struct ExpRand : TwoInputGen<ExpRand>
{	
    RGen r;
    
	ExpRand(Thread& th,  Arg a, Arg b) : TwoInputGen<ExpRand>(th, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "ExpRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = aa->asFloat();
			Z b = bb->asFloat(); 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = r.xrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				Z b = bb->asFloat(); bb += bStride; 
				swapifgt(a, b);
				out[i] = r.xrand(a, b);
			}
		}
	}
};

struct ExpRandz : TwoInputUGen<ExpRandz>
{	
    RGen r;
    
	ExpRandz(Thread& th,  Arg a, Arg b) : TwoInputUGen<ExpRandz>(th, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "ExpRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = *aa;
			Z b = *bb; 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = r.xrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				Z b = *bb; bb += bStride; 
				swapifgt(a, b);
				out[i] = r.xrand(a, b);
			}
		}
	}
};


struct NExpRand : NTwoInputGen<NExpRand>
{	
    RGen r;
    
	NExpRand(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputGen<NExpRand>(th, n, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NExpRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = aa->asFloat();
			Z b = bb->asFloat(); 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = r.xrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				Z b = bb->asFloat(); bb += bStride; 
				swapifgt(a, b);
				out[i] = r.xrand(a, b);
			}
		}
	}
};

struct NExpRandz : NTwoInputUGen<NExpRandz>
{	
    RGen r;
    
	NExpRandz(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputUGen<NExpRandz>(th, n, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NExpRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = *aa;
			Z b = *bb; 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = r.xrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				Z b = *bb; bb += bStride; 
				swapifgt(a, b);
				out[i] = r.xrand(a, b);
			}
		}
	}
};


static void xrands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	
	Gen* g = new ExpRand(th, a, b);
	th.push(new List(g));
}

static void xrandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("xrandz : hi");
	V a = th.popZIn("xrandz : lo");
	
	Gen* g = new ExpRandz(th, a, b);
	th.push(new List(g));
}

static void nxrands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	int64_t n = th.popInt("nxrands : n");
	
	Gen* g = new NExpRand(th, n, a, b);
	th.push(new List(g));
}

static void nxrandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("nxrandz : hi");
	V a = th.popZIn("nxrandz : lo");
	int64_t n = th.popInt("xrandz : n");
	
	Gen* g = new NExpRandz(th, n, a, b);
	th.push(new List(g));
}

static void xrand_(Thread& th, Prim* prim)
{
	Z b = th.popFloat("xrand : hi");
	Z a = th.popFloat("xrand : lo");
	RGen& r = th.rgen;
	
	if (b < a) { Z x; x = a; a = b; b = x; }
	Z z = r.xrand(a, b);
	th.push(z);
}

struct ILinRand : TwoInputGen<ILinRand>
{	
    RGen r;
    
	ILinRand(Thread& th,  Arg a, Arg b) : TwoInputGen<ILinRand>(th, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "ILinRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int64_t a = (int64_t)aa->asInt();
			int64_t b = (int64_t)bb->asInt();
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = (Z)r.irand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int64_t a = (int64_t)aa->asInt(); aa += astride;
				int64_t b = (int64_t)bb->asInt(); bb += bstride;
				swapifgt(a, b);
				out[i] = (Z)r.ilinrand(a, b);
			}
		}
	}
};

struct ILinRandz : TwoInputUGen<ILinRandz>
{	
    RGen r;
    
	ILinRandz(Thread& th,  Arg a, Arg b) : TwoInputUGen<ILinRandz>(th, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "ILinRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int64_t a = (int64_t)*aa;
			int64_t b = (int64_t)*bb;
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = (Z)r.ilinrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int64_t a = (int64_t)*aa; aa += astride;
				int64_t b = (int64_t)*bb; bb += bstride;
				swapifgt(a, b);
				out[i] = (Z)r.ilinrand(a, b);
			}
		}
	}
};


struct NILinRand : NTwoInputGen<NILinRand>
{	
    RGen r;
    
	NILinRand(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputGen<NILinRand>(th, n, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NILinRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int64_t a = (int64_t)aa->asInt();
			int64_t b = (int64_t)bb->asInt();
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = (Z)r.ilinrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int64_t a = (int64_t)aa->asInt(); aa += astride;
				int64_t b = (int64_t)bb->asInt(); bb += bstride;
				swapifgt(a, b);
				out[i] = (Z)r.ilinrand(a, b);
			}
		}
	}
};

struct NILinRandz : NTwoInputUGen<NILinRandz>
{	
    RGen r;
    
	NILinRandz(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputUGen<NILinRandz>(th, n, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NILinRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int astride, int bstride) 
	{
		if (astride == 0 && bstride == 0) {
			int64_t a = (int64_t)*aa;
			int64_t b = (int64_t)*bb;
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = (Z)r.ilinrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				int64_t a = (int64_t)*aa; aa += astride;
				int64_t b = (int64_t)*bb; bb += bstride;
				swapifgt(a, b);
				out[i] = (Z)r.ilinrand(a, b);
			}
		}
	}
};


static void ilinrands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	
	Gen* g = new ILinRand(th, a, b);
	th.push(new List(g));
}

static void ilinrandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("ilinrandz : hi");
	V a = th.popZIn("ilinrandz : lo");
	
	Gen* g = new ILinRandz(th, a, b);
	th.push(new List(g));
}

static void nilinrands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	int64_t n = th.popInt("nilinrands : n");
	
	Gen* g = new NILinRand(th, n, a, b);
	th.push(new List(g));
}

static void nilinrandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("nilinrandz : hi");
	V a = th.popZIn("nilinrandz : lo");
	int64_t n = th.popInt("nilinrandz : n");
	
	Gen* g = new NILinRandz(th, n, a, b);
	th.push(new List(g));
}

static void ilinrand_(Thread& th, Prim* prim)
{
	int64_t b = th.popInt("ilinrand : hi");
	int64_t a = th.popInt("ilinrand : lo");
	RGen& r = th.rgen;
	
	int64_t x;
	if (b < a) { x = a; a = b; b = x; }
	Z z = (Z)r.ilinrand(a, b);
	th.push(z);
}



struct LinRand : TwoInputGen<LinRand>
{	
    RGen r;
    
	LinRand(Thread& th,  Arg a, Arg b) : TwoInputGen<LinRand>(th, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "LinRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = aa->asFloat();
			Z b = bb->asFloat(); 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = r.linrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				Z b = bb->asFloat(); bb += bStride; 
				swapifgt(a, b);
				out[i] = r.linrand(a, b);
			}
		}
	}
};

struct LinRandz : TwoInputUGen<LinRandz>
{	
    RGen r;
    
	LinRandz(Thread& th,  Arg a, Arg b) : TwoInputUGen<LinRandz>(th, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "LinRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = *aa;
			Z b = *bb; 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = r.linrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				Z b = *bb; bb += bStride; 
				swapifgt(a, b);
				out[i] = r.linrand(a, b);
			}
		}
	}
};


struct NLinRand : NTwoInputGen<NLinRand>
{	
    RGen r;
    
	NLinRand(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputGen<NLinRand>(th, n, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NLinRand"; }
    
	void calc(int n, V* out, V* aa, V* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = aa->asFloat();
			Z b = bb->asFloat(); 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = r.linrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				Z b = bb->asFloat(); bb += bStride; 
				swapifgt(a, b);
				out[i] = r.linrand(a, b);
			}
		}
	}
};

struct NLinRandz : NTwoInputUGen<NLinRandz>
{	
    RGen r;
    
	NLinRandz(Thread& th, int64_t n, Arg a, Arg b) : NTwoInputUGen<NLinRandz>(th, n, a, b) 
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NLinRandz"; }
    
	void calc(int n, Z* out, Z* aa, Z* bb, int aStride, int bStride) 
	{
		if (aStride == 0 && bStride == 0) {
			Z a = *aa;
			Z b = *bb; 
			swapifgt(a, b);
			for (int i = 0; i < n; ++i) {
				out[i] = r.linrand(a, b);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				Z b = *bb; bb += bStride; 
				swapifgt(a, b);
				out[i] = r.linrand(a, b);
			}
		}
	}
};


static void linrands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	
	Gen* g = new LinRand(th, a, b);
	th.push(new List(g));
}

static void linrandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("linrandz : hi");
	V a = th.popZIn("linrandz : lo");
	
	Gen* g = new LinRandz(th, a, b);
	th.push(new List(g));
}

static void nlinrands_(Thread& th, Prim* prim)
{
	V b = th.pop();
	V a = th.pop();
	int64_t n = th.popInt("linrandz : n");
	
	Gen* g = new NLinRand(th, n, a, b);
	th.push(new List(g));
}

static void nlinrandz_(Thread& th, Prim* prim)
{
	V b = th.popZIn("linrandz : hi");
	V a = th.popZIn("linrandz : lo");
	int64_t n = th.popInt("randz : n");
	
	Gen* g = new NLinRandz(th, n, a, b);
	th.push(new List(g));
}

static void linrand_(Thread& th, Prim* prim)
{
	Z b = th.popFloat("linrand : hi");
	Z a = th.popFloat("linrand : lo");
	RGen& r = th.rgen;
	
	if (b < a) { Z x; x = a; a = b; b = x; }
	Z z = r.linrand(a, b);
	th.push(z);
}


struct Rand2 : OneInputGen<Rand2>
{	
    RGen r;
    
	Rand2(Thread& th,  Arg a) : OneInputGen<Rand2>(th, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "Rand2"; }
    
	void calc(int n, V* out, V* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = aa->asFloat();
			Z a2 = 2. * a;
			for (int i = 0; i < n; ++i) {
				out[i] = a2 * r.drand() - a;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				out[i] = 2. * a * r.drand() - a;
			}
		}
	}
};

struct Rand2z : OneInputUGen<Rand2z>
{	
    RGen r;
    
	Rand2z(Thread& th,  Arg a) : OneInputUGen<Rand2z>(th, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "Rand2z"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			Z a2 = 2. * a;
			for (int i = 0; i < n; ++i) {
				out[i] = a2 * r.drand() - a;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				out[i] = 2. * a * r.drand() - a;
			}
		}
	}
};

struct XorNoise1 : OneInputUGen<XorNoise1>
{	
    uint64_t x = 0xA40203C12F2AD936LL;
	
	XorNoise1(Thread& th,  Arg a) : OneInputUGen<XorNoise1>(th, a)
	{
	}
	
	virtual const char* TypeName() const override { return "XorNoise1"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			for (int i = 0; i < n; ++i) {
				x = xorshift64star(x);
				out[i] = itof2(x,a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				x = xorshift64star(x);
				out[i] = itof2(x,a);
			}
		}
	}
};


struct XorNoise2 : OneInputUGen<XorNoise2>
{	
    uint64_t s[2] = { 0xA40203C12F2AD936LL, 0x9E390BD16B74D6D3LL };
	
	XorNoise2(Thread& th,  Arg a) : OneInputUGen<XorNoise2>(th, a)
	{
	}
	
	virtual const char* TypeName() const override { return "XorNoise2"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			for (int i = 0; i < n; ++i) {
				uint64_t x = xorshift128plus(s);
				out[i] = itof2(x,a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				uint64_t x = xorshift128plus(s);
				out[i] = itof2(x,a);
			}
		}
	}
};

inline uint32_t raprng(uint64_t i, uint64_t seed)
{
// http://cessu.blogspot.com/2008/11/random-access-pseudo-random-numbers.html
  uint64_t r = (2857720171ULL * ((uint32_t) i)) ^ 0x1EF57D8A7B344E7BULL;
  r ^= r >> 29;
  r += r << 16;
  r ^= r >> 21;
  r += r >> 32;
  r = (2857720171ULL * ((uint32_t) (i ^ r))) ^ (0xD9EA571C8AF880B6ULL + seed);
  r ^= r >> 29;
  r += r << 16;
  r ^= r >> 21;
  return uint32_t(r + (r >> 32));
}


struct RandomAccessNoise : OneInputUGen<RandomAccessNoise>
{
    uint64_t seed = 0xA40203C12F2AD936LL;
	uint64_t k = 0;
	
	RandomAccessNoise(Thread& th,  Arg a) : OneInputUGen<RandomAccessNoise>(th, a)
	{
	}
	
	virtual const char* TypeName() const override { return "RandomAccessNoise"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			for (int i = 0; i < n; ++i) {
				uint32_t x = raprng(k++, seed);
				out[i] = itof2(x,a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				uint32_t x = raprng(k++, seed);
				out[i] = itof2(x,a);
			}
		}
	}
};

inline uint64_t hash64shift(uint64_t key)
{
  key = (~key) + (key << 21); // key = (key << 21) - key - 1;
  key = key ^ (key >> 24);
  key = (key + (key << 3)) + (key << 8); // key * 265
  key = key ^ (key >> 14);
  key = (key + (key << 2)) + (key << 4); // key * 21
  key = key ^ (key >> 28);
  key = key + (key << 31);
  return key;
}

struct WangNoise : OneInputUGen<WangNoise>
{	
	uint64_t k = 0xA40203C12F2AD936LL;
	
	WangNoise(Thread& th,  Arg a) : OneInputUGen<WangNoise>(th, a)
	{
	}
	
	virtual const char* TypeName() const override { return "WangNoise"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			for (int i = 0; i < n; ++i) {
				uint32_t x = (uint32_t)hash64shift(k++);
				out[i] = itof2(x,a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				uint32_t x = (uint32_t)hash64shift(k++);
				out[i] = itof2(x,a);
			}
		}
	}
};

inline uint64_t Hash128to64(uint64_t x, uint64_t y) {
	const uint64_t kMul = 0x9ddfea08eb382d69ULL;
	uint64_t a = (x ^ y) * kMul;
	a ^= (a >> 47);
	uint64_t b = (y ^ a) * kMul;
	b ^= (b >> 47);
	b *= kMul;
	return b;
}


struct CityNoise : OneInputUGen<CityNoise>
{	
	uint64_t k = 0xA40203C12F2AD936LL;
	
	CityNoise(Thread& th,  Arg a) : OneInputUGen<CityNoise>(th, a)
	{
	}
	
	virtual const char* TypeName() const override { return "CityNoise"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			for (int i = 0; i < n; ++i) {
				uint32_t x = (uint32_t)Hash128to64(k++, 0x1EF57D8A7B344E7BULL);
				out[i] = itof2(x,a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				uint32_t x = (uint32_t)Hash128to64(k++, 0x1EF57D8A7B344E7BULL);
				out[i] = itof2(x,a);
			}
		}
	}
};

struct Violet : OneInputUGen<Violet>
{	
    RGen r;
	Z prev;
    
	Violet(Thread& th,  Arg a) : OneInputUGen<Violet>(th, a), prev(0.)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "Violet"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			Z a2 = .5 * a;
			for (int i = 0; i < n; ++i) {
				Z x = a * r.drand() - a2;
				out[i] = x - prev;
				prev = x;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				Z x = a * r.drand() - .5 * a;
				out[i] = x - prev;
				prev = x;
			}
		}
	}
};

struct NRand2 : NOneInputGen<NRand2>
{	
    RGen r;
    
	NRand2(Thread& th, int64_t n, Arg a) : NOneInputGen<NRand2>(th, n, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NRand2"; }
    
	void calc(int n, V* out, V* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = aa->asFloat();
			Z a2 = 2. * a;
			for (int i = 0; i < n; ++i) {
				out[i] = a2 * r.drand() - a;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				out[i] = 2. * a * r.drand() - a;
			}
		}
	}
};

struct NRand2z : NOneInputUGen<NRand2z>
{	
    RGen r;
    
	NRand2z(Thread& th, int64_t n, Arg a) : NOneInputUGen<NRand2z>(th, n, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NRand2z"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			Z a2 = 2. * a;
			for (int i = 0; i < n; ++i) {
				out[i] = a2 * r.drand() - a;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				out[i] = 2. * a * r.drand() - a;
			}
		}
	}
};


static void rand2s_(Thread& th, Prim* prim)
{
	V a = th.pop();
	
	Gen* g = new Rand2(th, a);
	th.push(new List(g));
}

static void rand2z_(Thread& th, Prim* prim)
{
	V a = th.pop();
	
	Gen* g = new Rand2z(th, a);
	th.push(new List(g));
}

static void nrand2s_(Thread& th, Prim* prim)
{
	V a = th.pop();
	int64_t n = th.popInt("nrand2s : n");
	
	Gen* g = new NRand2(th, n, a);
	th.push(new List(g));
}

static void nrand2z_(Thread& th, Prim* prim)
{
	V a = th.pop();
	int64_t n = th.popInt("nrand2z : n");
	
	Gen* g = new NRand2z(th, n, a);
	th.push(new List(g));
}


static void white_(Thread& th, Prim* prim)
{
	// synonym for rand2z
	V a = th.popZIn("white : a");
	Gen* g = new Rand2z(th, a);
	th.push(new List(g));
}

static void wangwhite_(Thread& th, Prim* prim)
{
	// synonym for rand2z
	V a = th.popZIn("wangwhite : a");
	Gen* g = new WangNoise(th, a);
	th.push(new List(g));
}

static void citywhite_(Thread& th, Prim* prim)
{
	// synonym for rand2z
	V a = th.popZIn("citywhite : a");
	Gen* g = new CityNoise(th, a);
	th.push(new List(g));
}

static void rawhite_(Thread& th, Prim* prim)
{
	// synonym for rand2z
	V a = th.popZIn("rawhite : a");
	Gen* g = new RandomAccessNoise(th, a);
	th.push(new List(g));
}

static void xorwhite_(Thread& th, Prim* prim)
{
	// synonym for rand2z
	V a = th.popZIn("xorwhite : a");
	Gen* g = new XorNoise1(th, a);
	th.push(new List(g));
}

static void xorwhite2_(Thread& th, Prim* prim)
{
	// synonym for rand2z
	V a = th.popZIn("xorwhite2 : a");
	Gen* g = new XorNoise2(th, a);
	th.push(new List(g));
}

static void violet_(Thread& th, Prim* prim)
{
	// synonym for rand2z
	V a = th.popZIn("white : a");
	Gen* g = new Violet(th, a);
	th.push(new List(g));
}


static void rand2_(Thread& th, Prim* prim)
{
	Z a = th.popFloat("rand2 : a");
	RGen& r = th.rgen;
	
	Z z = 2. * a * r.drand() - a;
	th.push(z);
}

struct IRand2 : OneInputGen<IRand2>
{	
    RGen r;
    
	IRand2(Thread& th,  Arg a) : OneInputGen<IRand2>(th, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "IRand2"; }
    
	void calc(int n, V* out, V* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = aa->asFloat();
			Z a2p1 = 2. * a + 1.;
			for (int i = 0; i < n; ++i) {
				out[i] = floor(a2p1 * r.drand() - a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				out[i] = floor((2. * a + 1.) * r.drand() - a);
			}
		}
	}
};

struct IRand2z : OneInputUGen<IRand2z>
{	
    RGen r;
    
	IRand2z(Thread& th,  Arg a) : OneInputUGen<IRand2z>(th, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "IRand2z"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			Z a2p1 = 2. * a + 1.;
			for (int i = 0; i < n; ++i) {
				out[i] = floor(a2p1 * r.drand() - a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				out[i] = floor((2. * a + 1.) * r.drand() - a);
			}
		}
	}
};

struct NIRand2 : NOneInputGen<NIRand2>
{	
    RGen r;
    
	NIRand2(Thread& th, int64_t n, Arg a) : NOneInputGen<NIRand2>(th, n, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NIRand2"; }
    
	void calc(int n, V* out, V* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = aa->asFloat();
			Z a2p1 = 2. * a + 1.;
			for (int i = 0; i < n; ++i) {
				out[i] = floor(a2p1 * r.drand() - a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = aa->asFloat(); aa += aStride;
				out[i] = floor((2. * a + 1.) * r.drand() - a);
			}
		}
	}
};

struct NIRand2z : NOneInputUGen<NIRand2z>
{	
    RGen r;
    
	NIRand2z(Thread& th, int64_t n, Arg a) : NOneInputUGen<NIRand2z>(th, n, a)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NIRand2z"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		if (aStride == 0) {
			Z a = *aa;
			Z a2p1 = 2. * a + 1.;
			for (int i = 0; i < n; ++i) {
				out[i] = floor(a2p1 * r.drand() - a);
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa; aa += aStride;
				out[i] = floor((2. * a + 1.) * r.drand() - a);
			}
		}
	}
};


static void irand2s_(Thread& th, Prim* prim)
{
	V a = th.pop();
	
	Gen* g = new IRand2(th, a);
	th.push(new List(g));
}

static void irand2z_(Thread& th, Prim* prim)
{
	V a = th.pop();
	
	Gen* g = new IRand2z(th, a);
	th.push(new List(g));
}

static void nirand2s_(Thread& th, Prim* prim)
{
	V a = th.pop();
	int64_t n = th.popInt("nirand2s : n");
	
	Gen* g = new NIRand2(th, n, a);
	th.push(new List(g));
}

static void nirand2z_(Thread& th, Prim* prim)
{
	V a = th.pop();
	int64_t n = th.popInt("nirand2z : n");
	
	Gen* g = new NIRand2z(th, n, a);
	th.push(new List(g));
}

static void irand2_(Thread& th, Prim* prim)
{
	int64_t a = th.popInt("irand2 : a");
	RGen& r = th.rgen;
	
	Z z = floor((2. * a + 1.) * r.drand() - a);
	th.push(z);
}


struct Pick : ZeroInputGen<Pick>
{
	P<Array> _array;
	RGen r;
	
	Pick(Thread& th, P<Array> const& array) : ZeroInputGen<Pick>(th, false), _array(array)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "Pick"; }
    
	void calc(int n, V* out) 
	{
		int64_t hi = _array->size();
		if (_array->isZ()) {
			Z* items = _array->z();
			for (int i = 0; i < n; ++i) {
				out[i] = items[r.irand0(hi)];
			}
		} else {
			V* items = _array->v();
			for (int i = 0; i < n; ++i) {
				out[i] = items[r.irand0(hi)];
			}
		}
	}
};

struct Pickz : ZeroInputUGen<Pickz>
{
	P<Array> _array;
	RGen r;
	
	Pickz(Thread& th, P<Array> const& array) : ZeroInputUGen<Pickz>(th, false), _array(array)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "Pickz"; }
    
	void calc(int n, Z* out) 
	{
		int64_t hi = _array->size();
		Z* items = _array->z();
		for (int i = 0; i < n; ++i) {
			out[i] = items[r.irand0(hi)];
		}
	}
};

struct NPick : NZeroInputGen<NPick>
{
	P<Array> _array;
	RGen r;
	
	NPick(Thread& th, int64_t n, P<Array> const& array) : NZeroInputGen<NPick>(th, n), _array(array)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NPick"; }
    
	void calc(int n, V* out) 
	{
		int64_t hi = _array->size();
		if (_array->isZ()) {
			Z* items = _array->z();
			for (int i = 0; i < n; ++i) {
				out[i] = items[r.irand0(hi)];
			}
		} else {
			V* items = _array->v();
			for (int i = 0; i < n; ++i) {
				out[i] = items[r.irand0(hi)];
			}
		}
	}
};

struct NPickz : NZeroInputUGen<NPickz>
{
	P<Array> _array;
	RGen r;
	
	NPickz(Thread& th, int64_t n, P<Array> const& array) : NZeroInputUGen<NPickz>(th, n), _array(array)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NPickz"; }
    
	void calc(int n, Z* out) 
	{
		int64_t hi = _array->size();
		Z* items = _array->z();
		for (int i = 0; i < n; ++i) {
			out[i] = items[r.irand0(hi)];
		}
	}
};

static void picks_(Thread& th, Prim* prim)
{
	P<List> a = th.popList("picks : list");
	if (!a->isFinite())
		indefiniteOp("picks : list must be finite","");
    
	a = a->pack(th);
	
	Gen* g = new Pick(th, a->mArray);
	th.push(new List(g));
}

static void pickz_(Thread& th, Prim* prim)
{
	P<List> a = th.popList("pickz : list");
	if (!a->isFinite())
		indefiniteOp("pickz : list must be finite","");
    
	a = a->pack(th);
	
	Gen* g = new Pickz(th, a->mArray);
	th.push(new List(g));
}

static void npicks_(Thread& th, Prim* prim)
{
	P<List> a = th.popList("npicks : list");
	int64_t n = th.popInt("npicks : n");
    
	if (!a->isFinite())
		indefiniteOp("npicks : list must be finite","");
    
	a = a->pack(th);
	
	Gen* g = new NPick(th, n, a->mArray);
	th.push(new List(g));
}

static void npickz_(Thread& th, Prim* prim)
{
	P<List> a = th.popList("npickz : list");
	int64_t n = th.popInt("npickz : n");
    
	if (!a->isFinite())
		indefiniteOp("npickz : list must be finite","");
    
	a = a->pack(th);
	
	Gen* g = new NPickz(th, n, a->mArray);
	th.push(new List(g));
}

static void pick_(Thread& th, Prim* prim)
{
	P<List> a = th.popList("pick : list");
	if (!a->isFinite())
		indefiniteOp("pick : list must be finite","");
    
	a = a->pack(th);
	int64_t n = a->mArray->size();
	th.push(a->at(th.rgen.irand0(n)));
}

static int64_t weightIndex(int64_t n, Z* z, Z r)
{
	Z sum = 0.;
	for (int64_t i = 0; i < n-1; ++i) {
		sum += z[i];
		if (r < sum) {
			return i;
		}
	}
	return n-1;
}

struct WPick : ZeroInputGen<WPick>
{
	P<Array> _array;
	P<Array> _weights;
	RGen r;
	
	WPick(Thread& th, P<Array> const& array, P<Array> const& weights) : ZeroInputGen<WPick>(th, false), _array(array), _weights(weights)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "WPick"; }
    
	void calc(int n, V* out) 
	{
		int64_t an = _array->size();
		Z* w = _weights->z();
		if (_array->isZ()) {
			Z* items = _array->z();
			for (int i = 0; i < n; ++i) {
				int64_t j = weightIndex(an, w, r.drand());
				out[i] = items[j];
			}
		} else {
			V* items = _array->v();
			for (int i = 0; i < n; ++i) {
				int64_t j = weightIndex(an, w, r.drand());
				out[i] = items[j];
			}
		}
	}
};

struct WPickz : ZeroInputUGen<WPickz>
{
	P<Array> _array;
	P<Array> _weights;
	RGen r;
	
	WPickz(Thread& th, P<Array> const& array, P<Array> const& weights) : ZeroInputUGen<WPickz>(th, false), _array(array), _weights(weights)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "WPickz"; }
    
	void calc(int n, Z* out) 
	{
		int64_t an = _array->size();
		Z* w = _weights->z();
		Z* items = _array->z();
		for (int i = 0; i < n; ++i) {
			int64_t j = weightIndex(an, w, r.drand());
			out[i] = items[j];
		}
	}
};

struct NWPick : NZeroInputGen<NWPick>
{
	P<Array> _array;
	P<Array> _weights;
	RGen r;
	
	NWPick(Thread& th, int64_t n, P<Array> const& array, P<Array> const& weights) : NZeroInputGen<NWPick>(th, n), _array(array), _weights(weights)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NWPick"; }
    
	void calc(int n, V* out) 
	{
		int64_t an = _array->size();
		Z* w = _weights->z();
		if (_array->isZ()) {
			Z* items = _array->z();
			for (int i = 0; i < n; ++i) {
				int64_t j = weightIndex(an, w, r.drand());
				out[i] = items[j];
			}
		} else {
			V* items = _array->v();
			for (int i = 0; i < n; ++i) {
				int64_t j = weightIndex(an, w, r.drand());
				out[i] = items[j];
			}
		}
	}
};

struct NWPickz : NZeroInputUGen<NWPickz>
{
	P<Array> _array;
	P<Array> _weights;
	RGen r;
	
	NWPickz(Thread& th, int64_t n, P<Array> const& array, P<Array> const& weights) : NZeroInputUGen<NWPickz>(th, n), _array(array), _weights(weights)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NWPickz"; }
    
	void calc(int n, Z* out) 
	{
		int64_t an = _array->size();
		Z* w = _weights->z();
		Z* items = _array->z();
		for (int i = 0; i < n; ++i) {
			int64_t j = weightIndex(an, w, r.drand());
			out[i] = items[j];
		}
	}
};

static P<Array> sumWeights(P<Array>& weights)
{
	const int64_t n = weights->size();

	P<Array> summedWeights = new Array(itemTypeZ, n);
	summedWeights->setSize(n);
	
	Z sum = 0.;
	Z* z = summedWeights->z();
	for (int64_t i = 0; i < n-1; ++i) {
		sum += weights->atz(i);
		z[i] = sum;
	}
	Z scale = 1. / sum;
	
	for (int64_t i = 0; i < n-1; ++i) {
		z[i] *= scale;
	}
	
	return summedWeights;
}

static void wpicks_(Thread& th, Prim* prim)
{
	P<List> w = th.popList("wpicks : weights");
	P<List> a = th.popList("wpicks : list");
	if (!a->isFinite())
		indefiniteOp("wpicks : list must be finite","");
	if (!w->isFinite())
		indefiniteOp("wpicks : weights must be finite","");
	
	a = a->pack(th);
	w = w->packz(th);

	P<Array> aa = a->mArray;
	P<Array> wa = w->mArray;

	if (aa->size() != wa->size()) {
		post("list and weights are not the same length.\n");
		throw errFailed;
	}

	Gen* g = new WPick(th, aa, wa);
	th.push(new List(g));
}

static void wpickz_(Thread& th, Prim* prim)
{
	P<List> w = th.popList("wpickz : weights");
	P<List> a = th.popList("wpickz : list");
	if (!a->isFinite())
		indefiniteOp("wpickz : list must be finite","");
	if (!w->isFinite())
		indefiniteOp("wpickz : weights must be finite","");
	
	a = a->pack(th);
	w = w->packz(th);

	P<Array> aa = a->mArray;
	P<Array> wa = w->mArray;

	if (aa->size() != wa->size()) {
		post("list and weights are not the same length.\n");
		throw errFailed;
	}
    
	Gen* g = new WPickz(th, aa, wa);
	th.push(new List(g));
}

static void nwpicks_(Thread& th, Prim* prim)
{
	P<List> w = th.popList("nwpicks : weights");
	P<List> a = th.popList("nwpicks : list");
	int64_t n = th.popInt("nwpicks : n");

	if (!a->isFinite())
		indefiniteOp("nwpicks : list must be finite","");
	if (!w->isFinite())
		indefiniteOp("nwpicks : weights must be finite","");
	
	a = a->pack(th);
	w = w->packz(th);

	P<Array> aa = a->mArray;
	P<Array> wa = w->mArray;

	if (aa->size() != wa->size()) {
		post("list and weights are not the same length.\n");
		throw errFailed;
	}

	Gen* g = new NWPick(th, n, aa, wa);
	th.push(new List(g));
}

static void nwpickz_(Thread& th, Prim* prim)
{
	P<List> w = th.popList("nwpickz : weights");
	P<List> a = th.popList("nwpickz : list");
	int64_t n = th.popInt("nwpickz : n");

	if (!a->isFinite())
		indefiniteOp("nwpickz : list must be finite","");
	if (!w->isFinite())
		indefiniteOp("nwpickz : weights must be finite","");
	
	a = a->pack(th);
	w = w->packz(th);

	P<Array> aa = a->mArray;
	P<Array> wa = w->mArray;

	if (aa->size() != wa->size()) {
		post("list and weights are not the same length.\n");
		throw errFailed;
	}

	Gen* g = new NWPickz(th, n, aa, wa);
	th.push(new List(g));
}


static void wpick_(Thread& th, Prim* prim)
{
	P<List> w = th.popList("wpick : weights");
	P<List> a = th.popList("wpick : list");
	if (!a->isFinite())
		indefiniteOp("wpick : list must be finite","");
	if (!w->isFinite())
		indefiniteOp("wpick : weights must be finite","");
	
	a = a->pack(th);
	w = w->pack(th);
	
	P<Array> aa = a->mArray;
	P<Array> wa = w->mArray;
	const int64_t n = aa->size();
	const int64_t wn = wa->size();

	if (n != wn) {
		post("list and weights are not the same length.\n");
		throw errFailed;
	}

	const Z r = th.rgen.drand();
	Z sum = 0.;
	for (int64_t i = 0; i < n-1; ++i) {
		sum += wa->atz(i);
		if (r < sum) {
			th.push(aa->at(i));
			return;
		}
	}
	th.push(aa->at(n-1));
}


struct WRand : ZeroInputGen<WRand>
{
	P<Array> _weights;
	RGen r;
	
	WRand(Thread& th, P<Array> const& weights) : ZeroInputGen<WRand>(th, false), _weights(weights)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "WRand"; }
    
	void calc(int n, V* out) 
	{
		int64_t wn = _weights->size();
		Z* w = _weights->z();
		for (int i = 0; i < n; ++i) {
			out[i] = (Z)weightIndex(wn, w, r.drand());
		}
	}
};

struct WRandz : ZeroInputUGen<WRandz>
{
	P<Array> _weights;
	RGen r;
	
	WRandz(Thread& th, P<Array> const& weights) : ZeroInputUGen<WRandz>(th, false), _weights(weights)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "WRandz"; }
    
	void calc(int n, Z* out) 
	{
		int64_t wn = _weights->size();
		Z* w = _weights->z();
		for (int i = 0; i < n; ++i) {
			out[i] = (Z)weightIndex(wn, w, r.drand());
		}
	}
};

struct NWRand : NZeroInputGen<NWRand>
{
	P<Array> _weights;
	RGen r;
	
	NWRand(Thread& th, int64_t n, P<Array> const& weights) : NZeroInputGen<NWRand>(th, n), _weights(weights)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NWRand"; }
    
	void calc(int n, V* out) 
	{
		int64_t wn = _weights->size();
		Z* w = _weights->z();
		for (int i = 0; i < n; ++i) {
			out[i] = (Z)weightIndex(wn, w, r.drand());
		}
	}
};

struct NWRandz : NZeroInputUGen<NWRandz>
{
	P<Array> _weights;
	RGen r;
	
	NWRandz(Thread& th, int64_t n, P<Array> const& weights) : NZeroInputUGen<NWRandz>(th, n), _weights(weights)
	{
		r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "NWRandz"; }
    
	void calc(int n, Z* out) 
	{
		int64_t wn = _weights->size();
		Z* w = _weights->z();
		for (int i = 0; i < n; ++i) {
			out[i] = (Z)weightIndex(wn, w, r.drand());
		}
	}
};


static void wrands_(Thread& th, Prim* prim)
{
	P<List> w = th.popList("wrands : weights");
	if (!w->isFinite())
		indefiniteOp("wrands : weights must be finite","");
	
	w = w->packz(th);

	P<Array> wa = w->mArray;

	Gen* g = new WRand(th, wa);
	th.push(new List(g));
}

static void wrandz_(Thread& th, Prim* prim)
{
	P<List> w = th.popList("wrandz : weights");
	if (!w->isFinite())
		indefiniteOp("wrandz : weights must be finite","");
	
	w = w->packz(th);

	P<Array> wa = w->mArray;

	Gen* g = new WRandz(th, wa);
	th.push(new List(g));
}

static void nwrands_(Thread& th, Prim* prim)
{
	P<List> w = th.popList("nwrands : weights");
	int64_t n = th.popInt("nwrands : n");

	if (!w->isFinite())
		indefiniteOp("nwrands : weights must be finite","");
	
	w = w->packz(th);

	P<Array> wa = w->mArray;

	Gen* g = new NWRand(th, n, wa);
	th.push(new List(g));
}

static void nwrandz_(Thread& th, Prim* prim)
{
	P<List> w = th.popList("nwrandz : weights");
	int64_t n = th.popInt("nwrandz : n");

	if (!w->isFinite())
		indefiniteOp("nwrandz : weights must be finite","");
	
	w = w->packz(th);

	P<Array> wa = w->mArray;

	Gen* g = new NWRandz(th, n, wa);
	th.push(new List(g));
}


static void wrand_(Thread& th, Prim* prim)
{
	P<List> w = th.popList("wrand : weights");
	if (!w->isFinite())
		indefiniteOp("wrand : weights must be finite","");
	
	w = w->pack(th);
	
	P<Array> wa = w->mArray;
	const int64_t n = wa->size();

	const Z r = th.rgen.drand();
	Z sum = 0.;
	for (int64_t i = 0; i < n-1; ++i) {
		sum += wa->atz(i);
		if (r < sum) {
			th.push((Z)i);
			return;
		}
	}
	th.push((Z)(n-1));
}


struct GrayNoise : OneInputUGen<GrayNoise>
{	
    RGen r;
	int32_t counter_;
    
	GrayNoise(Thread& th,  Arg a) : OneInputUGen<GrayNoise>(th, a), counter_(0)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "GrayNoise"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		Z K = 4.65661287308e-10f;
		int32_t counter = counter_;
		if (aStride == 0) {
			Z a = *aa * K;
			for (int i = 0; i < n; ++i) {
				counter ^= int32_t(1) << (r.trand() & 31);
				out[i] = counter * a;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa * K; aa += aStride;
				counter ^= int32_t(1) << (r.trand() & 31);
				out[i] = counter * a;
			}
		}
		counter_ = counter;
	}
};

struct Gray64Noise : OneInputUGen<Gray64Noise>
{	
    RGen r;
	int64_t counter_;
    
	Gray64Noise(Thread& th,  Arg a) : OneInputUGen<Gray64Noise>(th, a), counter_(0)
	{
        r.init(th.rgen.trand());
	}
	
	virtual const char* TypeName() const override { return "Gray64Noise"; }
    
	void calc(int n, Z* out, Z* aa, int aStride) 
	{
		Z K = 1.084202172485504434e-19;
		int64_t counter = counter_;
		if (aStride == 0) {
			Z a = *aa * K;
			for (int i = 0; i < n; ++i) {
				counter ^= 1LL << (r.trand() & 63);
				out[i] = counter * a;
			}
		} else {
			for (int i = 0; i < n; ++i) {
				Z a = *aa * K; aa += aStride;
				counter ^= 1LL << (r.trand() & 63);
				out[i] = counter * a;
			}
		}
		counter_ = counter;
	}
};

static void gray_(Thread& th, Prim* prim)
{
	V a = th.popZIn("gray : a");
	
	th.push(new List(new GrayNoise(th, a)));
}

static void gray64_(Thread& th, Prim* prim)
{
	V a = th.popZIn("gray64 : a");
	
	th.push(new List(new Gray64Noise(th, a)));
}

struct PinkNoise : Gen
{
	ZIn _a;
	uint64_t dice[16];
	uint64_t total_;
	
	PinkNoise(Thread& th, Arg a)
    : Gen(th, itemTypeZ, a.isFinite()), _a(a) 
	{
		total_ = 0;
		RGen& r = th.rgen;
		for (int i = 0; i < 16; ++i) {
			int64_t x = (uint64_t)r.trand() >> 16;
			total_ += x;
			dice[i] = x;
		}
	}
    
	virtual const char* TypeName() const override { return "PinkNoise"; }
	
	virtual void pull(Thread& th) override {
		RGen& r = th.rgen;
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		uint64_t total = total_;
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			Z *aa;
			if (_a(th, n,astride, aa)) {
				setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					uint64_t newrand = r.trand(); // Magnus Jonsson's suggestion.
					uint32_t counter = (uint32_t)newrand;
					newrand = newrand >> 16;
					int k = (CTZ(counter)) & 15;
					uint64_t prevrand = dice[k];
					dice[k] = newrand;
					total += (newrand - prevrand);
					newrand = (uint64_t)r.trand() >> 16;
					union { int64_t i; double f; } u;
					u.i = (total + newrand) | 0x4000000000000000LL;
					out[i] = *aa * (u.f - 3.);
					aa += astride;
				}
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		total_ = total;
		produce(framesToFill);
	}
};

struct PinkNoise0 : Gen
{
	ZIn _a;
	uint64_t dice[16];
	uint64_t total_;
	
	PinkNoise0(Thread& th, Arg a)
    : Gen(th, itemTypeZ, a.isFinite()), _a(a) 
	{
		total_ = 0;
		for (int i = 0; i < 16; ++i) {
			dice[i] = 0;
		}
	}
    
	virtual const char* TypeName() const override { return "PinkNoise0"; }
	
	virtual void pull(Thread& th) override {
		RGen& r = th.rgen;
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		uint64_t total = total_;
		const double scale = pow(2.,-47.)/17.;
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			Z *aa;
			if (_a(th, n,astride, aa)) {
				setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					uint64_t newrand = r.trand(); // Magnus Jonsson's suggestion.
					uint32_t counter = (uint32_t)newrand;
					newrand = newrand >> 16;
					int k = (CTZ(counter)) & 15;
					uint64_t prevrand = dice[k];
					dice[k] = newrand;
					total += (newrand - prevrand);
					newrand = (uint64_t)r.trand() >> 16;
					out[i] = *aa * (scale * double(total + newrand)) - 1;
					aa += astride;
				}
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		total_ = total;
		produce(framesToFill);
	}
};


struct BlueNoise : Gen
{
	ZIn _a;
	uint64_t dice[16];
	uint64_t total_;
	Z prev;
	
	BlueNoise(Thread& th, Arg a)
    : Gen(th, itemTypeZ, a.isFinite()), _a(a), prev(0.)
	{
		total_ = 0;
		RGen& r = th.rgen;
		for (int i = 0; i < 16; ++i) {
			int64_t x = (uint64_t)r.trand() >> 16;
			total_ += x;
			dice[i] = x;
		}
	}
    
	virtual const char* TypeName() const override { return "BlueNoise"; }
	
	virtual void pull(Thread& th) override {
		RGen& r = th.rgen;
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		uint64_t total = total_;
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			Z *aa;
			if (_a(th, n,astride, aa)) {
				setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					uint64_t newrand = r.trand(); // Magnus Jonsson's suggestion.
					uint32_t counter = (uint32_t)newrand;
					newrand = newrand >> 16;
					int k = (CTZ(counter)) & 15;
					uint64_t prevrand = dice[k];
					dice[k] = newrand;
					total += (newrand - prevrand);
					newrand = (uint64_t)r.trand() >> 16;
					union { int64_t i; double f; } u;
					u.i = (total + newrand) | 0x4000000000000000LL;
					Z x = 4. * *aa * (u.f - 3.);
					out[i] = x - prev;
					prev = x;
					aa += astride;
				}
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		total_ = total;
		produce(framesToFill);
	}
};

static void pink_(Thread& th, Prim* prim)
{
	V a = th.popZIn("pink : a");
	
	th.push(new List(new PinkNoise(th, a)));
}

static void pink0_(Thread& th, Prim* prim)
{
	V a = th.popZIn("pink0 : a");
	
	th.push(new List(new PinkNoise0(th, a)));
}

static void blue_(Thread& th, Prim* prim)
{
	V a = th.popZIn("blue : a");
	
	th.push(new List(new BlueNoise(th, a)));
}


struct BrownNoise : Gen
{
	ZIn _a;
	Z total_;
	
	BrownNoise(Thread& th, Arg a)
    : Gen(th, itemTypeZ, a.isFinite()), _a(a) 
	{
		total_ = 0;
		RGen& r = th.rgen;
		total_ = r.drand2();
        
	}
    
	virtual const char* TypeName() const override { return "BrownNoise"; }
	
	virtual void pull(Thread& th) override {
		RGen& r = th.rgen;
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		Z z = total_;
		while (framesToFill) {
			int n = framesToFill;
			int astride;
			Z *aa;
			if (_a(th, n,astride, aa)) {
				setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					z += r.drand16();
					if (z > 1.) z = 2. - z;
					else if (z < -1.) z = -2. - z;
					out[i] = *aa * z;
					aa += astride;
				}
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		total_ = z;
		produce(framesToFill);
	}
};


static void brown_(Thread& th, Prim* prim)
{
	V a = th.popZIn("brown : a");
	
	th.push(new List(new BrownNoise(th, a)));
}

struct Dust : Gen
{
	ZIn _density;
	ZIn _amp;
	Z _densmul;
	
	Dust(Thread& th, Arg density, Arg amp)
    : Gen(th, itemTypeZ, mostFinite(density, amp)), _density(density), _amp(amp), _densmul(th.rate.invSampleRate)
	{
	}
    
	virtual const char* TypeName() const override { return "Dust"; }
	
	virtual void pull(Thread& th) override {
		RGen& r = th.rgen;
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int densityStride, ampStride;
			Z *density, *amp;
			if (_density(th, n, densityStride, density) || _amp(th, n, ampStride, amp)) {
				setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					Z thresh = *density * _densmul;
					Z z = r.drand();
					out[i] = z < thresh ? *amp * z / thresh : 0.;
					density += densityStride;
					amp += ampStride;
				}
				_density.advance(n);
				_amp.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};


struct Dust2 : Gen
{
	ZIn _density;
	ZIn _amp;
	Z _densmul;
	
	Dust2(Thread& th, Arg density, Arg amp)
    : Gen(th, itemTypeZ, mostFinite(density, amp)), _density(density), _amp(amp), _densmul(th.rate.invSampleRate)
	{
	}
    
	virtual const char* TypeName() const override { return "Dust2"; }
	
	virtual void pull(Thread& th) override {
		RGen& r = th.rgen;
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int densityStride, ampStride;
			Z *density, *amp;
			if (_density(th, n, densityStride, density) || _amp(th, n, ampStride, amp)) {
				setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					Z thresh = *density * _densmul;
					Z z = r.drand();
					out[i] = z < thresh ? *amp * (2. * z / thresh - 1.) : 0.;
					density += densityStride;
					amp += ampStride;
				}
				_density.advance(n);
				_amp.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

struct Velvet : Gen
{
	ZIn _density;
	ZIn _amp;
	Z _densmul;
	
	Velvet(Thread& th, Arg density, Arg amp)
    : Gen(th, itemTypeZ, mostFinite(density, amp)), _density(density), _amp(amp), _densmul(th.rate.invSampleRate)
	{
	}
    
	virtual const char* TypeName() const override { return "Velvet"; }
	
	virtual void pull(Thread& th) override {
		RGen& r = th.rgen;
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int densityStride, ampStride;
			Z *density, *amp;
			if (_density(th, n, densityStride, density) || _amp(th, n, ampStride, amp)) {
				setDone();
				break;
			} else {
				for (int i = 0; i < n; ++i) {
					Z thresh = *density * _densmul;
					Z thresh2 = .5 * thresh;
					Z z = r.drand();
					out[i] = z < thresh ? (z<thresh2 ? -*amp : *amp) : 0.;
					density += densityStride;
					amp += ampStride;
				}
				_density.advance(n);
				_amp.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};


static void dust_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("dust : amp");
	V density = th.popZIn("dust : density");
	
	th.push(new List(new Dust(th, density, amp)));
}

static void dust2_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("dust2 : amp");
	V density = th.popZIn("dust2 : density");
	
	th.push(new List(new Dust2(th, density, amp)));
}

static void velvet_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("velvet : amp");
	V density = th.popZIn("velvet : density");
	
	th.push(new List(new Velvet(th, density, amp)));
}


static inline double HashRand(int64_t i)
{
	union { uint64_t i; double f; } u;
	u.i = 0x3FF0000000000000LL | ((uint64_t)(Hash64(i) + Hash64bad(i)) & 0x000FffffFFFFffffLL);
	return u.f - 1.;
}

struct Toosh : Gen
{
	ZIn _delay;
	ZIn _amp;
	int64_t _counter;
	Z sampleRate;
	
	Toosh(Thread& th, Arg delay, Arg amp)
    : Gen(th, itemTypeZ, mostFinite(delay, amp)), _delay(delay), _amp(amp), sampleRate(th.rate.sampleRate)
	{
		_counter = th.rgen.trand();
	}
    
	virtual const char* TypeName() const override { return "Toosh"; }
	
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		
		while (framesToFill) {
			int n = framesToFill;
			int delayStride, ampStride;
			Z *delay, *amp;
			if (_delay(th, n, delayStride, delay) || _amp(th, n, ampStride, amp)) {
				setDone();
				break;
			} else {
				if (delayStride) {
					for (int i = 0; i < n; ++i) {
						int64_t delaySamples = (int64_t)floor(sampleRate * *delay + .5);
						Z x = HashRand(_counter);
						Z y = HashRand(_counter - delaySamples);
						out[i] = .5 * *amp * (x - y);
						delay += delayStride;
						amp += ampStride;
						++_counter;
					}
				} else {
					int64_t delaySamples = (int64_t)floor(sampleRate * *delay + .5);
					for (int i = 0; i < n; ++i) {
						Z x = HashRand(_counter);
						Z y = HashRand(_counter - delaySamples);
						out[i] = .5 * *amp * (x - y);
						amp += ampStride;
						++_counter;
					}
				}
				_delay.advance(n);
				_amp.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};


struct TooshPlus : Gen
{
	ZIn _delay;
	ZIn _amp;
	int64_t _counter;
	Z sampleRate;
	
	TooshPlus(Thread& th, Arg delay, Arg amp)
    : Gen(th, itemTypeZ, mostFinite(delay, amp)), _delay(delay), _amp(amp), sampleRate(th.rate.sampleRate)
	{
		_counter = th.rgen.trand();
	}
    
	virtual const char* TypeName() const override { return "TooshPlus"; }
	
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		
		while (framesToFill) {
			int n = framesToFill;
			int delayStride, ampStride;
			Z *delay, *amp;
			if (_delay(th, n, delayStride, delay) || _amp(th, n, ampStride, amp)) {
				setDone();
				break;
			} else {
				if (delayStride) {
					for (int i = 0; i < n; ++i) {
						int64_t delaySamples = (int64_t)floor(sampleRate * *delay + .5);
						Z x = HashRand(_counter);
						Z y = HashRand(_counter - delaySamples);
						out[i] = .5 * *amp * (x + y);
						delay += delayStride;
						amp += ampStride;
						++_counter;
					}
				} else {
					int64_t delaySamples = (int64_t)floor(sampleRate * *delay + .5);
					for (int i = 0; i < n; ++i) {
						Z x = HashRand(_counter);
						Z y = HashRand(_counter - delaySamples);
						out[i] = .5 * *amp * (x + y);
						amp += ampStride;
						++_counter;
					}
				}
				_delay.advance(n);
				_amp.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

static void toosh_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("toosh : amp");
	V delay = th.popZIn("toosh : delay");
    
	th.push(new List(new Toosh(th, delay, amp)));
}

static void tooshp_(Thread& th, Prim* prim)
{
	V amp = th.popZIn("tooshp : amp");
	V delay = th.popZIn("tooshp : delay");
    
	th.push(new List(new TooshPlus(th, delay, amp)));
}

struct Crackle : Gen
{
	ZIn _param;
	Z _y1, _y2;
	
	Crackle(Thread& th, Arg param) 
    : Gen(th, itemTypeZ, param.isFinite()), _param(param), _y1(th.rgen.drand()), _y2(0.)
	{
	}
    
	virtual const char* TypeName() const override { return "Crackle"; }
	
	virtual void pull(Thread& th) override {
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int paramStride;
			Z *param;
			if (_param(th, n, paramStride, param)) {
				setDone();
				break;
			} else {
                Z y1 = _y1;
                Z y2 = _y2;
				for (int i = 0; i < n; ++i) {
                    Z y0 = fabs(y1 * *param - y2 - 0.05);
                    y2 = y1; y1 = y0;
					out[i] = y0;
					param += paramStride;
				}
                _y1 = y1;
                _y2 = y2;
				_param.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};


static void crackle_(Thread& th, Prim* prim)
{
	V param = th.popZIn("cracke : param");
    
	th.push(new List(new Crackle(th, param)));
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma mark ADD RANDOM OPS

#define DEF(NAME, TAKES, LEAVES, HELP) 	vm.def(#NAME, TAKES, LEAVES, NAME##_, HELP);
#define DEFMCX(NAME, N, HELP) 	vm.defmcx(#NAME, N, NAME##_, HELP);
#define DEFAM(NAME, MASK, HELP) 	vm.defautomap(#NAME, #MASK, NAME##_, HELP);
#define DEFnoeach(NAME, TAKES, LEAVES, HELP) 	vm.def(#NAME, TAKES, LEAVES, NAME##_, HELP, V(0.), true);

void AddRandomOps();
void AddRandomOps()
{
	vm.addBifHelp("\n*** random number generation ***");

	DEFnoeach(newseed, 0, 1, "(--> seed) make a new random seed.");
	DEFnoeach(setseed, 1, 0, "(seed -->) set the random seed.");

	
	vm.addBifHelp("\n*** single random numbers ***");
	DEFAM(rand, kk, "(a b --> r) return a uniformly distributed random real value from a to b.")
	DEFAM(coin, k, "(p --> r) return 1 with probability p, or 0 with probability (1-p).")
	DEFAM(rand2, k, "(a --> r) return a uniformly distributed random real value from -a to +a.")
	DEFAM(irand, kk, "(a b --> r) return a uniformly distributed random integer value from a to b.")
	DEFAM(irand2, k, "(a --> r) return a uniformly distributed random real value from -a to +a.")
	DEFAM(xrand, kk, "(a b --> r) return a exponentially distributed random real value from a to b.") 
	DEFAM(linrand, kk, "(a b --> r) return a linearly distributed random real value from a to b.")	 
	DEFAM(ilinrand, kk, "(a b --> r) return a linearly distributed random integer value from a to b.") 
	DEF(wrand, 1, 1, "(w --> r) return a randomly chosen index from a list of probability weights. w should sum to one.") 
	DEF(pick, 1, 1, "(a --> r) return a randomly chosen element from the finite list a.") 
	DEF(wpick, 2, 1, "(a w --> r) return a randomly chosen element from the finite list a using probability weights from w. w must be the same length as a and should sum to one.") 
	
	vm.addBifHelp("\n*** random streams ***");
	DEFAM(rands, kk, "(a b --> r) return a stream of uniformly distributed random real values from a to b.")
	DEFAM(coins, k, "(p --> r) return a stream of 1 with probability p, or 0 with probability (1-p).")
	DEFAM(eprands, kk, "(a b --> r) return a stream of uniformly distributed random integer values from a to b, excluding the previously returned value.")	
	DEFAM(rand2s, k, "(a --> r) return a stream of uniformly distributed random real values from -a to +a.")
	DEFAM(irands, kk, "(a b --> r) return a stream of uniformly distributed random integer values from a to b.")
	DEFAM(irand2s, k, "(a --> r) return a stream of uniformly distributed random real values from -a to +a.")
	DEFAM(xrands, kk, "(a b --> r) return a stream of exponentially distributed random real values from a to b.")
	DEFAM(linrands, kk, "(a b --> r) return a stream of linearly distributed random real values from a to b.")
	DEFAM(ilinrands, kk, "(a b --> r) return a stream of linearly distributed random integer values from a to b.")
	DEF(wrands, 1, 1, "(w --> r) return a stream of randomly chosen indices from a list of probability weights. w should sum to one.") 
	DEF(picks, 1, 1, "(a --> r) return a stream of randomly chosen elements from the finite list a.") 
	DEF(wpicks, 2, 1, "(a w --> r) return a stream of randomly chosen elements from the finite list a using probability weights from w. w must be the same length as a and should sum to one.") 
	
	vm.addBifHelp("\n*** random signals ***");
	DEFMCX(randz, 2, "(a b --> r) return a signal of uniformly distributed random real values from a to b.")
	DEFMCX(coinz, 1, "(p --> r) return a signal of 1 with probability p, or 0 with probability (1-p).")
	DEFMCX(eprandz, 2, "(a b --> r) return a signal of uniformly distributed random integer values from a to b, excluding the previously returned value")
	DEFMCX(rand2z, 1, "(a --> r) return a signal of uniformly distributed random real values from -a to +a.")
	DEFMCX(irandz, 2, "(a b --> r) return a signal of uniformly distributed random integer values from a to b.")
	DEFMCX(irand2z, 1, "(a --> r) return a signal of uniformly distributed random real values from -a to +a.")
	DEFMCX(xrandz, 2, "(a b --> r) return a signal of exponentially distributed random real values from a to b.")
	DEFMCX(linrandz, 2, "(a b --> r) return a signal of linearly distributed random real values from a to b.")
	DEFMCX(ilinrandz, 2, "(a b --> r) return a signal of linearly distributed random integer values from a to b.")
	DEFMCX(wrandz, 1, "(w --> r) return a signal of randomly chosen indices from a list of probability weights. w should sum to one.") 
	DEFMCX(pickz, 1, "(a --> r) return a signal of randomly chosen elements from the finite list a.") 
	DEFMCX(wpickz, 2, "(a w --> r) return a signal of randomly chosen elements from the finite list a using probability weights from w. w must be the same length as a and should sum to one.") 
    
	vm.addBifHelp("\n*** finite random streams ***");
	DEFAM(nrands, kkk, "(n a b --> r) return a stream of n uniformly distributed random real values from a to b.")
	DEFAM(ncoins, kk, "(n p --> r) return a stream of n 1 with probability p, or 0 with probability (1-p).")
	DEFAM(neprands, kkk, "(n a b --> r) return a stream of n uniformly distributed random integer values from a to b, excluding the previously returned value.")
	DEFAM(nrand2s, kk, "(n a --> r) return a stream of n uniformly distributed random real values from -a to +a.")
	DEFAM(nirands, kkk, "(n a b --> r) return a stream of n uniformly distributed random integer values from a to b.")
	DEFAM(nirand2s, kk, "(n a --> r) return a stream of n uniformly distributed random real values from -a to +a.")
	DEFAM(nxrands, kkk, "(n a b --> r) return a stream of n exponentially distributed random real values from a to b.")
	DEFAM(nlinrands, kkk, "(n a b --> r) return a stream of n linearly distributed random real values from a to b.")
	DEFAM(nilinrands, kkk, "(n a b --> r) return a stream of n linearly distributed random integer values from a to b.")
	DEFAM(nwrands, ka, "(n w --> r) return a stream of n randomly chosen indices from a list of probability weights. w should sum to one.") 
	DEFAM(npicks, ka, "(n a --> r) return a stream of n randomly chosen elements from the finite list a.") 
	DEFAM(nwpicks, kaa, "(n a w --> r) return a stream of n randomly chosen elements from the finite list a using probability weights from w. w must be the same length as a and should sum to one.") 
	
	vm.addBifHelp("\n*** finite random signals ***");
	DEFMCX(nrandz, 3, "(n a b --> r) return a signal of n uniformly distributed random real values from a to b.")
	DEFMCX(ncoinz, 2, "(n p --> r) return a signal of n 1 with probability p, or 0 with probability (1-p).")
	DEFMCX(neprandz, 3, "(n a b --> r) return a signal of n uniformly distributed random integer values from a to b, excluding the previously returned value")
	DEFMCX(nrand2z, 2, "(n a --> r) return a signal of n uniformly distributed random real values from -a to +a.")
	DEFMCX(nirandz, 3, "(n a b --> r) return a signal of n uniformly distributed random integer values from a to b.")
	DEFMCX(nirand2z, 2, "(n a --> r) return a signal of n uniformly distributed random real values from -a to +a.")
	DEFMCX(nxrandz, 3, "(n a b --> r) return a signal of n exponentially distributed random real values from a to b.")
	DEFMCX(nlinrandz, 3, "(n a b --> r) return a signal of n linearly distributed random real values from a to b.")
	DEFMCX(nilinrandz, 3, "(n a b --> r) return a signal of n linearly distributed random integer values from a to b.")	
	DEFMCX(nwrandz, 2, "(n w --> r) return a signal of n randomly chosen indices from a list of probability weights. w should sum to one.") 
	DEFMCX(npickz, 2, "(n a --> r) return a signal of n randomly chosen elements from the finite signal a.") 
	DEFMCX(nwpickz, 3, "(n a w --> r) return a signal of n randomly chosen elements from the finite signal a using probability weights from w. w must be the same length as a and should sum to one.") 
    
	vm.addBifHelp("\n*** noise unit generators ***");
	DEFMCX(violet, 1, "(amp --> z) violet noise")
	DEFMCX(blue, 1, "(amp --> z) blue noise")
	DEFMCX(xorwhite, 1, "(amp --> z) white noise")
	DEFMCX(xorwhite2, 1, "(amp --> z) white noise")
	DEFMCX(rawhite, 1, "(amp --> z) white noise based on Cessu's random access random numbers")
	DEFMCX(wangwhite, 1, "(amp --> z) white noise based on Thomas Wang's integer hash")
	DEFMCX(citywhite, 1, "(amp --> z) white noise based on a function from CityHash")
	DEFMCX(white, 1, "(amp --> z) white noise")
	DEFMCX(pink, 1, "(amp --> z) pink noise")
	DEFMCX(pink0, 1, "(amp --> z) pink noise")
	DEFMCX(brown, 1, "(amp --> z) brown noise")
	DEFMCX(gray, 1, "(amp --> z) bit flip noise")
	DEFMCX(gray64, 1, "(amp --> z) bit flip noise")
	DEFMCX(dust, 2, "(density amp --> z) a stream of impulses whose amplitude is random from 0 to a and whose average density is in impulses per second.")	
	DEFMCX(dust2, 2, "(density amp --> z) a stream of impulses whose amplitude is random from -a to +a and whose average density is in impulses per second.")
	DEFMCX(velvet, 2, "(density amp --> z) a stream of impulses whose amplitude is randomly either -a or +a and whose average density is in impulses per second.")
	DEFMCX(toosh, 2, "(delay amp --> z) flanged noise. difference of two white noise sources with a delay.")
	DEFMCX(tooshp, 2, "(delay amp--> z) flanged noise. sum of two white noise sources with a delay. no null at delay == 0. ")
	DEFMCX(crackle, 1, "(param --> z) a chaotic generator.")	
}


