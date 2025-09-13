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

#ifndef __UGen_h__
#define __UGen_h__

#include "Object.hpp"

template <typename F>
struct ZeroInputGen : public Gen
{	
	ZeroInputGen(Thread& th, bool inIsFinite) 
    : Gen(th, itemTypeV, inIsFinite)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
        
		static_cast<F*>(this)->F::calc(framesToFill, out);
        
		mOut = mOut->nextp();
	}
};

template <typename F>
struct OneInputGen : public Gen
{
	VIn _a;
	
	OneInputGen(Thread& th, Arg a)
    : Gen(th, itemTypeV, a.isFinite()), 
    _a(a)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int aStride;
			V *a;
			if (_a(th, n,aStride, a)) {
				setDone();
				break;
			} else {
				static_cast<F*>(this)->F::calc(n, out, a, aStride);
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

template <typename F>
struct TwoInputGen : public Gen
{
	VIn _a;
	VIn _b;
	
	TwoInputGen(Thread& th, Arg a, Arg b) 
    : Gen(th, itemTypeV, mostFinite(a, b)), 
    _a(a), _b(b)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int aStride, bStride;
			V *a, *b;
			if (_a(th, n,aStride, a) || _b(th, n,bStride, b)) {
				setDone();
				break;
			} else {
				static_cast<F*>(this)->F::calc(n, out, a, b, aStride, bStride);
				_a.advance(n);
				_b.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

template <typename F>
struct ThreeInputGen : public Gen
{
	VIn _a;
	VIn _b;
	VIn _c;
	
	ThreeInputGen(Thread& th, Arg a, Arg b, Arg c)
    : Gen(th, itemTypeV, mostFinite(a, b, c)), 
    _a(a), _b(b), _c(c)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int aStride, bStride, cStride;
			V *a, *b, *c;
			if (_a(th, n,aStride, a) || _b(th, n,bStride, b)  || _c(th, n,cStride, c)) {
				setDone();
				break;
			} else {
				static_cast<F*>(this)->F::calc(n, out, a, b, c, aStride, bStride, cStride);
				_a.advance(n);
				_b.advance(n);
				_c.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

template <typename F>
struct FourInputGen : public Gen
{
	VIn _a;
	VIn _b;
	VIn _c;
	VIn _d;
	
	FourInputGen(Thread& th, Arg a, Arg b, Arg c, Arg d)
    : Gen(th, itemTypeV, mostFinite(a, b, c, d)), 
    _a(a), _b(b), _c(c), _d()
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		V* out = mOut->fulfill(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int aStride, bStride, cStride, dStride;
			V *a, *b, *c, *d;
			if (_a(th, n,aStride, a) || _b(th, n,bStride, b)  || _c(th, n,cStride, c) || _d(th, n, dStride, d)) {
				setDone();
				break;
			} else {
				static_cast<F*>(this)->F::calc(n, out, a, b, c, d, aStride, bStride, cStride, dStride);
				_a.advance(n);
				_b.advance(n);
				_c.advance(n);
				_d.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

template <typename F>
struct ZeroInputUGen : public Gen
{	
	ZeroInputUGen(Thread& th, bool inIsFinite) 
    : Gen(th, itemTypeZ, inIsFinite)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
        
		static_cast<F*>(this)->F::calc(framesToFill, out);
        
		mOut = mOut->nextp();
	}
};

template <typename F>
struct OneInputUGen : public Gen
{
	ZIn _a;
	
	OneInputUGen(Thread& th, Arg a)
    : Gen(th, itemTypeZ, a.isFinite()), 
    _a(a)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int aStride;
			Z *a;
			if (_a(th, n,aStride, a)) {
				setDone();
				break;
			} else {
				static_cast<F*>(this)->F::calc(n, out, a, aStride);
				_a.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

template <typename F>
struct TwoInputUGen : public Gen
{
	ZIn _a;
	ZIn _b;
	
	TwoInputUGen(Thread& th, Arg a, Arg b) 
    : Gen(th, itemTypeZ, mostFinite(a, b)), 
    _a(a), _b(b)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int aStride, bStride;
			Z *a, *b;
			if (_a(th, n,aStride, a) || _b(th, n,bStride, b)) {
				setDone();
				break;
			} else {
				static_cast<F*>(this)->F::calc(n, out, a, b, aStride, bStride);
				_a.advance(n);
				_b.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

template <typename F>
struct ThreeInputUGen : public Gen
{
	ZIn _a;
	ZIn _b;
	ZIn _c;
	
	ThreeInputUGen(Thread& th, Arg a, Arg b, Arg c)
    : Gen(th, itemTypeZ, mostFinite(a, b, c)), 
    _a(a), _b(b), _c(c)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int aStride, bStride, cStride;
			Z *a, *b, *c;
			if (_a(th, n,aStride, a) || _b(th, n,bStride, b)  || _c(th, n,cStride, c)) {
				setDone();
				break;
			} else {
				static_cast<F*>(this)->F::calc(n, out, a, b, c, aStride, bStride, cStride);
				_a.advance(n);
				_b.advance(n);
				_c.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

template <typename F>
struct FourInputUGen : public Gen
{
	ZIn _a;
	ZIn _b;
	ZIn _c;
	ZIn _d;
	
	FourInputUGen(Thread& th, Arg a, Arg b, Arg c, Arg d)
    : Gen(th, itemTypeZ, mostFinite(a, b, c, d)), 
    _a(a), _b(b), _c(c), _d(d)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int aStride, bStride, cStride, dStride;
			Z *a, *b, *c, *d;
			if (_a(th, n,aStride, a) || _b(th, n,bStride, b)  || _c(th, n,cStride, c) || _d(th, n, dStride, d)) {
				setDone();
				break;
			} else {
				static_cast<F*>(this)->F::calc(n, out, a, b, c, d, aStride, bStride, cStride, dStride);
				_a.advance(n);
				_b.advance(n);
				_c.advance(n);
				_d.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};


template <typename F>
struct FiveInputUGen : public Gen
{
	ZIn _a;
	ZIn _b;
	ZIn _c;
	ZIn _d;
	ZIn _e;
	
	FiveInputUGen(Thread& th, Arg a, Arg b, Arg c, Arg d, Arg e)
    : Gen(th, itemTypeZ, mostFinite(a, b, c, d, e)), 
    _a(a), _b(b), _c(c), _d(d), _e(e)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int aStride, bStride, cStride, dStride, eStride;
			Z *a, *b, *c, *d, *e;
			if (_a(th, n,aStride, a) || _b(th, n,bStride, b)  || _c(th, n,cStride, c) || _d(th, n, dStride, d) || _e(th, n, eStride, e)) {
				setDone();
				break;
			} else {
				static_cast<F*>(this)->F::calc(n, out, a, b, c, d, e, aStride, bStride, cStride, dStride, eStride);
				_a.advance(n);
				_b.advance(n);
				_c.advance(n);
				_d.advance(n);
				_e.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};

template <typename F>
struct EightInputUGen : public Gen
{
	ZIn _a;
	ZIn _b;
	ZIn _c;
	ZIn _d;
	ZIn _e;
	ZIn _f;
	ZIn _g;
	ZIn _h;
	
	EightInputUGen(Thread& th, Arg a, Arg b, Arg c, Arg d, Arg e, Arg f, Arg g, Arg h)
    : Gen(th, itemTypeZ, mostFinite(a, b, c, d, e, f, g, h)),
    _a(a), _b(b), _c(c), _d(d), _e(e), _f(f), _g(g), _h(h)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		int framesToFill = mBlockSize;
		Z* out = mOut->fulfillz(framesToFill);
		while (framesToFill) {
			int n = framesToFill;
			int aStride, bStride, cStride, dStride, eStride, fStride, gStride, hStride;
			Z *a, *b, *c, *d, *e, *f, *g, *h;
			if (_a(th, n,aStride, a) || _b(th, n,bStride, b)  || _c(th, n,cStride, c) || _d(th, n, dStride, d) || _e(th, n, eStride, e) || _f(th, n, fStride, f) || _g(th, n, gStride, g) || _h(th, n, hStride, h)) {
				setDone();
				break;
			} else {
				static_cast<F*>(this)->F::calc(n, out, a, b, c, d, e, f, g, h, aStride, bStride, cStride, dStride, eStride, fStride, gStride, hStride);
				_a.advance(n);
				_b.advance(n);
				_c.advance(n);
				_d.advance(n);
				_e.advance(n);
				_f.advance(n);
				_g.advance(n);
				_h.advance(n);
				framesToFill -= n;
				out += n;
			}
		}
		produce(framesToFill);
	}
};



template <typename F>
struct NZeroInputGen : public Gen
{	
    int64_t _n;
    
	NZeroInputGen(Thread& th, int64_t n) 
    : Gen(th, itemTypeV, true), _n(n)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		if (_n <= 0) {
			end();
		} else {
            int framesToFill = (int)std::min(_n, (int64_t)mBlockSize);
            V* out = mOut->fulfill(framesToFill);
            
            static_cast<F*>(this)->F::calc(framesToFill, out);
            
            mOut = mOut->nextp();
			_n -= framesToFill;
        }
    }
};

template <typename F>
struct NOneInputGen : public Gen
{
	VIn _a;
    int64_t _n;
	
	NOneInputGen(Thread& th, int64_t n, Arg a)
    : Gen(th, itemTypeV, true), 
    _a(a), _n(n)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		if (_n <= 0) {
			end();
		} else {
            int framesToFill = (int)std::min(_n, (int64_t)mBlockSize);
            V* out = mOut->fulfill(framesToFill);
            while (framesToFill) {
                int n = framesToFill;
                int aStride;
                V *a;
                if (_a(th, n,aStride, a)) {
                    setDone();
                    break;
                } else {
                    static_cast<F*>(this)->F::calc(n, out, a, aStride);
                    _a.advance(n);
                    framesToFill -= n;
                    out += n;
                }
                _n -= n;
            }
            produce(framesToFill);
        }
    }
};

template <typename F>
struct NTwoInputGen : public Gen
{
	VIn _a;
	VIn _b;
    int64_t _n;
	
	NTwoInputGen(Thread& th, int64_t n, Arg a, Arg b) 
    : Gen(th, itemTypeV, true), 
    _a(a), _b(b), _n(n)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		if (_n <= 0) {
			end();
		} else {
            int framesToFill = (int)std::min(_n, (int64_t)mBlockSize);
            V* out = mOut->fulfill(framesToFill);
            while (framesToFill) {
                int n = framesToFill;
                int aStride, bStride;
                V *a, *b;
                if (_a(th, n,aStride, a) || _b(th, n,bStride, b)) {
                    setDone();
                    break;
                } else {
                    static_cast<F*>(this)->F::calc(n, out, a, b, aStride, bStride);
                    _a.advance(n);
                    _b.advance(n);
                    framesToFill -= n;
                    out += n;
                }
                _n -= n;
            }
            produce(framesToFill);
        }
    }
};

template <typename F>
struct NThreeInputGen : public Gen
{
	VIn _a;
	VIn _b;
	VIn _c;
    int64_t _n;
	
	NThreeInputGen(Thread& th, int64_t n, Arg a, Arg b, Arg c)
    : Gen(th, itemTypeV, true), 
    _a(a), _b(b), _c(c), _n(n)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		if (_n <= 0) {
			end();
		} else {
            int framesToFill = (int)std::min(_n, (int64_t)mBlockSize);
            V* out = mOut->fulfill(framesToFill);
            while (framesToFill) {
                int n = framesToFill;
                int aStride, bStride, cStride;
                V *a, *b, *c;
                if (_a(th, n,aStride, a) || _b(th, n,bStride, b)  || _c(th, n,cStride, c)) {
                    setDone();
                    break;
                } else {
                    static_cast<F*>(this)->F::calc(n, out, a, b, c, aStride, bStride, cStride);
                    _a.advance(n);
                    _b.advance(n);
                    _c.advance(n);
                    framesToFill -= n;
                    out += n;
                }
                _n -= n;
            }
            produce(framesToFill);
        }
    }
};

template <typename F>
struct NFourInputGen : public Gen
{
	VIn _a;
	VIn _b;
	VIn _c;
	VIn _d;
    int64_t _n;
	
	NFourInputGen(Thread& th, int64_t n, Arg a, Arg b, Arg c, Arg d)
    : Gen(th, itemTypeV, true), 
    _a(a), _b(b), _c(c), _d(), _n(n)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		if (_n <= 0) {
			end();
		} else {
            int framesToFill = (int)std::min(_n, (int64_t)mBlockSize);
            V* out = mOut->fulfill(framesToFill);
            while (framesToFill) {
                int n = framesToFill;
                int aStride, bStride, cStride, dStride;
                V *a, *b, *c, *d;
                if (_a(th, n,aStride, a) || _b(th, n,bStride, b)  || _c(th, n,cStride, c) || _d(th, n, dStride, d)) {
                    setDone();
                    break;
                } else {
                    static_cast<F*>(this)->F::calc(n, out, a, b, c, d, aStride, bStride, cStride, dStride);
                    _a.advance(n);
                    _b.advance(n);
                    _c.advance(n);
                    _d.advance(n);
                    framesToFill -= n;
                    out += n;
                }
                _n -= n;
            }
            produce(framesToFill);
        }
	}
};



template <typename F>
struct NZeroInputUGen : public Gen
{	
    int64_t _n;
    
	NZeroInputUGen(Thread& th, int64_t n) 
    : Gen(th, itemTypeZ, true), _n(n)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		if (_n <= 0) {
			end();
		} else {
            int framesToFill = (int)std::min(_n, (int64_t)mBlockSize);
            Z* out = mOut->fulfillz(framesToFill);
            
            static_cast<F*>(this)->F::calc(framesToFill, out);
            
            mOut = mOut->nextp();
			_n -= framesToFill;
        }
    }
};

template <typename F>
struct NOneInputUGen : public Gen
{
	ZIn _a;
    int64_t _n;
	
	NOneInputUGen(Thread& th, int64_t n, Arg a)
    : Gen(th, itemTypeZ, true), 
    _a(a), _n(n)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		if (_n <= 0) {
			end();
		} else {
            int framesToFill = (int)std::min(_n, (int64_t)mBlockSize);
            Z* out = mOut->fulfillz(framesToFill);
            while (framesToFill) {
                int n = framesToFill;
                int aStride;
                Z *a;
                if (_a(th, n,aStride, a)) {
                    setDone();
                    break;
                } else {
                    static_cast<F*>(this)->F::calc(n, out, a, aStride);
                    _a.advance(n);
                    framesToFill -= n;
                    out += n;
                }
                _n -= n;
            }
            produce(framesToFill);
        }
    }
};

template <typename F>
struct NTwoInputUGen : public Gen
{
	ZIn _a;
	ZIn _b;
    int64_t _n;
	
	NTwoInputUGen(Thread& th, int64_t n, Arg a, Arg b) 
    : Gen(th, itemTypeZ, true), 
    _a(a), _b(b), _n(n)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		if (_n <= 0) {
			end();
		} else {
            int framesToFill = (int)std::min(_n, (int64_t)mBlockSize);
            Z* out = mOut->fulfillz(framesToFill);
            while (framesToFill) {
                int n = framesToFill;
                int aStride, bStride;
                Z *a, *b;
                if (_a(th, n,aStride, a) || _b(th, n,bStride, b)) {
                    setDone();
                    break;
                } else {
                    static_cast<F*>(this)->F::calc(n, out, a, b, aStride, bStride);
                    _a.advance(n);
                    _b.advance(n);
                    framesToFill -= n;
                    out += n;
                }
                _n -= n;
            }
            produce(framesToFill);
        }
    }
};

template <typename F>
struct NThreeInputUGen : public Gen
{
	ZIn _a;
	ZIn _b;
	ZIn _c;
    int64_t _n;
	
	NThreeInputUGen(Thread& th, int64_t n, Arg a, Arg b, Arg c)
    : Gen(th, itemTypeZ, true), 
    _a(a), _b(b), _c(c), _n(n)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		if (_n <= 0) {
			end();
		} else {
            int framesToFill = (int)std::min(_n, (int64_t)mBlockSize);
            Z* out = mOut->fulfillz(framesToFill);
            while (framesToFill) {
                int n = framesToFill;
                int aStride, bStride, cStride;
                Z *a, *b, *c;
                if (_a(th, n,aStride, a) || _b(th, n,bStride, b)  || _c(th, n,cStride, c)) {
                    setDone();
                    break;
                } else {
                    static_cast<F*>(this)->F::calc(n, out, a, b, c, aStride, bStride, cStride);
                    _a.advance(n);
                    _b.advance(n);
                    _c.advance(n);
                    framesToFill -= n;
                    out += n;
                }
                _n -= n;
            }
            produce(framesToFill);
        }
    }
};

template <typename F>
struct NFourInputUGen : public Gen
{
	ZIn _a;
	ZIn _b;
	ZIn _c;
	ZIn _d;
    int64_t _n;
	
	NFourInputUGen(Thread& th, int64_t n, Arg a, Arg b, Arg c, Arg d) 
    : Gen(th, itemTypeZ, true), 
    _a(a), _b(b), _c(c), _d(), _n(n)
	{
	}
    
	virtual void pull(Thread& th) override 
	{
		if (_n <= 0) {
			end();
		} else {
            int framesToFill = (int)std::min(_n, (int64_t)mBlockSize);
            Z* out = mOut->fulfillz(framesToFill);
            while (framesToFill) {
                int n = framesToFill;
                int aStride, bStride, cStride, dStride;
                Z *a, *b, *c, *d;
                if (_a(th, n,aStride, a) || _b(th, n,bStride, b)  || _c(th, n,cStride, c) || _d(th, n, dStride, d)) {
                    setDone();
                    break;
                } else {
                    static_cast<F*>(this)->F::calc(n, out, a, b, c, d, aStride, bStride, cStride, dStride);
                    _a.advance(n);
                    _b.advance(n);
                    _c.advance(n);
                    _d.advance(n);
                    framesToFill -= n;
                    out += n;
                }
                _n -= n;
            }
            produce(framesToFill);
        }
	}
};


void AddUGenOps();

#endif

