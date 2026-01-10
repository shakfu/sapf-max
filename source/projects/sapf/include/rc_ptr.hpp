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

// An intrusive reference counting smart pointer.

template <class T>
class P
{
	T* p_;
public:	
	typedef T elem_t;
	
	P() : p_(nullptr) {}
	P(T* p) : p_(p) { if (p_) retain(p_); }
	~P() { if (p_) release(p_); /*p_ = (T*)0xdeaddeaddeaddeadLL;*/ }
	
	P(P const & r) : p_(r.p_) { if (p_) retain(p_); }
	template <class U> P(P<U> const & r) : p_(r()) { retain(p_); }

	P(P && r) : p_(r.p_) { r.p_ = nullptr; }
	template <class U> P(P<U> && r) : p_(r()) { r.p_ = nullptr; }

	P& operator=(P<T> && r)
	{
		if (this != &r) {
			T* oldp = p_;
			p_ = r.p_;
			r.p_ = nullptr;
			if (oldp) release(oldp);
		}
		return *this;
	}

	void swap(P& r) { T* t = p_; p_ = r.p_; r.p_ = t; }
	
	T& operator*() const { return *p_; }
	T* operator->() const { return p_; }
	
	T* operator()() const { return p_; }
	T* get() const { return p_; }
	//template <class U> operator U*() const { return (U*)p_; }

	bool operator==(P const& that) const { return p_ == that.p_; }
	bool operator!=(P const& that) const { return p_ != that.p_; }
	bool operator==(T* p) const { return p_ == p; }
	bool operator!=(T* p) const { return p_ != p; }

    operator bool () const
    {
        return p_ != 0;
    }

	void set(T* p)
	{
		if (p != p_)
		{
			T* oldp = p_;
			if (p) retain(p);
			p_ = p;
			if (oldp) release(oldp);
		}
	}

	P& operator=(P<T> const& r)
	{
		set(r.p_);
		return *this;
	}
	
	P& operator=(T* p)
	{
		set(p);
		return *this;
	}
	
	template <class U> 
	P& operator=(U* p)
	{
		set(p);
		return *this;
	}

};

template<class T, class U>
bool operator==(P<T> const & a, P<U> const & b) 
{
	return a.p_ == b.p_;
}

template<class T, class U>
bool operator!=(P<T> const & a, P<U> const & b) // never throws
{
	return a.p_ != b.p_;
}

template<class T>
bool operator==(P<T> const & a, T * b) // never throws
{
	return a.p_ == b;
}

template<class T>
bool operator!=(P<T> const & a, T * b) // never throws
{
	return a.p_ != b;
}

template<class T>
bool operator==(T * a, P<T> const & b) // never throws
{
	return a != b.p_;
}

template<class T>
bool operator!=(T * a, P<T> const & b) // never throws
{
	return a == b.p_;
}

template<class T, class U>
bool operator<(P<T> const & a, P<U> const & b) // never throws
{
	return a.p_ < b.p_;
}

