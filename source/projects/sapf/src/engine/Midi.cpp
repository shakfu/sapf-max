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

#include "Midi.hpp"
#include "VM.hpp"
#include "UGen.hpp"
#include "ErrorCodes.hpp"
#include "MidiRouter.hpp"
#include "sapf/MidiBackend.hpp"
#include <vector>
#include <cmath>

const double kOneOver127 = 1./127.;
const double kOneOver8191 = 1./8191.;

// MIDI control ops that call through to the backend

static void midiStart_(Thread& th, Prim* prim)
{
	EnsureDefaultMidiBackend();
	GetMidiBackend().initialize(16, 16);
}

static void midiRestart_(Thread& th, Prim* prim)
{
	EnsureDefaultMidiBackend();
	GetMidiBackend().restart();
}

static void midiStop_(Thread& th, Prim* prim)
{
	if (HasMidiBackend()) {
		GetMidiBackend().cleanup();
	}
}

static void midiList_(Thread& th, Prim* prim)
{
	EnsureDefaultMidiBackend();
	GetMidiBackend().listDevices();
}

static void midiConnectInput_(Thread& th, Prim* prim)
{
	int index = (int)th.popInt("midiConnectInput : port");
	int uid = (int)th.popInt("midiConnectInput : sourceUID");
	EnsureDefaultMidiBackend();
	GetMidiBackend().connectInput(uid, index);
}

static void midiDisconnectInput_(Thread& th, Prim* prim)
{
	int index = (int)th.popInt("midiDisconnectInput : port");
	int uid = (int)th.popInt("midiDisconnectInput : sourceUID");
	if (HasMidiBackend()) {
		GetMidiBackend().disconnectInput(uid, index);
	}
}

static void midiDebug_(Thread& th, Prim* prim)
{
    gMidiDebug = th.popFloat("midiDebug : onoff") != 0.;
}

// Instantaneous MIDI value ops

static void mctl1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mctl1 : hi");
	Z lo   = th.popFloat("mctl1 : lo");

	int cnum = th.popInt("mctl1 : ctlNum") & 127;
	int chan = (th.popInt("mctl1 : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl1 : srcIndex") & 15;
	Z z = kOneOver127 * gMidiState[srcIndex][chan].control[cnum];
	th.push(lo + z * (hi - lo));
}

static void xmctl1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmctl1 : hi");
	Z lo   = th.popFloat("xmctl1 : lo");

	int cnum = th.popInt("xmctl1 : ctlNum") & 127;
	int chan = (th.popInt("xmctl1 : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl1 : srcIndex") & 15;
	Z z = kOneOver127 * gMidiState[srcIndex][chan].control[cnum];
	th.push(lo * pow(hi / lo, z));
}

static void mpoly1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mpoly1 : hi");
	Z lo   = th.popFloat("mpoly1 : lo");

	int key = th.popInt("mpoly1 : key") & 127;
	int chan = (th.popInt("mpoly1 : chan") - 1) & 15;
	int srcIndex = th.popInt("mpoly1 : srcIndex") & 15;

	Z z = kOneOver127 * gMidiState[srcIndex][chan].polytouch[key];
	th.push(lo + z * (hi - lo));
}

static void xmpoly1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmpoly1 : hi");
	Z lo   = th.popFloat("xmpoly1 : lo");

	int key = th.popInt("xmpoly1 : key") & 127;
	int chan = (th.popInt("xmpoly1 : chan") - 1) & 15;
	int srcIndex = th.popInt("xmpoly1 : srcIndex") & 15;

	Z z = kOneOver127 * gMidiState[srcIndex][chan].polytouch[key];
	th.push(lo * pow(hi / lo, z));
}

static void mgate1_(Thread& th, Prim* prim)
{
	int key = th.popInt("mgate1 : key") & 127;
	int chan = (th.popInt("mgate1 : chan") - 1) & 15;
	int srcIndex = th.popInt("mgate1 : srcIndex") & 15;

	th.pushBool(gMidiState[srcIndex][chan].keyvel[key] > 0);
}

static void mtouch1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mtouch1 : hi");
	Z lo   = th.popFloat("mtouch1 : lo");

	int chan = (th.popInt("mtouch1 : chan") - 1) & 15;
	int srcIndex = th.popInt("mtouch1 : srcIndex") & 15;

	Z z = kOneOver127 * gMidiState[srcIndex][chan].touch;
	th.push(lo + z * (hi - lo));
}

static void xmtouch1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmtouch1 : hi");
	Z lo   = th.popFloat("xmtouch1 : lo");

	int chan = (th.popInt("xmtouch1 : chan") - 1) & 15;
	int srcIndex = th.popInt("xmtouch1 : srcIndex") & 15;

	Z z = kOneOver127 * gMidiState[srcIndex][chan].touch;
	th.push(lo * pow(hi / lo, z));
}

static void mprog1_(Thread& th, Prim* prim)
{
	int chan = (th.popInt("mprog1 : chan") - 1) & 15;
	int srcIndex = th.popInt("mprog1 : srcIndex") & 15;

	th.push(gMidiState[srcIndex][chan].touch);
}

static void mlastkey1_(Thread& th, Prim* prim)
{
	int chan = (th.popInt("mlastkey1 : chan") - 1) & 15;
	int srcIndex = th.popInt("mlastkey1 : srcIndex") & 15;

	th.push(gMidiState[srcIndex][chan].lastkey);
}

static void mlastvel1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mlastvel1 : hi");
	Z lo   = th.popFloat("mlastvel1 : lo");

	int chan = (th.popInt("mlastvel1 : chan") - 1) & 15;
	int srcIndex = th.popInt("mlastvel1 : srcIndex") & 15;

	Z z = kOneOver127 * gMidiState[srcIndex][chan].lastvel;
	th.push(lo + z * (hi - lo));
}

static void xmlastvel1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmlastvel1 : hi");
	Z lo   = th.popFloat("xmlastvel1 : lo");

	int chan = (th.popInt("xmlastvel1 : chan") - 1) & 15;
	int srcIndex = th.popInt("xmlastvel1 : srcIndex") & 15;

	Z z = kOneOver127 * gMidiState[srcIndex][chan].lastvel;
	th.push(lo * pow(hi / lo, z));
}


static void mbend1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mbend1 : hi");
	Z lo   = th.popFloat("mbend1 : lo");

	int chan = (th.popInt("mbend1 : chan") - 1) & 15;
	int srcIndex = th.popInt("mbend1 : srcIndex") & 15;

	Z z = kOneOver8191 * gMidiState[srcIndex][chan].bend;
	th.push(lo + z * (hi - lo));
}
static void xmbend1_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmbend1 : hi");
	Z lo   = th.popFloat("xmbend1 : lo");

	int chan = (th.popInt("mbend1 : chan") - 1) & 15;
	int srcIndex = th.popInt("xmbend1 : srcIndex") & 15;

	Z z = kOneOver8191 * gMidiState[srcIndex][chan].bend;
	th.push(lo * pow(hi / lo, z));
}


// MIDI UGens - these read from gMidiState which is updated by the router

struct MCtl : public TwoInputUGen<MCtl>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    int _srcIndex;
    int _chan;
    int _cnum;

	MCtl(Thread& th, int srcIndex, int chan, int cnum, Arg lo, Arg hi)
        : TwoInputUGen<MCtl>(th, lo, hi), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        _srcIndex(srcIndex), _chan(chan), _cnum(cnum)
	{
	}

	virtual const char* TypeName() const override { return "MCtl"; }

	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        uint8_t& ctl = gMidiState[_srcIndex][_chan].control[_cnum];
		for (int i = 0; i < n; ++i) {
            Z z = kOneOver127 * ctl;
			Z y0 = *lo + z * (*hi - *lo);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};


struct XMCtl : public TwoInputUGen<XMCtl>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    int _srcIndex;
    int _chan;
    int _cnum;

	XMCtl(Thread& th, int srcIndex, int chan, int cnum, Arg lo, Arg hi)
        : TwoInputUGen<XMCtl>(th, lo, hi), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        _srcIndex(srcIndex), _chan(chan), _cnum(cnum)
	{
	}

	virtual const char* TypeName() const override { return "XMCtl"; }

	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        uint8_t& ctl = gMidiState[_srcIndex][_chan].control[_cnum];
		for (int i = 0; i < n; ++i) {
            Z z = kOneOver127 * ctl;
			Z y0 = *lo * pow(*hi / *lo, z);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};


struct MPoly : public TwoInputUGen<MPoly>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    int _srcIndex;
    int _chan;
    int _cnum;

	MPoly(Thread& th, int srcIndex, int chan, int cnum, Arg lo, Arg hi)
        : TwoInputUGen<MPoly>(th, lo, hi), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        _srcIndex(srcIndex), _chan(chan), _cnum(cnum)
	{
	}

	virtual const char* TypeName() const override { return "MPoly"; }

	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        uint8_t& ctl = gMidiState[_srcIndex][_chan].polytouch[_cnum];
		for (int i = 0; i < n; ++i) {
            Z z = kOneOver127 * ctl;
			Z y0 = *lo + z * (*hi - *lo);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};


struct XMPoly : public TwoInputUGen<XMPoly>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    int _srcIndex;
    int _chan;
    int _cnum;

	XMPoly(Thread& th, int srcIndex, int chan, int cnum, Arg lo, Arg hi)
        : TwoInputUGen<XMPoly>(th, lo, hi), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        _srcIndex(srcIndex), _chan(chan), _cnum(cnum)
	{
	}

	virtual const char* TypeName() const override { return "XMPoly"; }

	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        uint8_t& ctl = gMidiState[_srcIndex][_chan].polytouch[_cnum];
		for (int i = 0; i < n; ++i) {
            Z z = kOneOver127 * ctl;
			Z y0 = *lo * pow(*hi / *lo, z);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};


struct MTouch : public TwoInputUGen<MTouch>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    int _srcIndex;
    int _chan;

	MTouch(Thread& th, int srcIndex, int chan, Arg lo, Arg hi)
        : TwoInputUGen<MTouch>(th, lo, hi), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        _srcIndex(srcIndex), _chan(chan)
	{
	}

	virtual const char* TypeName() const override { return "MTouch"; }

	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        uint8_t& ctl = gMidiState[_srcIndex][_chan].touch;
		for (int i = 0; i < n; ++i) {
            Z z = kOneOver127 * ctl;
			Z y0 = *lo + z * (*hi - *lo);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};


struct XMTouch : public TwoInputUGen<XMTouch>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    int _srcIndex;
    int _chan;

	XMTouch(Thread& th, int srcIndex, int chan, Arg lo, Arg hi)
        : TwoInputUGen<XMTouch>(th, lo, hi), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        _srcIndex(srcIndex), _chan(chan)
	{
	}

	virtual const char* TypeName() const override { return "XMTouch"; }

	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        uint8_t& ctl = gMidiState[_srcIndex][_chan].touch;
		for (int i = 0; i < n; ++i) {
            Z z = kOneOver127 * ctl;
			Z y0 = *lo * pow(*hi / *lo, z);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};


struct MBend : public TwoInputUGen<MBend>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    int _srcIndex;
    int _chan;

	MBend(Thread& th, int srcIndex, int chan, Arg lo, Arg hi)
        : TwoInputUGen<MBend>(th, lo, hi), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        _srcIndex(srcIndex), _chan(chan)
	{
	}

	virtual const char* TypeName() const override { return "MBend"; }

	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        uint16_t& ctl = gMidiState[_srcIndex][_chan].bend;
		for (int i = 0; i < n; ++i) {
            Z z = kOneOver8191 * ctl;
			Z y0 = *lo + z * (*hi - *lo);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};


struct XMBend : public TwoInputUGen<XMBend>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    int _srcIndex;
    int _chan;

	XMBend(Thread& th, int srcIndex, int chan, Arg lo, Arg hi)
        : TwoInputUGen<XMBend>(th, lo, hi), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        _srcIndex(srcIndex), _chan(chan)
	{
	}

	virtual const char* TypeName() const override { return "XMBend"; }

	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        uint16_t& ctl = gMidiState[_srcIndex][_chan].bend;
		for (int i = 0; i < n; ++i) {
            Z z = kOneOver8191 * ctl;
			Z y0 = *lo * pow(*hi / *lo, z);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};


struct MLastVel : public TwoInputUGen<MLastVel>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    int _srcIndex;
    int _chan;

	MLastVel(Thread& th, int srcIndex, int chan, Arg lo, Arg hi)
        : TwoInputUGen<MLastVel>(th, lo, hi), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        _srcIndex(srcIndex), _chan(chan)
	{
	}

	virtual const char* TypeName() const override { return "MLastVel"; }

	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        uint8_t& ctl = gMidiState[_srcIndex][_chan].lastvel;
		for (int i = 0; i < n; ++i) {
            Z z = kOneOver127 * ctl;
			Z y0 = *lo + z * (*hi - *lo);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};

struct XMLastVel : public TwoInputUGen<XMLastVel>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    int _srcIndex;
    int _chan;

	XMLastVel(Thread& th, int srcIndex, int chan, Arg lo, Arg hi)
        : TwoInputUGen<XMLastVel>(th, lo, hi), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        _srcIndex(srcIndex), _chan(chan)
	{
	}

	virtual const char* TypeName() const override { return "XMLastVel"; }

	void calc(int n, Z* out, Z* lo, Z* hi, int loStride, int hiStride)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        uint8_t& ctl = gMidiState[_srcIndex][_chan].lastvel;
		for (int i = 0; i < n; ++i) {
            Z z = kOneOver127 * ctl;
			Z y0 = *lo * pow(*hi / *lo, z);
			out[i] = y1 = y0 + b1 * (y1 - y0);
			lo += loStride;
			hi += hiStride;
		}
		_y1 = y1;
	}
};


struct MLastKey : public ZeroInputUGen<MLastKey>
{
    int _srcIndex;
    int _chan;

	MLastKey(Thread& th, int srcIndex, int chan)
        : ZeroInputUGen<MLastKey>(th, false),
        _srcIndex(srcIndex), _chan(chan)
	{
	}

	virtual const char* TypeName() const override { return "MLastKey"; }

	void calc(int n, Z* out)
	{
        uint8_t& ctl = gMidiState[_srcIndex][_chan].lastkey;
		for (int i = 0; i < n; ++i) {
			out[i] = ctl;
		}
	}
};

struct MProg : public ZeroInputUGen<MProg>
{
    int _srcIndex;
    int _chan;

	MProg(Thread& th, int srcIndex, int chan)
        : ZeroInputUGen<MProg>(th, false),
        _srcIndex(srcIndex), _chan(chan)
	{
	}

	virtual const char* TypeName() const override { return "MProg"; }

	void calc(int n, Z* out)
	{
        uint8_t& ctl = gMidiState[_srcIndex][_chan].program;
		for (int i = 0; i < n; ++i) {
			out[i] = ctl;
		}
	}
};

struct MGate : public ZeroInputUGen<MGate>
{
    int _srcIndex;
    int _chan;
    int _key;

	MGate(Thread& th, int srcIndex, int chan, int key)
        : ZeroInputUGen<MGate>(th, false),
        _srcIndex(srcIndex), _chan(chan), _key(key)
	{
	}

	virtual const char* TypeName() const override { return "MGate"; }

	void calc(int n, Z* out)
	{
        uint8_t& ctl = gMidiState[_srcIndex][_chan].keyvel[_key];
		for (int i = 0; i < n; ++i) {
			out[i] = ctl > 0 ? 1. : 0.;
		}
	}
};


struct ZCtl : public ZeroInputUGen<ZCtl>
{
	Z _b1;
	Z _y1;
	Z _lagmul;
    P<ZRef> zref;

	ZCtl(Thread& th, P<ZRef> const& inZRef)
        : ZeroInputUGen<ZCtl>(th, false), _b1(1. + gMidiLagMul * th.rate.invSampleRate),
        zref(inZRef)
	{
	}

	virtual const char* TypeName() const override { return "ZCtl"; }

	void calc(int n, Z* out)
	{
		Z y1 = _y1;
		Z b1 = _b1;
        Z& ctl = zref->z;
		for (int i = 0; i < n; ++i) {
			Z y0 = ctl;
			out[i] = y1 = y0 + b1 * (y1 - y0);
		}
		_y1 = y1;
	}
};

static void zctl_(Thread& th, Prim* prim)
{
	P<ZRef> zref = th.popZRef("mctl : zref");

	th.push(new List(new ZCtl(th, zref)));
}


static void mctl_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mctl : hi");
	Z lo   = th.popFloat("mctl : lo");

	int cnum = th.popInt("mctl : ctlNum") & 127;
	int chan = (th.popInt("mctl : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new MCtl(th, srcIndex, chan, cnum, lo, hi)));
}

static void xmctl_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmctl : hi");
	Z lo   = th.popFloat("xmctl : lo");

	int cnum = th.popInt("xmctl : ctlNum") & 127;
	int chan = (th.popInt("xmctl : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new XMCtl(th, srcIndex, chan, cnum, lo, hi)));
}

static void mpoly_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mpoly : hi");
	Z lo   = th.popFloat("mpoly : lo");

	int key = th.popInt("mpoly : key") & 127;
	int chan = (th.popInt("mpoly : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new MPoly(th, srcIndex, chan, key, lo, hi)));
}

static void xmpoly_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmpoly : hi");
	Z lo   = th.popFloat("xmpoly : lo");

	int key = th.popInt("xmpoly : key") & 127;
	int chan = (th.popInt("xmpoly : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new XMPoly(th, srcIndex, chan, key, lo, hi)));
}

static void mtouch_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mtouch : hi");
	Z lo   = th.popFloat("mtouch : lo");

	int chan = (th.popInt("mtouch : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new MTouch(th, srcIndex, chan, lo, hi)));
}

static void xmtouch_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmtouch : hi");
	Z lo   = th.popFloat("xmtouch : lo");

	int chan = (th.popInt("xmtouch : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new XMTouch(th, srcIndex, chan, lo, hi)));
}

static void mbend_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mbend : hi");
	Z lo   = th.popFloat("mbend : lo");

	int chan = (th.popInt("mbend : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new MBend(th, srcIndex, chan, lo, hi)));
}

static void xmbend_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmbend : hi");
	Z lo   = th.popFloat("xmbend : lo");

	int chan = (th.popInt("xmbend : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new XMBend(th, srcIndex, chan, lo, hi)));
}


static void mprog_(Thread& th, Prim* prim)
{
	int chan = (th.popInt("mprog : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new MProg(th, srcIndex, chan)));
}

static void mgate_(Thread& th, Prim* prim)
{
	int key = th.popInt("mgate : key") & 127;
	int chan = (th.popInt("mgate : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new MGate(th, srcIndex, chan, key)));
}


static void mlastvel_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("mlastvel : hi");
	Z lo   = th.popFloat("mlastvel : lo");

	int chan = (th.popInt("mlastvel : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new MLastVel(th, srcIndex, chan, lo, hi)));
}

static void mlastkey_(Thread& th, Prim* prim)
{
	int chan = (th.popInt("mlastkey : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new MLastKey(th, srcIndex, chan)));
}

static void xmlastvel_(Thread& th, Prim* prim)
{
	Z hi   = th.popFloat("xmlastvel : hi");
	Z lo   = th.popFloat("xmlastvel : lo");

	int chan = (th.popInt("xmlastvel : chan") - 1) & 15;
	int srcIndex = th.popInt("mctl : srcIndex") & 15;
	th.push(new List(new XMLastVel(th, srcIndex, chan, lo, hi)));
}

#define DEF(NAME, TAKES, LEAVES, HELP) 	vm.def(#NAME, TAKES, LEAVES, NAME##_, HELP);
#define DEFMCX(NAME, N, HELP) 	vm.defmcx(#NAME, N, NAME##_, HELP);
#define DEFAM(NAME, MASK, HELP) 	vm.defautomap(#NAME, #MASK, NAME##_, HELP);

void AddMidiOps()
{
	vm.addBifHelp("\n*** MIDI control ***");
	DEF(midiStart, 0, 0, "(-->) start up MIDI services");
	DEF(midiRestart, 0, 0, "(-->) rescan MIDI services");
	DEF(midiStop, 0, 0, "(-->) stop MIDI services");
	DEF(midiList, 0, 0, "(-->) list MIDI endpoints");
	DEF(midiConnectInput, 2, 0, "(sourceUID index -->) connect a MIDI source");
	DEF(midiDisconnectInput, 2, 0, "(sourceUID index -->) disconnect a MIDI source");
	DEF(midiDebug, 1, 0, "(onoff -->) turn on or off midi input monitoring");

	vm.addBifHelp("\n*** MIDI instantaneous value ***");
	DEFMCX(mctl1, 5, "(srcIndex chan ctlnum lo hi --> out) value of midi controller mapped to the linear range [lo,hi].");
	DEFMCX(mpoly1, 5, "(srcIndex chan key lo hi --> out) value of midi poly key pressure mapped to the linear range [lo,hi].");
	DEFMCX(mtouch1, 4, "(srcIndex chan lo hi --> out) value of midi channel pressure mapped to the linear range [lo,hi].");
	DEFMCX(mbend1, 4, "(srcIndex chan lo hi --> out) value of midi pitch bend mapped to the linear range [lo,hi].");
	DEFMCX(mprog1, 2, "(srcIndex chan --> out) value of midi channel program 0-127.");
	DEFMCX(mgate1, 3, "(srcIndex chan key --> out) value of midi key state. 1 if key is down, 0 if key is up.");
	DEFMCX(mlastkey1, 2, "(srcIndex chan --> out) value of key of most recent midi note on.");
	DEFMCX(mlastvel1, 4, "(srcIndex chan lo hi --> out) value of velocity of most recent midi note on mapped to the linear range [lo,hi].");

	DEFMCX(xmctl1, 5, "(srcIndex chan ctlnum lo hi --> out) value of midi controller mapped to the exponential range [lo,hi].");
	DEFMCX(xmpoly1, 5, "(srcIndex chan key lo hi --> out) value of midi poly key pressure mapped to the exponential range [lo,hi].");
	DEFMCX(xmtouch1, 4, "(srcIndex chan lo hi --> out) value of midi channel pressure mapped to the exponential range [lo,hi].");
	DEFMCX(xmbend1, 4, "(srcIndex chan lo hi --> out) value of midi pitch bend mapped to the exponential range [lo,hi].");
	DEFMCX(xmlastvel1, 4, "(srcIndex chan lo hi --> out) value of velocity of most recent midi note on mapped to the exponential range [lo,hi].");

	vm.addBifHelp("\n*** MIDI control signal ***");
	DEFMCX(mctl, 5, "(srcIndex chan ctlnum lo hi --> out) signal of midi controller mapped to the linear range [lo,hi].");
	DEFMCX(mpoly, 5, "(srcIndex chan key lo hi --> out) signal of midi poly key pressure mapped to the linear range [lo,hi].");
	DEFMCX(mtouch, 4, "(srcIndex chan lo hi --> out) signal of midi channel pressure mapped to the linear range [lo,hi].");
	DEFMCX(mbend, 4, "(srcIndex chan lo hi --> out) signal of midi pitch bend mapped to the linear range [lo,hi].");
	DEFMCX(mlastkey, 2, "(srcIndex chan --> out) signal of key of most recent midi note on.");
	DEFMCX(mlastvel, 4, "(srcIndex chan lo hi --> out) signal of velocity of most recent midi note on mapped to the linear range [lo,hi].");

	DEFMCX(mprog, 2, "(srcIndex chan --> out) signal of midi channel program 0-127.");
	DEFMCX(mgate, 3, "(srcIndex chan key --> out) signal of midi key state. 1 if key is down, 0 if key is up.");

	DEFMCX(xmctl, 5, "(srcIndex chan ctlnum lo hi --> out) signal of midi controller mapped to the exponential range [lo,hi].");
	DEFMCX(xmpoly, 5, "(srcIndex chan key lo hi --> out) signal of midi poly key pressure mapped to the exponential range [lo,hi].");
	DEFMCX(xmtouch, 4, "(srcIndex chan lo hi --> out) signal of midi channel pressure mapped to the exponential range [lo,hi].");
	DEFMCX(xmbend, 4, "(srcIndex chan lo hi --> out) signal of midi pitch bend mapped to the exponential range [lo,hi].");
	DEFMCX(xmlastvel, 4, "(srcIndex chan lo hi --> out) signal of velocity of most recent midi note on mapped to the exponential range [lo,hi].");

	vm.addBifHelp("\n*** ZRef control signal ***");
	DEFMCX(zctl, 1, "(zref --> out) makes a smoothed control signal from a zref.");
}
