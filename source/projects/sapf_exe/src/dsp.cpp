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

#include "dsp.hpp"
#include <string.h>
#include <stdio.h>
#include <cmath>


FFTSetupD fftSetups[kMaxFFTLogSize+1];

void initFFT()
{
	for (int i = kMinFFTLogSize; i <= kMaxFFTLogSize; ++i) {
		fftSetups[i] = vDSP_create_fftsetupD(i, kFFTRadix2);
	}
}

void fft(int n, double* inReal, double* inImag, double* outReal, double* outImag)
{
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);

	DSPDoubleSplitComplex in;
	DSPDoubleSplitComplex out;
	
	in.realp = inReal;
	in.imagp = inImag;
	out.realp = outReal;
	out.imagp = outImag;

	vDSP_fft_zopD(fftSetups[log2n], &in, 1, &out, 1, log2n, FFT_FORWARD);

	double scale = 2. / n;
	vDSP_vsmulD(outReal, 1, &scale, outReal, 1, n);
	vDSP_vsmulD(outImag, 1, &scale, outImag, 1, n);
}

void ifft(int n, double* inReal, double* inImag, double* outReal, double* outImag)
{
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);

	DSPDoubleSplitComplex in;
	DSPDoubleSplitComplex out;
	
	in.realp = inReal;
	in.imagp = inImag;
	out.realp = outReal;
	out.imagp = outImag;

	vDSP_fft_zopD(fftSetups[log2n], &in, 1, &out, 1, log2n, FFT_INVERSE);
	
	double scale = .5;
	vDSP_vsmulD(outReal, 1, &scale, outReal, 1, n);
	vDSP_vsmulD(outImag, 1, &scale, outImag, 1, n);
}

void fft(int n, double* ioReal, double* ioImag)
{
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);

	DSPDoubleSplitComplex io;
	
	io.realp = ioReal;
	io.imagp = ioImag;

	vDSP_fft_zipD(fftSetups[log2n], &io, 1, log2n, FFT_FORWARD);

	double scale = 2. / n;
	vDSP_vsmulD(ioReal, 1, &scale, ioReal, 1, n);
	vDSP_vsmulD(ioImag, 1, &scale, ioImag, 1, n);
}

void ifft(int n, double* ioReal, double* ioImag)
{
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);

	DSPDoubleSplitComplex io;
	
	io.realp = ioReal;
	io.imagp = ioImag;

	vDSP_fft_zipD(fftSetups[log2n], &io, 1, log2n, FFT_INVERSE);
	
	double scale = .5;
	vDSP_vsmulD(ioReal, 1, &scale, ioReal, 1, n);
	vDSP_vsmulD(ioImag, 1, &scale, ioImag, 1, n);
}


void rfft(int n, double* inReal, double* outReal, double* outImag)
{
    int n2 = n/2;
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);

	DSPDoubleSplitComplex in;
	DSPDoubleSplitComplex out;

    vDSP_ctozD((DSPDoubleComplex*)inReal, 1, &in, 1, n2);
	
	out.realp = outReal;
	out.imagp = outImag;

	vDSP_fft_zropD(fftSetups[log2n], &in, 1, &out, 1, log2n, FFT_FORWARD);

	double scale = 2. / n;
	vDSP_vsmulD(outReal, 1, &scale, outReal, 1, n2);
	vDSP_vsmulD(outImag, 1, &scale, outImag, 1, n2);
    
    out.realp[n2] = out.imagp[0];
    out.imagp[0] = 0.;
    out.imagp[n2] = 0.;
}


void rifft(int n, double* inReal, double* inImag, double* outReal)
{
    int n2 = n/2;
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);

	DSPDoubleSplitComplex in;
	
	in.realp = inReal;
	in.imagp = inImag;
	
    //in.imagp[0] = in.realp[n2];
    in.imagp[0] = 0.;

	vDSP_fft_zripD(fftSetups[log2n], &in, 1, log2n, FFT_INVERSE);

    vDSP_ztocD(&in, 1, (DSPDoubleComplex*)outReal, 2, n2);

	double scale = .5;
	vDSP_vsmulD(outReal, 1, &scale, outReal, 1, n);    
}


#define USE_VFORCE 1

inline void complex_expD_conj(double& re, double& im)
{
	double rho = expf(re);
	re = rho * cosf(im);
	im = rho * sinf(im);
}

