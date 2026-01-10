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

FFT::FFT()
	: n(0)
	, log2n(0)
#if SAPF_ACCELERATE
	, setup(nullptr)
#else
	, in(nullptr)
	, out(nullptr)
	, in_out_real(nullptr)
	, forward_out_of_place_plan(nullptr)
	, backward_out_of_place_plan(nullptr)
	, forward_in_place_plan(nullptr)
	, backward_in_place_plan(nullptr)
	, forward_real_plan(nullptr)
	, backward_real_plan(nullptr)
#endif
{
}

void FFT::init(size_t log2n) {
	this->n = 1ULL << log2n;
	this->log2n = log2n;

#if SAPF_ACCELERATE
	this->setup = vDSP_create_fftsetupD(this->log2n, kFFTRadix2);
#else
	this->in = (fftw_complex *) fftw_malloc(this->n * sizeof(fftw_complex));
	this->out = (fftw_complex *) fftw_malloc(this->n * sizeof(fftw_complex));
	// For real FFT: the real array has n elements, complex array has n/2+1 elements
	// For in-place: need 2*(n/2+1) elements for padding
	this->in_out_real = (double *) fftw_malloc(2 * (this->n / 2 + 1) * sizeof(double));

	this->forward_out_of_place_plan = fftw_plan_dft_1d(this->n, this->in, this->out, FFTW_FORWARD, FFTW_ESTIMATE);
	this->backward_out_of_place_plan = fftw_plan_dft_1d(this->n, this->in, this->out, FFTW_BACKWARD, FFTW_ESTIMATE);
	this->forward_in_place_plan = fftw_plan_dft_1d(this->n, this->in, this->in, FFTW_FORWARD, FFTW_ESTIMATE);
	this->backward_in_place_plan = fftw_plan_dft_1d(this->n, this->in, this->in, FFTW_BACKWARD, FFTW_ESTIMATE);
	this->forward_real_plan = fftw_plan_dft_r2c_1d(this->n, this->in_out_real, (fftw_complex *) this->in_out_real, FFTW_ESTIMATE);
	this->backward_real_plan = fftw_plan_dft_c2r_1d(this->n, (fftw_complex *) this->in_out_real, this->in_out_real, FFTW_ESTIMATE);
#endif
}

FFT::~FFT() {
#if SAPF_ACCELERATE
	if (this->setup) {
		vDSP_destroy_fftsetupD(this->setup);
	}
#else
	if (forward_out_of_place_plan) fftw_destroy_plan(forward_out_of_place_plan);
	if (backward_out_of_place_plan) fftw_destroy_plan(backward_out_of_place_plan);
	if (forward_in_place_plan) fftw_destroy_plan(forward_in_place_plan);
	if (backward_in_place_plan) fftw_destroy_plan(backward_in_place_plan);
	if (forward_real_plan) fftw_destroy_plan(forward_real_plan);
	if (backward_real_plan) fftw_destroy_plan(backward_real_plan);

	if (this->in) fftw_free(this->in);
	if (this->out) fftw_free(this->out);
	if (this->in_out_real) fftw_free(this->in_out_real);
#endif
}

void FFT::forward(double *inReal, double *inImag, double *outReal, double *outImag) {
	double scale = 2. / this->n;
#if SAPF_ACCELERATE
	DSPDoubleSplitComplex in_split;
	DSPDoubleSplitComplex out_split;

	in_split.realp = inReal;
	in_split.imagp = inImag;
	out_split.realp = outReal;
	out_split.imagp = outImag;

	vDSP_fft_zopD(this->setup, &in_split, 1, &out_split, 1, this->log2n, FFT_FORWARD);

	vDSP_vsmulD(outReal, 1, &scale, outReal, 1, this->n);
	vDSP_vsmulD(outImag, 1, &scale, outImag, 1, this->n);
#else
	for(size_t i = 0; i < this->n; i++) {
		this->in[i][0] = inReal[i];
		this->in[i][1] = inImag[i];
	}
	fftw_execute(this->forward_out_of_place_plan);
	for(size_t i = 0; i < this->n; i++) {
		outReal[i] = this->out[i][0] * scale;
		outImag[i] = this->out[i][1] * scale;
	}
#endif
}

void FFT::backward(double *inReal, double *inImag, double *outReal, double *outImag) {
	double scale = .5;
#if SAPF_ACCELERATE
	DSPDoubleSplitComplex in_split;
	DSPDoubleSplitComplex out_split;

	in_split.realp = inReal;
	in_split.imagp = inImag;
	out_split.realp = outReal;
	out_split.imagp = outImag;

	vDSP_fft_zopD(this->setup, &in_split, 1, &out_split, 1, this->log2n, FFT_INVERSE);

	vDSP_vsmulD(outReal, 1, &scale, outReal, 1, this->n);
	vDSP_vsmulD(outImag, 1, &scale, outImag, 1, this->n);
#else
	for(size_t i = 0; i < this->n; i++) {
		this->in[i][0] = inReal[i];
		this->in[i][1] = inImag[i];
	}
	fftw_execute(this->backward_out_of_place_plan);
	for(size_t i = 0; i < this->n; i++) {
		outReal[i] = this->out[i][0] * scale;
		outImag[i] = this->out[i][1] * scale;
	}
#endif
}

void FFT::forward_in_place(double *ioReal, double *ioImag) {
	double scale = 2. / this->n;
#if SAPF_ACCELERATE
	DSPDoubleSplitComplex io;

	io.realp = ioReal;
	io.imagp = ioImag;

	vDSP_fft_zipD(this->setup, &io, 1, this->log2n, FFT_FORWARD);

	vDSP_vsmulD(ioReal, 1, &scale, ioReal, 1, this->n);
	vDSP_vsmulD(ioImag, 1, &scale, ioImag, 1, this->n);
#else
	for(size_t i = 0; i < this->n; i++) {
		this->in[i][0] = ioReal[i];
		this->in[i][1] = ioImag[i];
	}
	fftw_execute(this->forward_in_place_plan);
	for(size_t i = 0; i < this->n; i++) {
		ioReal[i] = this->in[i][0] * scale;
		ioImag[i] = this->in[i][1] * scale;
	}
#endif
}

void FFT::backward_in_place(double *ioReal, double *ioImag) {
	double scale = .5;
#if SAPF_ACCELERATE
	DSPDoubleSplitComplex io;

	io.realp = ioReal;
	io.imagp = ioImag;

	vDSP_fft_zipD(this->setup, &io, 1, this->log2n, FFT_INVERSE);

	vDSP_vsmulD(ioReal, 1, &scale, ioReal, 1, this->n);
	vDSP_vsmulD(ioImag, 1, &scale, ioImag, 1, this->n);
#else
	for(size_t i = 0; i < this->n; i++) {
		this->in[i][0] = ioReal[i];
		this->in[i][1] = ioImag[i];
	}
	fftw_execute(this->backward_in_place_plan);
	for(size_t i = 0; i < this->n; i++) {
		ioReal[i] = this->in[i][0] * scale;
		ioImag[i] = this->in[i][1] * scale;
	}
#endif
}

void FFT::forward_real(double *inReal, double *outReal, double *outImag) {
	double scale = 2. / n;
	int n2 = this->n/2;
#if SAPF_ACCELERATE
	DSPDoubleSplitComplex in_split;
	DSPDoubleSplitComplex out_split;

	vDSP_ctozD((DSPDoubleComplex*)inReal, 1, &in_split, 1, n2);

	out_split.realp = outReal;
	out_split.imagp = outImag;

	vDSP_fft_zropD(this->setup, &in_split, 1, &out_split, 1, this->log2n, FFT_FORWARD);

	vDSP_vsmulD(outReal, 1, &scale, outReal, 1, n2);
	vDSP_vsmulD(outImag, 1, &scale, outImag, 1, n2);

	out_split.realp[n2] = out_split.imagp[0];
	out_split.imagp[0] = 0.;
	out_split.imagp[n2] = 0.;
#else
	for(size_t i = 0; i < this->n; i++) {
		this->in_out_real[i] = inReal[i];
	}
	fftw_execute(this->forward_real_plan);
	for(int i = 0; i < n2; i++) {
		outReal[i] = this->in_out_real[2*i] * scale;
		outImag[i] = this->in_out_real[2*i+1] * scale;
	}
#endif
}

void FFT::backward_real(double *inReal, double *inImag, double *outReal) {
	double scale = .5;
	int n2 = this->n/2;
#if SAPF_ACCELERATE
	DSPDoubleSplitComplex in_split;

	in_split.realp = inReal;
	in_split.imagp = inImag;

	in_split.imagp[0] = 0.;

	vDSP_fft_zripD(this->setup, &in_split, 1, this->log2n, FFT_INVERSE);

	vDSP_ztocD(&in_split, 1, (DSPDoubleComplex*)outReal, 2, n2);

	vDSP_vsmulD(outReal, 1, &scale, outReal, 1, n);
#else
	for(int i = 0; i < n2; i++) {
		this->in_out_real[2*i] = inReal[i];
		this->in_out_real[2*i+1] = inImag[i];
	}
	fftw_execute(this->backward_real_plan);
	for(size_t i = 0; i < this->n; i++) {
		outReal[i] = this->in_out_real[i] * scale;
	}
#endif
}

FFT ffts[kMaxFFTLogSize+1];

void initFFT()
{
	for (int i = kMinFFTLogSize; i <= kMaxFFTLogSize; ++i) {
		ffts[i].init(i);
	}
}

void fft(int n, double* inReal, double* inImag, double* outReal, double* outImag)
{
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);
	ffts[log2n].forward(inReal, inImag, outReal, outImag);
}

void ifft(int n, double* inReal, double* inImag, double* outReal, double* outImag)
{
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);
	ffts[log2n].backward(inReal, inImag, outReal, outImag);
}

void fft(int n, double* ioReal, double* ioImag)
{
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);
	ffts[log2n].forward_in_place(ioReal, ioImag);
}

void ifft(int n, double* ioReal, double* ioImag)
{
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);
	ffts[log2n].backward_in_place(ioReal, ioImag);
}


void rfft(int n, double* inReal, double* outReal, double* outImag)
{
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);
	ffts[log2n].forward_real(inReal, outReal, outImag);
}


void rifft(int n, double* inReal, double* inImag, double* outReal)
{
	int log2n = n == 0 ? 0 : 64 - __builtin_clzll(n - 1);
	ffts[log2n].backward_real(inReal, inImag, outReal);
}


#define USE_VFORCE 1

inline void complex_expD_conj(double& re, double& im)
{
	double rho = expf(re);
	re = rho * cosf(im);
	im = rho * sinf(im);
}
