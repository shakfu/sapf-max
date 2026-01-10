#pragma once

#if defined(__APPLE__)
#define SAPF_HAS_ACCELERATE 1
#include <Accelerate/Accelerate.h>
#else
#define SAPF_HAS_ACCELERATE 0

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstring>
#include <limits>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

struct DSPDoubleComplex {
	double real;
	double imag;
};

struct DSPDoubleSplitComplex {
	double* realp;
	double* imagp;
};

struct FFTSetupDStruct {
	int log2n;
};

using FFTSetupD = FFTSetupDStruct*;

constexpr int kFFTRadix2 = 0;
constexpr int FFT_FORWARD = 1;
constexpr int FFT_INVERSE = -1;
constexpr int kFFTDirection_Forward = FFT_FORWARD;
constexpr int kFFTDirection_Inverse = FFT_INVERSE;

inline FFTSetupD vDSP_create_fftsetupD(int log2n, int)
{
	return new FFTSetupDStruct{log2n};
}

inline void vDSP_destroy_fftsetupD(FFTSetupD setup)
{
	delete setup;
}

template <typename Func>
inline void sapf_loop_unary(const double* in, int istride, double* out, int ostride, int n, Func fn)
{
	for (int i = 0; i < n; ++i) {
		out[i * ostride] = fn(in[i * istride]);
	}
}

template <typename Func>
inline void sapf_loop_binary(const double* a, int astride, const double* b, int bstride, double* out, int ostride, int n, Func fn)
{
	for (int i = 0; i < n; ++i) {
		out[i * ostride] = fn(a[i * astride], b[i * bstride]);
	}
}

inline void vDSP_vnegD(const double* aa, int astride, double* out, int ostride, int n)
{
	sapf_loop_unary(aa, astride, out, ostride, n, [](double v) { return -v; });
}

inline void vDSP_vsubD(const double* aa, int astride, const double* bb, int bstride, double* out, int ostride, int n)
{
	sapf_loop_binary(aa, astride, bb, bstride, out, ostride, n, [](double a, double b) { return b - a; });
}

inline void vDSP_vssqD(const double* aa, int astride, double* out, int ostride, int n)
{
	sapf_loop_unary(aa, astride, out, ostride, n, [](double v) {
		return std::copysign(v * v, v);
	});
}

inline void vDSP_vsqD(const double* aa, int astride, double* out, int ostride, int n)
{
	sapf_loop_unary(aa, astride, out, ostride, n, [](double v) { return v * v; });
}

inline void vDSP_vsmulD(const double* aa, int astride, const double* scalar, double* out, int ostride, int n)
{
	const double s = scalar ? *scalar : 0.;
	sapf_loop_unary(aa, astride, out, ostride, n, [s](double v) { return v * s; });
}

inline void vDSP_vsaddD(const double* aa, int astride, const double* scalar, double* out, int ostride, int n)
{
	const double s = scalar ? *scalar : 0.;
	sapf_loop_unary(aa, astride, out, ostride, n, [s](double v) { return v + s; });
}

inline void vDSP_vdbconD(const double* aa, int astride, const double* ref, double* out, int ostride, int n, int useAmplitude)
{
	const double refVal = ref ? std::max(*ref, std::numeric_limits<double>::min()) : 1.0;
	const double factor = useAmplitude ? 20.0 : 10.0;
	sapf_loop_unary(aa, astride, out, ostride, n, [refVal, factor](double v) {
		double ratio = std::fabs(v) / refVal;
		if (ratio <= std::numeric_limits<double>::min()) {
			ratio = std::numeric_limits<double>::min();
		}
		return factor * std::log10(ratio);
	});
}

inline void vDSP_svdivD(const double* scalar, const double* aa, int astride, double* out, int ostride, int n)
{
	const double s = scalar ? *scalar : 0.;
	sapf_loop_unary(aa, astride, out, ostride, n, [s](double v) { return s / v; });
}

inline void vDSP_vaddD(const double* aa, int astride, const double* bb, int bstride, double* out, int ostride, int n)
{
	sapf_loop_binary(aa, astride, bb, bstride, out, ostride, n, [](double a, double b) { return a + b; });
}

inline void vDSP_vmulD(const double* aa, int astride, const double* bb, int bstride, double* out, int ostride, int n)
{
	sapf_loop_binary(aa, astride, bb, bstride, out, ostride, n, [](double a, double b) { return a * b; });
}

inline void vDSP_vdivD(const double* denom, int dstride, const double* numer, int nstride, double* out, int ostride, int n)
{
	sapf_loop_binary(denom, dstride, numer, nstride, out, ostride, n, [](double d, double nval) { return nval / d; });
}

inline void vDSP_vminD(const double* aa, int astride, const double* bb, int bstride, double* out, int ostride, int n)
{
	sapf_loop_binary(aa, astride, bb, bstride, out, ostride, n, [](double a, double b) { return std::min(a, b); });
}

inline void vDSP_vmaxD(const double* aa, int astride, const double* bb, int bstride, double* out, int ostride, int n)
{
	sapf_loop_binary(aa, astride, bb, bstride, out, ostride, n, [](double a, double b) { return std::max(a, b); });
}

inline void vDSP_vdistD(const double* aa, int astride, const double* bb, int bstride, double* out, int ostride, int n)
{
	sapf_loop_binary(aa, astride, bb, bstride, out, ostride, n, [](double a, double b) { return std::hypot(a, b); });
}

inline void vDSP_hann_windowD(double* data, int n, int flag)
{
	const double denom = std::max(1, flag ? n : n - 1);
	for (int i = 0; i < n; ++i) {
		data[i] = 0.5 - 0.5 * std::cos(2.0 * M_PI * i / denom);
	}
}

inline void vDSP_hamm_windowD(double* data, int n, int flag)
{
	const double denom = std::max(1, flag ? n : n - 1);
	for (int i = 0; i < n; ++i) {
		data[i] = 0.54 - 0.46 * std::cos(2.0 * M_PI * i / denom);
	}
}

inline void vDSP_blkman_windowD(double* data, int n, int flag)
{
	const double denom = std::max(1, flag ? n : n - 1);
	for (int i = 0; i < n; ++i) {
		const double phase = 2.0 * M_PI * i / denom;
		data[i] = 0.42 - 0.5 * std::cos(phase) + 0.08 * std::cos(2.0 * phase);
	}
}

inline void vDSP_rectD(const double* polar, int pstride, double* rect, int rstride, size_t n)
{
	for (size_t i = 0; i < n; ++i) {
		const double mag = polar[2 * i * pstride];
		const double phase = polar[2 * i * pstride + 1];
		rect[2 * i * rstride] = mag * std::cos(phase);
		rect[2 * i * rstride + 1] = mag * std::sin(phase);
	}
}

inline void vDSP_ctozD(const DSPDoubleComplex* input, int istride, DSPDoubleSplitComplex* output, int ostride, size_t n)
{
	for (size_t i = 0; i < n; ++i) {
		const DSPDoubleComplex& c = input[i * istride];
		output->realp[i * ostride] = c.real;
		output->imagp[i * ostride] = c.imag;
	}
}

inline void vDSP_ztocD(const DSPDoubleSplitComplex* input, int istride, DSPDoubleComplex* output, int ostride, size_t n)
{
	for (size_t i = 0; i < n; ++i) {
		DSPDoubleComplex& c = output[i * ostride];
		c.real = input->realp[i * istride];
		c.imag = input->imagp[i * istride];
	}
}

template <typename Func>
inline void sapf_apply_vforce(Func fn, double* out, const double* in, int n)
{
	for (int i = 0; i < n; ++i) {
		out[i] = fn(in[i]);
	}
}

inline void vvfloor(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::floor(a); }, out, in, *n);
}

inline void vvceil(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::ceil(a); }, out, in, *n);
}

inline void vvnint(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::rint(a); }, out, in, *n);
}

inline void vvfabs(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::fabs(a); }, out, in, *n);
}

inline void vvrec(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return 1. / a; }, out, in, *n);
}

inline void vvsqrt(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::sqrt(a); }, out, in, *n);
}

inline void vvrsqrt(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return 1. / std::sqrt(a); }, out, in, *n);
}

inline void vvexp(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::exp(a); }, out, in, *n);
}

inline void vvexp2(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::exp2(a); }, out, in, *n);
}

inline void vvexpm1(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::expm1(a); }, out, in, *n);
}

inline void vvlog(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::log(a); }, out, in, *n);
}

inline void vvlog2(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::log2(a); }, out, in, *n);
}

inline void vvlog10(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::log10(a); }, out, in, *n);
}

inline void vvlog1p(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::log1p(a); }, out, in, *n);
}

inline void vvlogb(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::logb(a); }, out, in, *n);
}

inline void vvsin(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::sin(a); }, out, in, *n);
}

inline void vvcos(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::cos(a); }, out, in, *n);
}

inline void vvtan(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::tan(a); }, out, in, *n);
}

inline void vvasin(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::asin(a); }, out, in, *n);
}

inline void vvacos(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::acos(a); }, out, in, *n);
}

inline void vvatan(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::atan(a); }, out, in, *n);
}

inline void vvsinh(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::sinh(a); }, out, in, *n);
}

inline void vvcosh(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::cosh(a); }, out, in, *n);
}

inline void vvtanh(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::tanh(a); }, out, in, *n);
}

inline void vvasinh(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::asinh(a); }, out, in, *n);
}

inline void vvacosh(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::acosh(a); }, out, in, *n);
}

inline void vvatanh(double* out, const double* in, const int* n)
{
	sapf_apply_vforce([](double a) { return std::atanh(a); }, out, in, *n);
}

inline void vvcopysign(double* out, const double* a, const double* b, const int* n)
{
	for (int i = 0; i < *n; ++i) {
		out[i] = std::copysign(a[i], b[i]);
	}
}

inline void vvnextafter(double* out, const double* a, const double* b, const int* n)
{
	for (int i = 0; i < *n; ++i) {
		out[i] = std::nextafter(a[i], b[i]);
	}
}

inline void vvpow(double* out, const double* a, const double* b, const int* n)
{
	for (int i = 0; i < *n; ++i) {
		out[i] = std::pow(a[i], b[i]);
	}
}

inline void vvatan2(double* out, const double* a, const double* b, const int* n)
{
	for (int i = 0; i < *n; ++i) {
		out[i] = std::atan2(a[i], b[i]);
	}
}

namespace sapf {

inline void fft(std::vector<std::complex<double>>& data, bool inverse)
{
	const size_t n = data.size();
	size_t j = 0;
	for (size_t i = 1; i < n; ++i) {
		size_t bit = n >> 1;
		for (; j & bit; bit >>= 1) {
			j ^= bit;
		}
		j ^= bit;
		if (i < j) {
			std::swap(data[i], data[j]);
		}
	}
	for (size_t len = 2; len <= n; len <<= 1) {
		const double ang = 2.0 * M_PI / len * (inverse ? 1.0 : -1.0);
		const std::complex<double> wlen(std::cos(ang), std::sin(ang));
		for (size_t i = 0; i < n; i += len) {
			std::complex<double> w(1.0, 0.0);
			for (size_t k = 0; k < len / 2; ++k) {
				std::complex<double> u = data[i + k];
				std::complex<double> v = data[i + k + len / 2] * w;
				data[i + k] = u + v;
				data[i + k + len / 2] = u - v;
				w *= wlen;
			}
		}
	}
	if (inverse) {
		// vDSP leaves scaling to the caller, so we match that behavior.
	}
}

inline std::vector<std::complex<double>> toComplex(const DSPDoubleSplitComplex* data, int stride, size_t n)
{
	std::vector<std::complex<double>> result(n);
	for (size_t i = 0; i < n; ++i) {
		result[i] = {data->realp[i * stride], data->imagp[i * stride]};
	}
	return result;
}

inline void fromComplex(const std::vector<std::complex<double>>& data, DSPDoubleSplitComplex* out, int stride)
{
	for (size_t i = 0; i < data.size(); ++i) {
		out->realp[i * stride] = data[i].real();
		out->imagp[i * stride] = data[i].imag();
	}
}

} // namespace sapf

inline void vDSP_fft_zopD(FFTSetupD setup, const DSPDoubleSplitComplex* in, int istride, DSPDoubleSplitComplex* out, int ostride, int log2n, int direction)
{
	(void)setup;
	const size_t n = static_cast<size_t>(1) << log2n;
	auto data = sapf::toComplex(in, istride, n);
	sapf::fft(data, direction == FFT_INVERSE || direction == kFFTDirection_Inverse);
	sapf::fromComplex(data, out, ostride);
}

inline void vDSP_fft_zipD(FFTSetupD setup, DSPDoubleSplitComplex* io, int stride, int log2n, int direction)
{
	vDSP_fft_zopD(setup, io, stride, io, stride, log2n, direction);
}

inline void vDSP_fft_zropD(FFTSetupD setup, const DSPDoubleSplitComplex* in, int istride, DSPDoubleSplitComplex* out, int ostride, int log2n, int direction)
{
	vDSP_fft_zopD(setup, in, istride, out, ostride, log2n, direction);
}

inline void vDSP_fft_zripD(FFTSetupD setup, DSPDoubleSplitComplex* io, int stride, int log2n, int direction)
{
	(void)setup;
	const size_t n = static_cast<size_t>(1) << log2n;
	std::vector<std::complex<double>> data(n);
	for (size_t i = 0; i < n; ++i) {
		data[i] = {0.0, 0.0};
	}
	const size_t half = n / 2;
	data[0] = {io->realp[0], 0.0};
	for (size_t k = 1; k < half; ++k) {
		const double re = io->realp[k * stride];
		const double im = io->imagp[k * stride];
		data[k] = {re, im};
		data[n - k] = {re, -im};
	}
	data[half] = {io->imagp[0], 0.0};

	sapf::fft(data, direction == FFT_INVERSE || direction == kFFTDirection_Inverse);

	for (size_t k = 0; k < half; ++k) {
		io->realp[k * stride] = data[k].real();
		io->imagp[k * stride] = data[k].imag();
	}
	// Pack DC in realp[0] and Nyquist in imagp[0] (vDSP packed format)
	// Note: realp[0] already set in loop above, imagp[0] gets Nyquist
	io->imagp[0] = data[half].real();
}

#endif // SAPF_HAS_ACCELERATE
