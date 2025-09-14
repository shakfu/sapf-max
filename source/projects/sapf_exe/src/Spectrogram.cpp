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

#include "Spectrogram.hpp"
#include "makeImage.hpp"
#include <Accelerate/Accelerate.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void makeColorTable(unsigned char* table);

static double bessi0(double x)
{
	//returns the modified Bessel function I_0(x) for any real x
	//from numerical recipes
	double ax, ans;
	double y;
	
	if((ax=fabs(x))<3.75){
		y=x/3.75;
		y *= y;
		ans =1.0+y*(3.5156229+y*(3.0899424+y*(1.2067492
			+y*(0.2659732+y*(0.360768e-1+y*0.45813e-2)))));
	}
	else{
		y=3.75/ax;
		ans = (exp(ax)/sqrt(ax))*(0.39894228+y*(0.1328592e-1
			+y*(0.225319e-2+y*(-0.157565e-2+y*(0.916281e-2
			+y*(-0.2057706e-1+y*(0.2635537e-1+y*(-0.1647633e-1
			+y*0.392377e-2))))))));
	}

	return ans;
}

static double i0(double x)
{
	const double epsilon = 1e-18;
	int n = 1;
	double S = 1., D = 1., T;

	while (D > epsilon * S) {
		T = x / (2 * n++);
		D *= T * T;
		S += D;
	}
	return S;
}

static void calcKaiserWindowD(size_t size, double* window, double stopBandAttenuation)
{
	size_t M = size - 1;
	size_t N = M-1;
#if VERBOSE
	printf("FillKaiser %d %g\n", M, stopBandAttenuation);
#endif

	double alpha = 0.;
	if (stopBandAttenuation <= -50.)
		alpha = 0.1102 * (-stopBandAttenuation - 8.7);
    else if (stopBandAttenuation < -21.)
		alpha = 0.5842 * pow(-stopBandAttenuation - 21., 0.4) + 0.07886 * (-stopBandAttenuation - 21.);

	double p = N / 2;
	double kk = 1.0 / i0(alpha);

	for(unsigned int k = 0; k < M; k++ )
	{
		double x = (k-p) / p;
		
		// Kaiser window
		window[k+1] *= kk * bessi0(alpha * sqrt(1.0 - x*x) );
	}
	window[0] = 0.;
	window[size-1] = 0.;
#if VERBOSE
	printf("done\n");
#endif
}

const int border = 8;

void spectrogram(int size, double* data, int width, int log2bins, const char* path, double dBfloor)
{
	int numRealFreqs = 1 << log2bins;

	int log2n = log2bins + 1;
	int n = 1 << log2n;
	int nOver2 = n / 2;
	

	double scale = 1./nOver2;
	
	int64_t paddedSize = size + n;
	double* paddedData = (double*)calloc(paddedSize, sizeof(double));
	memcpy(paddedData + nOver2, data, size * sizeof(double));

	
	double* dBMags = (double*)calloc(numRealFreqs + 1, sizeof(double));

	double hopSize = size <= n ? 0 : (double)(size - n) / (double)(width - 1);

	double* window = (double*)calloc(n, sizeof(double));
	for (int i = 0; i < n; ++i) window[i] = 1.;
	calcKaiserWindowD(n, window, -180.);

	unsigned char table[1028];
	makeColorTable(table);

	int heightOfAmplitudeView = 128;
	int heightOfFFT = numRealFreqs+1;
	int totalHeight = heightOfAmplitudeView+heightOfFFT+3*border;
	int topOfSpectrum = heightOfAmplitudeView + 2*border;
	int totalWidth = width+2*border;
	Bitmap* b = createBitmap(totalWidth, totalHeight);
	fillRect(b, 0, 0, totalWidth, totalHeight, 160, 160, 160, 255);
	fillRect(b, border, border, width, heightOfAmplitudeView, 0, 0, 0, 255);
	
	FFTSetupD fftSetup = vDSP_create_fftsetupD(log2n, kFFTRadix2);

	double* windowedData = (double*)calloc(n, sizeof(double));
	double* interleavedData = (double*)calloc(n, sizeof(double));
	double* resultData = (double*)calloc(n, sizeof(double));
	DSPDoubleSplitComplex interleaved;
	interleaved.realp = interleavedData;
	interleaved.imagp = interleavedData + nOver2;
	DSPDoubleSplitComplex result;
	result.realp = resultData;
	result.imagp = resultData + nOver2;
	double maxmag = 0.;
	
	double hpos = nOver2;
	for (int i = 0; i < width; ++i) {
		size_t ihpos = (size_t)hpos;
		
		// do analysis
		// find peak
		double peak = 1e-20;
		for (int w = 0; w < n; ++w) {
			double x = paddedData[w+ihpos];
			x = fabs(x);
			if (x > peak) peak = x;
		}
		
		for (int64_t w = 0; w < n; ++w) windowedData[w] = window[w] * paddedData[w+ihpos];
		
		vDSP_ctozD((DSPDoubleComplex*)windowedData, 2, &interleaved, 1, nOver2);
		
		vDSP_fft_zropD(fftSetup, &interleaved, 1, &result, 1, log2n, kFFTDirection_Forward);
		
		dBMags[0] = result.realp[0] * scale;
		dBMags[numRealFreqs] = result.imagp[0] * scale;
		if (dBMags[0] > maxmag) maxmag = dBMags[0];
		if (dBMags[numRealFreqs] > maxmag) maxmag = dBMags[numRealFreqs];
		for (int64_t j = 1; j < numRealFreqs-1; ++j) {
			double x = result.realp[j] * scale;
			double y = result.imagp[j] * scale;
			dBMags[j] = sqrt(x*x + y*y);
			if (dBMags[j] > maxmag) maxmag = dBMags[j];
		}

		double invmag = 1.;
		dBMags[0] = 20.*log2(dBMags[0]*invmag);
		dBMags[numRealFreqs] = 20.*log10(dBMags[numRealFreqs]*invmag);
		for (int64_t j = 0; j <= numRealFreqs-1; ++j) {
			dBMags[j] = 20.*log10(dBMags[j]*invmag);
		}
		
		
		
		// set pixels
		{
			double peakdB =  20.*log10(peak);
			int peakColorIndex = 256. - peakdB * (256. / dBfloor);
			int peakIndex = heightOfAmplitudeView - peakdB * (heightOfAmplitudeView / dBfloor);
			if (peakIndex < 0) peakIndex = 0;
			if (peakIndex > heightOfAmplitudeView) peakIndex = heightOfAmplitudeView;
			if (peakColorIndex < 0) peakColorIndex = 0;
			if (peakColorIndex > 255) peakColorIndex = 255;

			unsigned char* t = table + 4*peakColorIndex;
			fillRect(b, i+border, border+128-peakIndex, 1, peakIndex, t[0], t[1], t[2], t[3]);
		}
		
		for (int j = 0; j < numRealFreqs; ++j) {
			int colorIndex = 256. - dBMags[j] * (256. / dBfloor); 
			if (colorIndex < 0) colorIndex = 0;
			if (colorIndex > 255) colorIndex = 255;
			
			unsigned char* t = table + 4*colorIndex;
			
			setPixel(b, i+border, numRealFreqs-j+topOfSpectrum, t[0], t[1], t[2], t[3]);
		}
		
		hpos += hopSize;
	}

	vDSP_destroy_fftsetupD(fftSetup);
	
	writeBitmap(b, path);
	freeBitmap(b);
	free(dBMags);
	free(paddedData);
	free(window);
	free(windowedData);
	free(interleavedData);
	free(resultData);
}


static void makeColorTable(unsigned char* table)
{
	// white >> red >> yellow >> green >> cyan >> blue >> magenta >> pink >> black
	// 0        -20    -40       -60      -80     -100    -120       -140    -160
	// 255      224    192       160      128     96      64         32      0
	
	int colors[9][4] = {
		{  0,   0,  64, 255},	// dk blue
		{  0,   0, 255, 255},	// blue
		{255,   0,   0, 255},	// red
		{255, 255,   0, 255},	// yellow
		{255, 255, 255, 255}	// white
	};
	
	for (int j = 0; j < 4; ++j) {
		for (int i = 0; i < 64; ++i) {
			for (int k = 0; k < 4; ++k) {
				int x = (colors[j][k] * (64 - i) + colors[j+1][k] * i) / 64;
				if (x > 255) x = 255;
				table[j*64*4 + i*4 + k + 4] = x;
			}
		}
	}
	
	table[0] = 0;
	table[1] = 0;
	table[2] = 0;
	table[3] = 255;
}

