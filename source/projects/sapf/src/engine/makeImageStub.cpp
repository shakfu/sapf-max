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

// Cross-platform bitmap implementation using BMP file format
// Note: Output is .bmp instead of .jpg on non-Apple platforms

#if !defined(__APPLE__)

#include "makeImage.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

struct Bitmap
{
	int width;
	int height;
	int bytesPerRow;
	unsigned char* data;  // RGBA format, 4 bytes per pixel
};

Bitmap* createBitmap(int width, int height)
{
	Bitmap* bitmap = (Bitmap*)calloc(1, sizeof(Bitmap));
	if (!bitmap) return nullptr;

	bitmap->width = width;
	bitmap->height = height;
	bitmap->bytesPerRow = width * 4;
	bitmap->data = (unsigned char*)calloc(width * height * 4, 1);

	if (!bitmap->data) {
		free(bitmap);
		return nullptr;
	}

	return bitmap;
}

void setPixel(Bitmap* bitmap, int x, int y, int r, int g, int b, int a)
{
	if (!bitmap || !bitmap->data) return;
	if (x < 0 || x >= bitmap->width || y < 0 || y >= bitmap->height) return;

	size_t index = (size_t)bitmap->bytesPerRow * y + 4 * x;
	bitmap->data[index + 0] = (unsigned char)r;
	bitmap->data[index + 1] = (unsigned char)g;
	bitmap->data[index + 2] = (unsigned char)b;
	bitmap->data[index + 3] = (unsigned char)a;
}

void fillRect(Bitmap* bitmap, int x, int y, int width, int height, int r, int g, int b, int a)
{
	if (!bitmap || !bitmap->data) return;

	for (int j = y; j < y + height && j < bitmap->height; ++j) {
		if (j < 0) continue;
		for (int i = x; i < x + width && i < bitmap->width; ++i) {
			if (i < 0) continue;
			size_t index = (size_t)bitmap->bytesPerRow * j + 4 * i;
			bitmap->data[index + 0] = (unsigned char)r;
			bitmap->data[index + 1] = (unsigned char)g;
			bitmap->data[index + 2] = (unsigned char)b;
			bitmap->data[index + 3] = (unsigned char)a;
		}
	}
}

// Write a 24-bit BMP file (standard format supported everywhere)
void writeBitmap(Bitmap* bitmap, const char *path)
{
	if (!bitmap || !bitmap->data) {
		fprintf(stderr, "writeBitmap: Invalid bitmap\n");
		return;
	}

	// Change extension from .wav/.jpg to .bmp if present
	char bmpPath[1024];
	strncpy(bmpPath, path, sizeof(bmpPath) - 1);
	bmpPath[sizeof(bmpPath) - 1] = '\0';

	char* ext = strrchr(bmpPath, '.');
	if (ext && (strcmp(ext, ".jpg") == 0 || strcmp(ext, ".jpeg") == 0 || strcmp(ext, ".wav") == 0)) {
		strcpy(ext, ".bmp");
	} else if (!ext) {
		strncat(bmpPath, ".bmp", sizeof(bmpPath) - strlen(bmpPath) - 1);
	}

	FILE* f = fopen(bmpPath, "wb");
	if (!f) {
		fprintf(stderr, "writeBitmap: Cannot open file '%s'\n", bmpPath);
		return;
	}

	int width = bitmap->width;
	int height = bitmap->height;

	// BMP rows must be padded to 4-byte boundaries
	int rowSize = ((width * 3 + 3) / 4) * 4;
	int imageSize = rowSize * height;
	int fileSize = 54 + imageSize;  // 54 = header size

	// BMP file header (14 bytes)
	uint8_t fileHeader[14] = {
		'B', 'M',           // Signature
		0, 0, 0, 0,         // File size (filled below)
		0, 0, 0, 0,         // Reserved
		54, 0, 0, 0         // Offset to pixel data
	};
	fileHeader[2] = (uint8_t)(fileSize);
	fileHeader[3] = (uint8_t)(fileSize >> 8);
	fileHeader[4] = (uint8_t)(fileSize >> 16);
	fileHeader[5] = (uint8_t)(fileSize >> 24);

	// BMP info header (40 bytes)
	uint8_t infoHeader[40] = {0};
	infoHeader[0] = 40;  // Header size
	infoHeader[4] = (uint8_t)(width);
	infoHeader[5] = (uint8_t)(width >> 8);
	infoHeader[6] = (uint8_t)(width >> 16);
	infoHeader[7] = (uint8_t)(width >> 24);
	infoHeader[8] = (uint8_t)(height);
	infoHeader[9] = (uint8_t)(height >> 8);
	infoHeader[10] = (uint8_t)(height >> 16);
	infoHeader[11] = (uint8_t)(height >> 24);
	infoHeader[12] = 1;   // Color planes
	infoHeader[14] = 24;  // Bits per pixel
	infoHeader[20] = (uint8_t)(imageSize);
	infoHeader[21] = (uint8_t)(imageSize >> 8);
	infoHeader[22] = (uint8_t)(imageSize >> 16);
	infoHeader[23] = (uint8_t)(imageSize >> 24);

	fwrite(fileHeader, 1, 14, f);
	fwrite(infoHeader, 1, 40, f);

	// Write pixel data (BMP is bottom-up, BGR order)
	unsigned char* row = (unsigned char*)malloc(rowSize);
	if (!row) {
		fclose(f);
		return;
	}

	for (int y = height - 1; y >= 0; --y) {
		memset(row, 0, rowSize);
		for (int x = 0; x < width; ++x) {
			size_t srcIdx = (size_t)bitmap->bytesPerRow * y + 4 * x;
			size_t dstIdx = x * 3;
			row[dstIdx + 0] = bitmap->data[srcIdx + 2];  // B
			row[dstIdx + 1] = bitmap->data[srcIdx + 1];  // G
			row[dstIdx + 2] = bitmap->data[srcIdx + 0];  // R
		}
		fwrite(row, 1, rowSize, f);
	}

	free(row);
	fclose(f);
}

void freeBitmap(Bitmap* bitmap)
{
	if (bitmap) {
		free(bitmap->data);
		free(bitmap);
	}
}

#endif // !__APPLE__
