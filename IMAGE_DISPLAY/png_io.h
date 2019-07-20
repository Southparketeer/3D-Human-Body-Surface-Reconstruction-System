#pragma once
#include "lodepng.h"
#include <assert.h>
#include <iostream>
#include "ppl.h"
class PNG_IO
{
public:
	static void  PNG_to_array_depth(const char * s_name, _Out_cap_c_(w * h) float* depth,  int w, int h)
	{
		std::vector<unsigned char> image; //the raw pixels
		unsigned width, height;
		unsigned error = lodepng::decode(image, width, height, s_name, LCT_GREY, 16);
		assert(width == w);
		assert(height == h);
		if(error)
		{
			std::cout << "error " << error << ": " << lodepng_error_text(error) << std::endl;
			return;
		}

		Concurrency::parallel_for( 0, w * h, [&](int k){
			float d = (float)(image[k * 2] * 256 + image[k * 2 + 1]);
			depth[k] = ( d  > 3000 || d  < 200 ) ? UINT16_MAX : d;
		});
		return;
	}

	static void  PNG_to_array_BYTE(const char * s_name, _Out_cap_c_(w * h * 3) unsigned char* img,  int w, int h, bool if_flip_x = false)
	{
		std::vector<unsigned char> image; //the raw pixels
		unsigned width, height;
		unsigned error = lodepng::decode(image, width, height, s_name, LCT_RGB, 8);
		assert(width == w);
		assert(height == h);
		if(error)
		{
			std::cout << "error " << error << ": " << lodepng_error_text(error) << std::endl;
			return;
		}

		Concurrency::parallel_for( 0, w * h, [&](int k){
			int x = k % w;
			int y = k / w;
			int base = if_flip_x ? (w - 1 - x + y * w) : x + y * w;
			img[base * 3 + 0] = image[k * 3 + 0];
			img[base * 3 + 1] = image[k * 3 + 1];
			img[base * 3 + 2] = image[k * 3 + 2];
		});
		return;
	}

	static void  PNG_to_array_BYTE(const char * s_name, vector<Point3f>& image3f,  int w, int h)
	{
		std::vector<unsigned char> image; //the raw pixels
		unsigned width, height;
		unsigned error = lodepng::decode(image, width, height, s_name, LCT_RGB, 8);
		assert(width == w);
		assert(height == h);
		if(error)
		{
			std::cout << "error " << error << ": " << lodepng_error_text(error) << std::endl;
			return;
		}

		image3f.resize(w * h);
		Concurrency::parallel_for( 0, w * h, [&](int k){
			image3f[k].X() = image[k * 3 + 0];
			image3f[k].Y() = image[k * 3 + 1];
			image3f[k].Z() = image[k * 3 + 2];
		});
		return;
	}


	static void  PNG_to_array_depth(const char * s_name, _Out_cap_c_(w * h) float* depth,  int w, int h, int clip_x_min, int clip_x_max, int clip_y_min, int clip_y_max, bool if_flip_x = false, float clip_near = 200, float clip_far = 3000, int2 bad_pixel = make_int2(-1, -1))
	{
		std::vector<unsigned char> image; //the raw pixels
		unsigned width, height;
		unsigned error = lodepng::decode(image, width, height, s_name, LCT_GREY, 16);
		assert(width == w);
		assert(height == h);
		if(error)
		{
			std::cout << "error " << error << ": " << lodepng_error_text(error) << std::endl;
			return;
		}

		if(bad_pixel.x > 0 && bad_pixel.y > 0)
		{
			float d = 0;
			d += 0.25 * depth[bad_pixel.x + 1 + bad_pixel.y * width];
			d += 0.25 * depth[bad_pixel.x - 1 + bad_pixel.y * width];
			d += 0.25 * depth[bad_pixel.x + (bad_pixel.y + 1) * width];
			d += 0.25 * depth[bad_pixel.x + (bad_pixel.y - 1) * width];
			depth[bad_pixel.x + bad_pixel.y * width] = d;
		}

		Concurrency::parallel_for( 0, w * h, [&](int k){

			int x = k % w;
			int y = k / w;

			int base = if_flip_x ? (w - 1 - x + y * w) : x + y * w;

			if(x >= clip_x_min && x <= clip_x_max && y >= clip_y_min && y <= clip_y_max) 
			{ 
				float d = (float)(image[k * 2] * 256 + image[k * 2 + 1]);
				depth[base] = ( d  > clip_far || d  < clip_near ) ? UINT16_MAX : d;
			}
			else
			{
				depth[base] = UINT16_MAX; 
				return;
			}
		});


		return;
	}

	static void  depth_array_to_PNG(_Out_cap_c_(w * h) float* depth,  const char * s_name, int w, int h, int scale = 1., float F = 3000, float N = 500)
	{
		std::vector<unsigned char> image(w * h * 2);
		Concurrency::parallel_for( 0, w * h, [&](int k){
			int d = ceil(depth[k] * scale);
			if( d  > F || d  < N ) 
			{
				image[k * 2 + 0] = 0;
				image[k * 2 + 1] = 0;
			}
			else
			{
				image[k * 2 + 1] = d % 256;
				image[k * 2 + 0] = d >> 8;
			}
		});		
		std::vector<unsigned char> png;
		unsigned error = lodepng::encode(png, image, w, h, LCT_GREY, 16);
		lodepng::save_file(png, s_name);
	}

	static void  depth_array_to_PNG(unsigned char* img,  const char * s_name, int w, int h)
	{
		std::vector<unsigned char> image(w * h * 4);
		Concurrency::parallel_for( 0, w * h, [&](int k){
			image[k * 4 + 0] = img[k * 4 + 0];
			image[k * 4 + 1] = img[k * 4 + 1];
			image[k * 4 + 2] = img[k * 4 + 2];
			image[k * 4 + 3] = 255;
		});		
		std::vector<unsigned char> png;
		unsigned error = lodepng::encode(png, image, w, h, LCT_RGBA, 8);
		lodepng::save_file(png, s_name);
	}

	static void  depth_array_to_PNG3(unsigned char* img,  const char * s_name, int w, int h)
	{
		std::vector<unsigned char> image(w * h * 4);
		Concurrency::parallel_for( 0, w * h, [&](int k){
			image[k * 4 + 0] = img[k * 3 + 0];
			image[k * 4 + 1] = img[k * 3 + 1];
			image[k * 4 + 2] = img[k * 3 + 2];
			image[k * 4 + 3] = 255;
		});		
		std::vector<unsigned char> png;
		unsigned error = lodepng::encode(png, image, w, h, LCT_RGBA, 8);
		lodepng::save_file(png, s_name);
	}

	inline static Color4b getColor(const int & k)
	{
		switch(k)
		{
		case 0:
			return Color4b::Black;
		case 1:
			return Color4b::White;
		case 2: 
			return Color4b::Red;
		case 3:
			return Color4b::Green;
		case 4:
			return Color4b::DarkGreen;
		case 5:
			return Color4b::Cyan;
		case 6:
			return Color4b::Yellow;
		case 7: 
			return Color4b::Blue;
		case 8:
			return Color4b::Magenta;
		default:
			return Color4b::Black;
		}
	}
	static void  BYTE_Mark8_array_to_PNG(_Out_cap_c_(w * h) unsigned char* img,  const char * s_name, int w, int h)
	{
		std::vector<unsigned char> image(w * h * 4);
		Concurrency::parallel_for( 0, w * h, [&](int k){
			Color4b c = getColor(img[k]);
			image[k * 4 + 0] = c.X();
			image[k * 4 + 1] = c.Y();
			image[k * 4 + 2] = c.Z();
			image[k * 4 + 3] = 255;
		});		
		std::vector<unsigned char> png;
		unsigned error = lodepng::encode(png, image, w, h, LCT_RGBA, 8);
		lodepng::save_file(png, s_name);
	}

	static void  BYTE_array_to_PNG(_Out_cap_c_(w * h * 3) unsigned char* img,  const char * s_name, int w, int h)
	{
		std::vector<unsigned char> image(w * h * 4);
		Concurrency::parallel_for( 0, w * h, [&](int k){
			image[k * 4 + 0] = img[k * 3 + 0];
			image[k * 4 + 1] = img[k * 3 + 1];
			image[k * 4 + 2] = img[k * 3 + 2];
			image[k * 4 + 3] = 255;
		});		
		std::vector<unsigned char> png;
		unsigned error = lodepng::encode(png, image, w, h, LCT_RGBA, 8);
		lodepng::save_file(png, s_name);
	}
};












///*
//LodePNG Examples
//
//Copyright (c) 2005-2010 Lode Vandevenne
//
//This software is provided 'as-is', without any express or implied
//warranty. In no event will the authors be held liable for any damages
//arising from the use of this software.
//
//Permission is granted to anyone to use this software for any purpose,
//including commercial applications, and to alter it and redistribute it
//freely, subject to the following restrictions:
//
//1. The origin of this software must not be misrepresented; you must not
//claim that you wrote the original software. If you use this software
//in a product, an acknowledgment in the product documentation would be
//appreciated but is not required.
//
//2. Altered source versions must be plainly marked as such, and must not be
//misrepresented as being the original software.
//
//3. This notice may not be removed or altered from any source
//distribution.
//*/
//
///*
//Load a BMP image and convert it to a PNG image. This example also shows how
//to use other data with the same memory structure as BMP, such as the image
//format native to win32, GDI (HBITMAP, BITMAPINFO, ...) often encountered if
//you're programming for Windows in Visual Studio.
//
//This example only supports uncompressed 24-bit RGB or 32-bit RGBA bitmaps.
//For other types of BMP's, use a full fledged BMP decoder, or convert the
//bitmap to 24-bit or 32-bit format.
//
//NOTE: it overwrites the output file without warning if it exists!
//*/
//
////g++ lodepng.cpp example_bmp2png.cpp -ansi -pedantic -Wall -Wextra -O3
//
//#include "lodepng.h"
//
//#include <iostream>
//
////returns 0 if all went ok, non-0 if error
////output image is always given in RGBA (with alpha channel), even if it's a BMP without alpha channel
//unsigned decodeBMP(std::vector<unsigned char>& image, unsigned& w, unsigned& h, const std::vector<unsigned char>& bmp)
//{
//	static const unsigned MINHEADER = 54; //minimum BMP header size
//
//	if(bmp.size() < MINHEADER) return -1;
//	if(bmp[0] != 'B' || bmp[1] != 'M') return 1; //It's not a BMP file if it doesn't start with marker 'BM'
//	unsigned pixeloffset = bmp[10] + 256 * bmp[11]; //where the pixel data starts
//	//read width and height from BMP header
//	w = bmp[18] + bmp[19] * 256;
//	h = bmp[22] + bmp[23] * 256;
//	//read number of channels from BMP header
//	if(bmp[28] != 24 && bmp[28] != 32) return 2; //only 24-bit and 32-bit BMPs are supported.
//	unsigned numChannels = bmp[28] / 8;
//
//	//The amount of scanline bytes is width of image times channels, with extra bytes added if needed
//	//to make it a multiple of 4 bytes.
//	unsigned scanlineBytes = w * numChannels;
//	if(scanlineBytes % 4 != 0) scanlineBytes = (scanlineBytes / 4) * 4 + 4;
//
//	unsigned dataSize = scanlineBytes * h;
//	if(bmp.size() < dataSize + pixeloffset) return 3; //BMP file too small to contain all pixels
//
//	image.resize(w * h * 4);
//
//	/*
//	There are 3 differences between BMP and the raw image buffer for LodePNG:
//	-it's upside down
//	-it's in BGR instead of RGB format (or BRGA instead of RGBA)
//	-each scanline has padding bytes to make it a multiple of 4 if needed
//	The 2D for loop below does all these 3 conversions at once.
//	*/
//	for(unsigned y = 0; y < h; y++)
//		for(unsigned x = 0; x < w; x++)
//		{
//			//pixel start byte position in the BMP
//			unsigned bmpos = pixeloffset + (h - y - 1) * scanlineBytes + numChannels * x;
//			//pixel start byte position in the new raw image
//			unsigned newpos = 4 * y * w + 4 * x;
//			if(numChannels == 3)
//			{
//				image[newpos + 0] = bmp[bmpos + 2]; //R
//				image[newpos + 1] = bmp[bmpos + 1]; //G
//				image[newpos + 2] = bmp[bmpos + 0]; //B
//				image[newpos + 3] = 255;            //A
//			}
//			else
//			{
//				image[newpos + 0] = bmp[bmpos + 3]; //R
//				image[newpos + 1] = bmp[bmpos + 2]; //G
//				image[newpos + 2] = bmp[bmpos + 1]; //B
//				image[newpos + 3] = bmp[bmpos + 0]; //A
//			}
//		}
//		return 0;
//}
//
//int main(int argc, char *argv[])
//{
//	if(argc < 3)
//	{
//		std::cout << "Please provice input PNG and output BMP file names" << std::endl;
//		return 0;
//	}
//
//	std::vector<unsigned char> bmp;
//	lodepng::load_file(bmp, argv[1]);
//	std::vector<unsigned char> image;
//	unsigned w, h;
//	unsigned error = decodeBMP(image, w, h, bmp);
//
//	if(error)
//	{
//		std::cout << "BMP decoding error " << error << std::endl;
//		return 0;
//	}
//
//	std::vector<unsigned char> png;
//	error = lodepng::encode(png, image, w, h);
//
//	if(error)
//	{
//		std::cout << "PNG encoding error " << error << ": " << lodepng_error_text(error) << std::endl;
//		return 0;
//	}
//
//	lodepng::save_file(png, argv[2]);
//
//}