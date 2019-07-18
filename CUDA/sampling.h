#pragma once

#include <cuda_runtime.h>
#include "cutil_math.h"

//////////////////////////////////////////////////////
// Additional lerp definitions for integral vector types
//////////////////////////////////////////////////////

inline __device__ __host__ float lerp(unsigned char a, unsigned char b, float t)
{
	return (float)a + t*((float)b-(float)a);
}

inline __device__ __host__ float2 lerp(uchar2 a, uchar2 b, float t)
{
	return make_float2(
		a.x + t*(b.x-a.x),
		a.y + t*(b.y-a.y)
		);
}

inline __device__ __host__ float3 lerp(uchar3 a, uchar3 b, float t)
{
	return make_float3(
		a.x + t*(b.x-a.x),
		a.y + t*(b.y-a.y),
		a.z + t*(b.z-a.z)
		);
}

inline __device__ __host__ float4 lerp(uchar4 a, uchar4 b, float t)
{
	return make_float4(
		a.x + t*(b.x-a.x),
		a.y + t*(b.y-a.y),
		a.z + t*(b.z-a.z),
		a.w + t*(b.w-a.w)
		);
}

namespace loo {

	//////////////////////////////////////////////////////
	// Sampling
	//////////////////////////////////////////////////////

	// w0, w1, w2, and w3 are the four cubic B-spline basis functions
	__host__ __device__ inline
		float w0(float a)
	{
		//    return (1.0f/6.0f)*(-a*a*a + 3.0f*a*a - 3.0f*a + 1.0f);
		return (1.0f/6.0f)*(a*(a*(-a + 3.0f) - 3.0f) + 1.0f);   // optimized
	}

	__host__ __device__ inline
		float w1(float a)
	{
		//    return (1.0f/6.0f)*(3.0f*a*a*a - 6.0f*a*a + 4.0f);
		return (1.0f/6.0f)*(a*a*(3.0f*a - 6.0f) + 4.0f);
	}

	__host__ __device__ inline
		float w2(float a)
	{
		//    return (1.0f/6.0f)*(-3.0f*a*a*a + 3.0f*a*a + 3.0f*a + 1.0f);
		return (1.0f/6.0f)*(a*(a*(-3.0f*a + 3.0f) + 3.0f) + 1.0f);
	}

	__host__ __device__ inline
		float w3(float a)
	{
		return (1.0f/6.0f)*(a*a*a);
	}

	// g0 and g1 are the two amplitude functions
	__host__ __device__ inline
		float g0(float a)
	{
		return w0(a) + w1(a);
	}

	__host__ __device__ inline
		float g1(float a)
	{
		return w2(a) + w3(a);
	}

	// h0 and h1 are the two offset functions
	__host__ __device__ inline
		float h0(float a)
	{
		// note +0.5 offset to compensate for CUDA linear filtering convention
		return -1.0f + w1(a) / (w0(a) + w1(a)) + 0.5f;
	}

	__host__ __device__ inline
		float h1(float a)
	{
		return 1.0f + w3(a) / (w2(a) + w3(a)) + 0.5f;
	}

	// filter 4 values using cubic splines
	template<typename R, typename T> inline
		__host__ __device__
		R cubicFilter(float x, T c0, T c1, T c2, T c3)
	{
		R r;
		r = c0 * w0(x);
		r += c1 * w1(x);
		r += c2 * w2(x);
		r += c3 * w3(x);
		return r;
	}

	// Catmull-Rom interpolation

	__host__ __device__ inline
		float catrom_w0(float a)
	{
		//return -0.5f*a + a*a - 0.5f*a*a*a;
		return a*(-0.5f + a*(1.0f - 0.5f*a));
	}

	__host__ __device__ inline
		float catrom_w1(float a)
	{
		//return 1.0f - 2.5f*a*a + 1.5f*a*a*a;
		return 1.0f + a*a*(-2.5f + 1.5f*a);
	}

	__host__ __device__ inline
		float catrom_w2(float a)
	{
		//return 0.5f*a + 2.0f*a*a - 1.5f*a*a*a;
		return a*(0.5f + a*(2.0f - 1.5f*a));
	}

	__host__ __device__ inline
		float catrom_w3(float a)
	{
		//return -0.5f*a*a + 0.5f*a*a*a;
		return a*a*(-0.5f + 0.5f*a);
	}

	template<typename R, typename T>
	__host__ __device__ inline
		R catRomFilter(float x, T c0, T c1, T c2, T c3)
	{
		R r;
		r = c0 * catrom_w0(x);
		r += c1 * catrom_w1(x);
		r += c2 * catrom_w2(x);
		r += c3 * catrom_w3(x);
		return r;
	}

	// (x,y) are continuous coordinates (top-left of top-left pixel at 0,0)
	template<typename R, typename T> inline
		__host__ __device__ R nearestneighbour_continuous(const T* img, size_t stride, float x, float y)
	{
		const int xi = floor(x);
		const int yi = floor(y);
		return img[xi + stride*yi];
	}

	// (x,y) are continuous coordinates (top-left of top-left pixel at 0,0)
	template<typename R, typename T> inline
		__host__ __device__ R nearestneighbour_discrete(const T* img, size_t stride, float x, float y)
	{
		return nearestneighbour_continuous<R,T>(x+0.5, y+0.5);
	}

	// (x,y) are continuous coordinates (top-left of top-left pixel at 0,0)
	template<typename R, typename T> inline
		__host__ __device__ R bilinear_discrete(const T* img, size_t stride, float px, float py)
	{
		//  if( 0.0 <= px && px < w-1.0 && 0.0 <= py && py < h-1.0 ) {
		const float ix = floorf(px);
		const float iy = floorf(py);
		const float fx = px - ix;
		const float fy = py - iy;
		const int idx = (int)ix + (int)iy*stride;

		return lerp(
			lerp( img[idx], img[idx+1], fx ),
			lerp( img[idx+stride], img[idx+stride+1], fx ),
			fy
			);
		//  }else{
		//    return nearestneighbour(img,stride,w,h,x,y);
		//  }
	}

	// (x,y) are continuous coordinates (top-left of top-left pixel at 0,0)
	template<typename R, typename T> inline
		__host__ __device__ R bilinear_continuous(const T* img, size_t stride, float x, float y)
	{
		return bilinear_discrete<R,T>(img, stride,x-0.5, y-0.5);
	}

	// (px,py) are pixel coordinates (center of top-left pixel at 0,0)
	template<typename R, typename T> inline
		__host__ __device__ R bicubic_discrete(const T* img, size_t stride, float px, float py)
	{
		//  if( 1.0 <= px && px < w-2.0 && 1.0 <= py && py < h-2.0 ) {
		const int ix = floor(px);
		const int iy = floor(py);
		const float fx = px - ix;
		const float fy = py - iy;
		const int idx = ((int)ix) + ((int)iy)*stride;

		return cubicFilter<R,R>(
			fy,
			cubicFilter<R,T>(fx, img[idx-stride-1], img[idx-stride], img[idx-stride+1], img[idx-stride+2]),
			cubicFilter<R,T>(fx, img[idx-1], img[idx], img[idx+1], img[idx+2]),
			cubicFilter<R,T>(fx, img[idx+stride-1], img[idx+stride], img[idx+stride+1], img[idx+stride+2]),
			cubicFilter<R,T>(fx, img[idx+2*stride-1], img[idx+2*stride], img[idx+2*stride+1], img[idx+2*stride+2])
			);
		//  }else{
		//    return nearestneighbour(img,stride,w,h,x,y);
		//  }
	}

	// (x,y) are continuous coordinates (top-left of top-left pixel at 0,0)
	template<typename R, typename T> inline
		__host__ __device__ R bicubic_continuous(const T* img, size_t stride, float x, float y)
	{
		return bicubic_discrete<R,T>(img, stride, x-0.5, y-0.5);
	}

	// (px,py) are pixel coordinates (center of top-left pixel at 0,0)
	template<typename R, typename T> inline
		__host__ __device__ R catrom_discrete(const T* img, size_t stride, float px, float py)
	{
		//  if( 1.0 <= px && px < w-2.0 && 1.0 <= py && py < h-2.0 ) {
		const int ix = floor(px);
		const int iy = floor(py);
		const float fx = px - ix;
		const float fy = py - iy;
		const unsigned idx = ((unsigned)ix) + ((unsigned)iy)*stride;
		const unsigned stride2 = 2 *stride;

		return catRomFilter<R,R>(
			fy,
			catRomFilter<R,T>(fx, img[idx-stride-1], img[idx-stride], img[idx-stride+1], img[idx-stride+2]),
			catRomFilter<R,T>(fx, img[idx-1], img[idx], img[idx+1], img[idx+2]),
			catRomFilter<R,T>(fx, img[idx+stride-1], img[idx+stride], img[idx+stride+1], img[idx+stride+2]),
			catRomFilter<R,T>(fx, img[idx+stride2-1], img[idx+stride2], img[idx+stride2+1], img[idx+stride2+2])
			);
		//  }else{
		//    return nearestneighbour<R,T>(img,stride,x,y);
		//  }
	}

	// (x,y) are continuous coordinates (top-left of top-left pixel at 0,0)
	template<typename R, typename T>
	__device__ R catrom_continuous(const T* img, size_t stride, float x, float y)
	{
		return catrom_discrete<R,T>(img, stride, x-0.5, y-0.5);
	}

	template<typename T, typename R>
	R Bilinear(T img, float px, float py)
	{
		const float ix = floorf(px);
		const float iy = floorf(py);
		const float fx = px - ix;
		const float fy = py - iy;
		const int ixp = ix+1;
		const int iyp = iy+1;

		return lerp(
			lerp( img(ix,iy), img(ixp,iy), fx ),
			lerp( img(ix,iyp), img(ixp,iyp), fx ),
			fy
			);
	}


}
