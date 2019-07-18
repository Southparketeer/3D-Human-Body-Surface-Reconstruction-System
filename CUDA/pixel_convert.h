#pragma once

#include <cuda_runtime.h>

namespace loo
{

	template<typename To, typename Ti>
	__host__ __device__ inline
		To ConvertPixel(Ti p)
	{
		return p;
	}

	template<>
	__host__ __device__ inline
		uchar4 ConvertPixel(unsigned char p)
	{
		return make_uchar4(p,p,p,255);
	}

	template<>
	__host__ __device__ inline
		uchar3 ConvertPixel(unsigned char p)
	{
		return make_uchar3(p,p,p);
	}

	template<>
	__host__ __device__ inline
		unsigned char ConvertPixel(uchar3 p)
	{
		const unsigned sum = p.x + p.y + p.z;
		return sum / 3;
	}

	template<>
	__host__ __device__ inline
		unsigned char ConvertPixel(uchar4 p)
	{
		const unsigned sum = p.x + p.y + p.z;
		return sum / 3;
	}

	template<>
	__host__ __device__ inline
		uchar4 ConvertPixel(uchar3 p)
	{
		return make_uchar4(p.x,p.y,p.z,255);
	}

	template<>
	__host__ __device__ inline
		uchar3 ConvertPixel(uint3 p)
	{
		return make_uchar3(
			(unsigned char)(p.x),
			(unsigned char)(p.y),
			(unsigned char)(p.z)
			);
	}

	template<>
	__host__ __device__ inline
		uint3 ConvertPixel(uchar3 p)
	{
		return make_uint3(
			(unsigned int)(p.x),
			(unsigned int)(p.y),
			(unsigned int)(p.z)
			);
	}

	template<>
	__host__ __device__ inline
		uchar4 ConvertPixel(uint4 p)
	{
		return make_uchar4(
			(unsigned char)(p.x),
			(unsigned char)(p.y),
			(unsigned char)(p.z),
			(unsigned char)(p.w)
			);
	}

	template<>
	__host__ __device__ inline
		uint4 ConvertPixel(uchar4 p)
	{
		return make_uint4(
			(unsigned int)(p.x),
			(unsigned int)(p.y),
			(unsigned int)(p.z),
			(unsigned int)(p.w)
			);
	}

	template<>
	__host__ __device__ inline
		uchar4 ConvertPixel(float4 p)
	{
		return make_uchar4(
			(unsigned char)(p.x*255.0f),
			(unsigned char)(p.y*255.0f),
			(unsigned char)(p.z*255.0f),
			(unsigned char)(p.w*255.0f)
			);
	}

	template<>
	__host__ __device__ inline
		uchar3 ConvertPixel(uchar4 p)
	{
		return make_uchar3(p.x,p.y,p.z);
	}

	template<>
	__host__ __device__ inline
		float4 ConvertPixel(float p)
	{
		return make_float4(p,p,p,1.0f);
	}

	template<>
	__host__ __device__ inline
		float3 ConvertPixel(uchar3 p)
	{
		return make_float3(p.x,p.y,p.z);
	}

	template<>
	__host__ __device__ inline
		float ConvertPixel(uchar3 p)
	{
		return (p.x+p.y+p.z) / (3.0f*255.0f);
	}

	template<>
	__host__ __device__ inline
		float4 ConvertPixel(uchar4 p)
	{
		return make_float4(p.x/255.0f,p.y/255.0f,p.z/255.0f,p.w);
	}

	template<>
	__host__ __device__ inline
		float4 ConvertPixel(uchar3 p)
	{
		return make_float4(p.x/255.0f,p.y/255.0f,p.z/255.0f,1.0);
	}

	template<>
	__host__ __device__ inline
		float3 ConvertPixel(float p)
	{
		return make_float3(p,p,p);
	}

	template<>
	__host__ __device__ inline
		float ConvertPixel(float3 p)
	{
		return (p.x + p.y + p.z) / 3.0f;
	}

}
