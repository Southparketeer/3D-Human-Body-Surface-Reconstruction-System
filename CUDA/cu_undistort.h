#pragma once 
#include "Image.h"
#include "ImageIntrinsics.h"
namespace loo
{

	struct distortParam
	{
		inline __host__ __device__	
			distortParam(): k1(0), k2(0), p1(0), p2(0), k3(0){}
		inline __host__ __device__	
			distortParam(float k1, float k2, float p1, float p2, float k3) : k1(k1), k2(k2), p1(p1), p2(p2), k3(k3)	{}
		inline __host__ __device__ void setdistortParam(distortParam& param)
		{
			k1 = param.k1;
			k2 = param.k2;
			p1 = param.p1;
			p2 = param.p2;
			k3 = param.k3;
		}
		float k1;
		float k2;
		float p1;
		float p2;
		float k3;
	};
	template<typename To, typename Ti>
	__declspec(dllexport)
		void DepthUndistort( Image<To> dOut, const Image<Ti> dIn, ImageIntrinsics K, distortParam dP);
}