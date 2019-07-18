#include "cu_undistort.h"
#include "launch_utils.h"
#include "InvalidValue.h"

namespace loo
{
	template<typename To, typename Ti>
	__global__ void DepthUndistortKern( Image<To> dOut, const Image<Ti> dIn, ImageIntrinsics K,  distortParam dP) 
	{
		const int w = dIn.Width();
		const int h = dIn.Height();
		const uint x = blockIdx.x*blockDim.x + threadIdx.x;
		const uint y = blockIdx.y*blockDim.y + threadIdx.y;
		const int2 coord = make_int2(x,y);
		int idx = coord.x + coord.y * w;
		//	get normalized coordinates
		float fx = K.fu;
		float fy = K.fv;
		float ox = K.u0;
		float oy = K.v0;

		//	projection -> camera
		float2 v = make_float2((coord.x - ox) / fx, (oy - coord.y) / fy);

		//	undistortion
		const float k1 = dP.k1;
		const float k2 = dP.k2;
		const float p1 = dP.p1;
		const float p2 = dP.p2;
		const float k3 = dP.k3;

		const float2 v2 =  make_float2(v.x, v.y);
		float r2 = (v2.x * v2.x) + (v2.y * v2.y);
		float r4 = r2 * r2;
		float r6 = r2 * r2 * r2;
		float coff = (1 + k1 * r2 + k2 * r4 + k3 * r6);
		float2 v3 = make_float2(0.0f, 0.0f);
		v3.x = v2.x * coff + (2 * p1 * v2.x * v2.y) + (p2 * (r2 + 2 * v2.x * v2.x));
		v3.y = v2.y * coff + (p1 * (r2 + 2 * v2.y * v2.y)) + (2 * p2 * v2.x * v2.y);

		const float2 newIdx = make_float2(v3.x * fx + ox, oy - v3.y * fy);
		if (newIdx.x > w || newIdx.x < 0 || newIdx.y > h || newIdx.y < 0)
		{
			dOut[idx] = 0.0f;
			return;
		}

		int idx2 = floor(newIdx.x) + floor(newIdx.y) * w;

		//	Undistored depth
		dOut[idx] = dIn[idx2];
	}

	template<typename To, typename Ti>
	void DepthUndistort( Image<To> dOut, const Image<Ti> dIn, ImageIntrinsics K,  distortParam dP) 
	{
			dim3 blockDim, gridDim;
			InitDimFromOutputImageOver(blockDim,gridDim, dOut);
			DepthUndistortKern<To,Ti><<<gridDim,blockDim>>>(dOut, dIn,K, dP);
	}

	template __declspec(dllexport) void DepthUndistort(Image<float>, const Image<float>, ImageIntrinsics, distortParam);
}
