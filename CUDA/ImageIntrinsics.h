#pragma once

#include "Image.h"
#include "MatUtils.h"

#ifndef __CUDACC__
#ifdef HAVE_EIGEN
#define USE_EIGEN
#endif // HAVE_EIGEN
#endif // __CUDACC__

#ifdef USE_EIGEN
#include <Eigen/Dense>
#endif // USE_EIGEN

namespace loo
{

#ifdef HAVE_NPP
	inline NppiRect GetTopLeftAlignedRegion(int w, int h, int blockx, int blocky)
	{
		NppiRect ret;
		ret.width = blockx * (w / blockx);
		ret.height = blocky * (h / blocky);
		ret.x = 0;
		ret.y = 0;
		return ret;
	}

	inline NppiRect GetCenteredAlignedRegion(int w, int h, int blockx, int blocky)
	{
		NppiRect ret;
		ret.width = blockx * (w / blockx);
		ret.height = blocky * (h / blocky);
		ret.x = (w - ret.width) / 2;
		ret.y = (h - ret.height) / 2;
		return ret;
	}
#endif // HAVE_NPP

	inline int GetLevelFromMaxPixels(size_t w, size_t h, unsigned long maxpixels)
	{
		size_t level = 0;
		while( (w >> level)*(h >> level) > maxpixels ) {
			++level;
		}
		return level;
	}


	struct ImageIntrinsics
	{

		//////////////////////////////////////////////////////
		// Constructors
		//////////////////////////////////////////////////////

		inline __host__ __device__
			ImageIntrinsics()
			: fu(0), fv(0), u0(0), v0(0)
		{
		}

		inline __host__ __device__
			ImageIntrinsics(float fu, float fv, float u0, float v0)
			: fu(fu), fv(fv), u0(u0), v0(v0)
		{
		}

		inline __host__ __device__
			ImageIntrinsics(float f, float u0, float v0)
			: fu(f), fv(f), u0(u0), v0(v0)
		{
		}

		template<typename T, typename Target, typename Manage>
		inline __host__ __device__
			ImageIntrinsics(float f, const Image<T,Target,Manage>& img)
			: fu(f), fv(f), u0(img.w/2.0f - 0.5), v0(img.h/2.0f - 0.5)
		{
		}

		//////////////////////////////////////////////////////
		// Image projection
		//////////////////////////////////////////////////////

		inline __host__ __device__
			float2 Project(const float3 P_c) const
		{
			return make_float2(u0 + fu*P_c.x/P_c.z, v0 + fv*P_c.y/P_c.z);
		}

		inline __host__ __device__
			float2 Project(float x, float y, float z) const
		{
			return make_float2(u0 + fu*x/z, v0 + fv*y/z);
		}

		inline __host__ __device__
			float2 operator*(float3 P_c) const
		{
			return Project(P_c);
		}

		//////////////////////////////////////////////////////
		// Image Unprojection
		//////////////////////////////////////////////////////

		inline __host__ __device__
			float3 Unproject(float u, float v) const
		{
			return make_float3((u-u0)/fu,(v-v0)/fv, 1);
		}

		inline __host__ __device__
			float3 Unproject(const float2 p_c) const
		{
			return make_float3((p_c.x-u0)/fu,(p_c.y-v0)/fv, 1);
		}

		inline __host__ __device__
			float3 Unproject(const float2 p_c, float z) const
		{
			return make_float3(z*(p_c.x-u0)/fu,z*(p_c.y-v0)/fv, z);
		}
				//inline __host__ __device__
		//	float3 Unproject(const float2 p_c, float z) const
		//{
		//	return make_float3(z*(p_c.x-u0)/fu,z*(p_c.y-v0)/fv, z);
		//}

		inline __host__ __device__
			float3 Unproject(float u, float v, float z) const
		{
			return make_float3(z*(u-u0)/fu,z*(v-v0)/fv, z);
		}

		//////////////////////////////////////////////////////
		// Intrinsics for pow 2 pyramid
		//////////////////////////////////////////////////////

		inline __host__ __device__
			ImageIntrinsics operator[](int l) const
		{
			const float scale = 1.0f / (1 << l);
			return ImageIntrinsics(scale*fu, scale*fv, scale*(u0+0.5f)-0.5f, scale*(v0+0.5f)-0.5f);
		}


		//////////////////////////////////////////////////////
		// Scaling and ROI
		//////////////////////////////////////////////////////
		inline __host__ __device__
			ImageIntrinsics Scale(double scale)
		{
			ImageIntrinsics scaledImage;
			scaledImage.fu = fu * scale;
			scaledImage.fv = fv * scale;
			scaledImage.u0 = u0;
			scaledImage.v0 = v0;
			return scaledImage;
		}

#ifdef HAVE_NPP
		inline __host__ __device__
			ImageIntrinsics CropToROI(const NppiRect& roi)
		{
			ImageIntrinsics roidImage;
			roidImage.u0 = u0 - roi.x;
			roidImage.v0 = v0 - roi.y;
			roidImage.fu = fu;
			roidImage.fv = fv;
			return roidImage;
		}
#endif // HAVE_NPP

		//////////////////////////////////////////////////////
		// Interop
		//////////////////////////////////////////////////////

#ifdef USE_EIGEN
		inline __host__
			Eigen::Matrix3d Matrix() const {
				Eigen::Matrix3d K;
				K << fu, 0, u0,   0, fv, v0,  0,0,1;
				return K;
		}

		inline __host__
			Eigen::Matrix3d InverseMatrix() const {
				Eigen::Matrix3d K;
				K << 1.0/fu, 0, -u0/fu,  0, 1.0/fv, -v0/fv,  0,0,1;
				return K;
		}
#endif // USE_EIGEN

		//////////////////////////////////////////////////////
		// Member variables
		//////////////////////////////////////////////////////

		float fu;
		float fv;
		float u0;
		float v0;
	};

	struct ImageTransformProject
	{
		inline __host__ __device__
			float2 Project(const float3 P_w) const
		{
			return K * (T_iw * P_w);
		}

		ImageIntrinsics K;
		Mat<float,3,4> T_iw;
	};

}
