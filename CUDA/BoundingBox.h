#pragma once
#include <cuda_runtime.h>
#include "MatUtils.h"
#include "ImageIntrinsics.h"
#include <iostream>

namespace loo
{

	// Axis Aligned Bounding Box
	struct BoundingBox
	{
		inline __device__ __host__
			BoundingBox()
		{
		}

		inline __device__ __host__
			BoundingBox(const BoundingBox& bbox)
			: boxmin(bbox.boxmin), boxmax(bbox.boxmax)
		{
		}

		inline __device__ __host__
			BoundingBox(const float3 boxmin, const float3 boxmax)
			: boxmin(boxmin), boxmax(boxmax)
		{
		}

		// Construct bounding box from Frustum.
		inline __host__
			BoundingBox(
			const Mat<float,3,4> T_wc,
			float w, float h,
			float fu, float fv, float u0, float v0,
			float near, float far
			) {
				FitToFrustum(T_wc,w,h,fu,fv,u0,v0,near,far);
		}

		// Construct bounding box from Frustum.
		inline __host__
			BoundingBox(
			const Mat<float,3,4> T_wc,
			float w, float h,
			ImageIntrinsics K,
			float near, float far
			) {
				FitToFrustum(T_wc,w,h,K.fu,K.fv,K.u0,K.v0,near,far);
		}

		inline __host__ __device__
			float3& Min() {
				return boxmin;
		}

		inline __host__ __device__
			float3 Min() const {
				return boxmin;
		}

		inline __host__ __device__
			float3& Max() {
				return boxmax;
		}

		inline __host__ __device__
			float3 Max() const {
				return boxmax;
		}

		inline __host__
			void FitToFrustum(
			const Mat<float,3,4> T_wc,
			float w, float h,
			float fu, float fv, float u0, float v0,
			float near, float far
			) {
				Clear();

				// Insert each edge of frustum into bounding box
				const float3 c_w = SE3Translation(T_wc);
				const float3 ray_tl = mulSO3(T_wc, make_float3((0-u0)/fu,(0-v0)/fv, 1));
				const float3 ray_tr = mulSO3(T_wc, make_float3((w-u0)/fu,(0-v0)/fv, 1));
				const float3 ray_bl = mulSO3(T_wc, make_float3((0-u0)/fu,(h-v0)/fv, 1));
				const float3 ray_br = mulSO3(T_wc, make_float3((w-u0)/fu,(h-v0)/fv, 1));

				Insert(c_w + near*ray_tl);
				Insert(c_w + near*ray_tr);
				Insert(c_w + near*ray_bl);
				Insert(c_w + near*ray_br);
				Insert(c_w + far*ray_tl);
				Insert(c_w + far*ray_tr);
				Insert(c_w + far*ray_bl);
				Insert(c_w + far*ray_br);
		}

		inline __host__
			void FitToFrustum(
			const Mat<float,3,4> T_wc,
			float w, float h,
			ImageIntrinsics K,
			float near, float far
			) {
				FitToFrustum(T_wc,w,h,K.fu,K.fv,K.u0,K.v0,near,far);
		}

		inline __host__
			void Clear()
		{
			boxmin = make_float3(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
			boxmax = make_float3(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());
		}

		// Expand bounding box to include p
		inline __host__
			void Insert(const float3 p)
		{
			boxmax = fmaxf(p,boxmax);
			boxmin = fminf(p,boxmin);
		}

		// Expand bounding box to include bb
		inline __host__
			void Insert(const BoundingBox& bb)
		{
			boxmin = make_float3(fminf(bb.boxmin.x,boxmin.x), fminf(bb.boxmin.y,boxmin.y), fminf(bb.boxmin.z,boxmin.z));
			boxmax = make_float3(fmaxf(bb.boxmax.x,boxmax.x), fmaxf(bb.boxmax.y,boxmax.y), fmaxf(bb.boxmax.z,boxmax.z));
		}

		// Contract bounding box to represent intersection (common space)
		// between this and bb
		inline __host__
			void Intersect(const BoundingBox& bb)
		{
			boxmin = make_float3(fmaxf(bb.boxmin.x,boxmin.x), fmaxf(bb.boxmin.y,boxmin.y), fmaxf(bb.boxmin.z,boxmin.z));
			boxmax = make_float3(fminf(bb.boxmax.x,boxmax.x), fminf(bb.boxmax.y,boxmax.y), fminf(bb.boxmax.z,boxmax.z));
		}

		inline __host__ __device__
			float3 Size() const
		{
			return boxmax - boxmin;
		}

		inline __host__ __device__
			float3 Center() const
		{
			return boxmin + (boxmax - boxmin)/2.0f;
		}

		inline __host__ __device__
			void Enlarge(float3 scale)
		{
			const float3 center = Center();
			const float3 newHalfSize = (scale * Size()) / 2.0f;
			boxmin = center - newHalfSize;
			boxmax = center + newHalfSize;

		}

		float3 boxmin;
		float3 boxmax;
	};

	inline std::ostream& operator<<( std::ostream& os, const loo::BoundingBox& bbox)
	{
		os << bbox.Min() << " - " << bbox.Max();
		return os;
	}

	inline std::istream& operator>>( std::istream& is, loo::BoundingBox& bbox)
	{
		is >> bbox.Min();
		is.ignore(3, '-');
		is >> bbox.Max();
		return is;
	}

}
