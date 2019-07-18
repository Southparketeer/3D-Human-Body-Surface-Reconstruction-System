#pragma once

//#include <kangaroo/platform.h>
#include "Volume.h"
#include "BoundingBox.h"

namespace loo
{

	template<typename T, typename Target = TargetDevice, typename Management = DontManage>
	class BoundedVolume : public Volume<T,Target,Management>
	{
	public:

		//////////////////////////////////////////////////////
		// Constructors
		//////////////////////////////////////////////////////

		template<typename TargetFrom, typename ManagementFrom> inline __host__ __device__
			BoundedVolume( const BoundedVolume<T,TargetFrom,ManagementFrom>& vol )
			: Volume<T,Target,Management>(vol), bbox(vol.bbox)
		{
		}

		template<typename TargetFrom, typename ManagementFrom> inline __host__ __device__
			BoundedVolume(const Volume<T,TargetFrom,ManagementFrom>& vol, const BoundingBox& bbox)
			: Volume<T,Target,Management>(vol), bbox(bbox)
		{
		}

		inline __host__ __device__
			BoundedVolume()
		{
		}

		inline __host__
			BoundedVolume(unsigned int w, unsigned int h, unsigned int d)
			: Volume<T,Target,Management>(w,h,d), bbox(make_float3(-1,-1,-1), make_float3(1,1,1))
		{
		}

		inline __host__
			BoundedVolume(unsigned int w, unsigned int h, unsigned int d, const BoundingBox& bbox )
			: Volume<T,Target,Management>(w,h,d), bbox(bbox)
		{
		}

		inline __host__
			BoundedVolume(unsigned int w, unsigned int h, unsigned int d, float3 min_bounds, float3 max_bounds)
			: Volume<T,Target,Management>(w,h,d), bbox(min_bounds,max_bounds)
		{
		}

		//////////////////////////////////////////////////////
		// Dimensions
		//////////////////////////////////////////////////////

		inline __device__ __host__
			float3 SizeUnits() const
		{
			return bbox.Size();
		}

		inline __device__ __host__
			float3 VoxelSizeUnits() const
		{
			return bbox.Size() /
				make_float3(
				Volume<T,Target,Management>::w-1,
				Volume<T,Target,Management>::h-1,
				Volume<T,Target,Management>::d-1
				);
		}

		//////////////////////////////////////////////////////
		// Return true if this BoundedVolume represents a positive
		// amount of space.
		//////////////////////////////////////////////////////

		inline __device__ __host__
			bool IsValid() const {
				const uint3 size = Volume<T,Target,Management>::Voxels();
				return size.x >= 8 && size.y >= 8 && size.z >= 8;
		}

		//////////////////////////////////////////////////////
		// Access volume in units of Bounding Box
		//////////////////////////////////////////////////////

		inline  __device__ __host__
			float GetUnitsTrilinearClamped(float3 pos_w) const
		{
			const float3 pos_v = (pos_w - bbox.Min()) / (bbox.Size());
			return Volume<T,Target,Management>::GetFractionalTrilinearClamped(pos_v);
		}

		inline __device__ __host__
			float3 GetUnitsBackwardDiffDxDyDz(float3 pos_w) const
		{
			const float3 pos_v = (pos_w - bbox.Min()) / (bbox.Size());
			const float3 deriv = Volume<T,Target,Management>::GetFractionalBackwardDiffDxDyDz(pos_v);
			return deriv / VoxelSizeUnits();
		}

		inline __device__ __host__
			float3 GetUnitsOutwardNormal(float3 pos_w) const
		{
			const float3 deriv = GetUnitsBackwardDiffDxDyDz(pos_w);
			return deriv / length(deriv);
		}

		inline __device__ __host__
			float3 VoxelPositionInUnits(int x, int y, int z) const
		{
			const float3 vol_size = bbox.Size();

			return make_float3(
				bbox.Min().x + vol_size.x*x/(float)(Volume<T,Target,Management>::w-1),
				bbox.Min().y + vol_size.y*y/(float)(Volume<T,Target,Management>::h-1),
				bbox.Min().z + vol_size.z*z/(float)(Volume<T,Target,Management>::d-1)
				);
		}

		inline __device__ __host__
			float3 VoxelPositionInUnits(int3 p_v) const
		{
			return VoxelPositionInUnits(p_v.x,p_v.y,p_v.z);
		}

		//////////////////////////////////////////////////////
		// Access sub-regions
		//////////////////////////////////////////////////////

		inline __device__ __host__
			BoundedVolume<T,Target,DontManage> SubBoundingVolume(const BoundingBox& region)
		{
			const float3 min_fv = (region.Min() - bbox.Min()) / (bbox.Size());
			const float3 max_fv = (region.Max() - bbox.Min()) / (bbox.Size());

			const int3 min_v = make_int3(
				fmaxf((Volume<T,Target,Management>::w-1)*min_fv.x, 0),
				fmaxf((Volume<T,Target,Management>::h-1)*min_fv.y, 0),
				fmaxf((Volume<T,Target,Management>::d-1)*min_fv.z, 0)
				);
			const int3 max_v = make_int3(
				fminf(ceilf((Volume<T,Target,Management>::w-1)*max_fv.x), Volume<T,Target,Management>::w-1),
				fminf(ceilf((Volume<T,Target,Management>::h-1)*max_fv.y), Volume<T,Target,Management>::h-1),
				fminf(ceilf((Volume<T,Target,Management>::d-1)*max_fv.z), Volume<T,Target,Management>::d-1)
				);

			const int3 size_v = max((max_v - min_v) + make_int3(1,1,1), make_int3(0,0,0) );

			const BoundingBox nbbox(
				VoxelPositionInUnits(min_v),
				VoxelPositionInUnits(max_v)
				);

			return BoundedVolume<T,Target,DontManage>(
				Volume<T,Target,Management>::SubVolume(min_v, size_v),
				nbbox
				);
		}
		BoundingBox bbox;
	};

}
