#include "cu_sdffusion.h"
#include "MatUtils.h"
#include "launch_utils.h"
#include "InvalidValue.h"
#include <helper_functions.h>
#include <helper_cuda.h>
namespace loo
{

	//////////////////////////////////////////////////////
	// Truncated SDF Fusion
	// KinectFusion: Real-Time Dense Surface Mapping and Tracking, Newcombe et. al.
	// http://www.doc.ic.ac.uk/~rnewcomb/
	//////////////////////////////////////////////////////

	__global__ void KernSdfFuse(BoundedVolume<SDF_t> vol, Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics K, float trunc_dist, float max_w, float mincostheta, unsigned char if_clip )
	{
		const int x = blockIdx.x*blockDim.x + threadIdx.x;
		const int y = blockIdx.y*blockDim.y + threadIdx.y;
		const int z = blockIdx.z*blockDim.z + threadIdx.z;

		const float3 P_w = vol.VoxelPositionInUnits(x,y,z);
		float3 P_c = T_cw * P_w;
		const float2 proj_c = K.Project(P_c);
		float factor = 1.0f;
		const float margin = 25;
		const float clip_thr = 200;
		const float width = depth.w;

		if(if_clip == 1) //up
		{
			if(proj_c.x < margin) // 25
			{
				factor = 0;
			}
			else if(proj_c.x < (256 - clip_thr)) // 256 - 200
			{
				factor = ((proj_c.x - margin) / (256 - margin - clip_thr)); //256 - 25 - 200
			}
			else 
			{
				factor = 1;
			}
		}
		else if(if_clip == 2) // down
		{
			if(proj_c.x > width - margin) // 256 + 256 - 25
			{
				factor = 0;
			}
			else if(proj_c.x > clip_thr + 256) //256 + 200
			{
				factor = ((width - proj_c.x - margin) / (256.0 - margin - clip_thr)); //256 - 200 - 25
			}
			else
			{
				factor = 1;
			}
		}
		factor = factor * factor;

		const float2 proj_c_l = make_float2( proj_c.x + 1.0, proj_c.y);
		const float2 proj_c_r = make_float2( proj_c.x - 1.0, proj_c.y);

		if( depth.InBounds(proj_c, 2) )
		{
			const float vd = P_c.z;
			const float md = depth.GetBilinear<float>(proj_c);
			//const float mdl = depth.GetBilinear<float>(proj_c_l);
			//const float mdr = depth.GetNearestNeighbour(proj_c_r);
			//if(abs(mdl - md) > 400) return;
			if(md <= 500 || md >= 3000) return;

			const float3 mdn = make_float3(normals.GetBilinear<float4>(proj_c));
			const float costheta = dot(mdn, P_c) / -length(P_c);
			const float sd = costheta * (md - vd);
			const float w = costheta * 1000.0f/vd;
			if(sd <= -trunc_dist) {
				// Further than truncation distance from surface
				// We do nothing.
				return;
			}else{
				if(isfinite(md) && isfinite(w) && costheta > mincostheta ) {
					SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w * factor);
					sdf += vol(x,y,z);
					sdf.LimitWeight(max_w);
					vol(x,y,z) = sdf;
				}
			}
		}
	}

	void SdfFuse(BoundedVolume<SDF_t> vol, Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics K, float trunc_dist, float max_w, float mincostheta , unsigned char if_clip)
	{
		//if_clip == 0 ---> no clip
		//if_clip == 1 ---> clip up
		//if_clip == 2 ---> clip down
		dim3 blockDim(4,4,4);
		dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y, vol.d / blockDim.z);
		KernSdfFuse<<<gridDim,blockDim>>>(vol, depth, norm, T_cw, K, trunc_dist, max_w, mincostheta, if_clip);
		//GpuCheckErrors();
	}

	//////////////////////////////////////////////////////
	// Color Truncated SDF Fusion
	// Similar extension to KinectFusion as described by:
	// Robust Tracking for Real-Time Dense RGB-D Mapping with Kintinous
	// Whelan et. al.
	//////////////////////////////////////////////////////

	__global__ void KernSdfFuse(
		BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,
		Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics K,
		Image<uchar3> img, Mat<float,3,4> T_iw, ImageIntrinsics Kimg,
		float trunc_dist, float max_w, float mincostheta
		)
	{
		const int x = blockIdx.x*blockDim.x + threadIdx.x;
		const int y = blockIdx.y*blockDim.y + threadIdx.y;

		//    const int z = blockIdx.z*blockDim.z + threadIdx.z;
		for(int z=0; z < vol.d; ++z) {
			const float3 P_w = vol.VoxelPositionInUnits(x,y,z);
			const float3 P_c = T_cw * P_w;
			const float2 p_c = K.Project(P_c);
			const float3 P_i = T_iw * P_w;
			const float2 p_i = Kimg.Project(P_i);

			if( depth.InBounds(p_c, 2) && img.InBounds(p_i,2) )
			{
				const float vd = P_c.z;
				const float md = depth.GetBilinear<float>(p_c);
				const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));
				const float c = ConvertPixel<float,float3>( img.GetBilinear<float3>(p_i) ) / 255.0;

				const float costheta = dot(mdn, P_c) / -length(P_c);
				const float sd = costheta * (md - vd);
				const float w = costheta * 1.0f/vd;

				if(sd <= -trunc_dist) {
					// Further than truncation distance from surface
					// We do nothing.
				}else{
					if(isfinite(md) && isfinite(w) && costheta > mincostheta ) {
						const SDF_t curvol = vol(x,y,z);
						SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
						sdf += curvol;
						sdf.LimitWeight(max_w);
						vol(x,y,z) = sdf;
						colorVol(x,y,z) = (w*c + colorVol(x,y,z) * curvol.w) / (w + curvol.w);
					}
				}
			}
		}
	}

	void SdfFuse(
		BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,
		Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics K,
		Image<uchar3> img, Mat<float,3,4> T_iw, ImageIntrinsics Kimg,
		float trunc_dist, float max_w, float mincostheta
		) {
			//    // 3d invoke
			//    dim3 blockDim(8,8,8);
			//    dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y, vol.d / blockDim.z);
			//    KernSdfFuse<<<gridDim,blockDim>>>(vol, colorVol, depth, norm, T_cw, K, img, T_iw, Kimg, trunc_dist, max_w, mincostheta);
			//    GpuCheckErrors();

			dim3 blockDim(16,16);
			dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y);
			KernSdfFuse<<<gridDim,blockDim>>>(vol, colorVol, depth, norm, T_cw, K, img, T_iw, Kimg, trunc_dist, max_w, mincostheta);
			//GpuCheckErrors();

	}

	//////////////////////////////////////////////////////
	// Reset SDF
	//////////////////////////////////////////////////////

	__global__ void KernSdfReset(BoundedVolume<SDF_t> vol, float trunc_dist)
	{
		const int x = blockIdx.x*blockDim.x + threadIdx.x;
		const int y = blockIdx.y*blockDim.y + threadIdx.y;
		const int z = blockIdx.z*blockDim.z + threadIdx.z;

		vol(x,y,z) = SDF_t( trunc_dist, 0);
	}

	void SdfReset(BoundedVolume<SDF_t> vol, float trunc_dist)
	{
#ifndef _MSC_VER
		vol.Fill(SDF_t( trunc_dist, 0));
#else
		// On Windows, can't call thrust::fill with aligned struct...
		dim3 blockDim(8,8,8);
		dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y, vol.d / blockDim.z);
		KernSdfReset<<<gridDim,blockDim>>>(vol,trunc_dist);
		//GpuCheckErrors();
#endif
	}

	void SdfReset(BoundedVolume<float> vol)
	{
		vol.Fill(0.5);
	}

	//////////////////////////////////////////////////////
	// Create SDF representation of sphere
	//////////////////////////////////////////////////////

	__global__ void KernSdfSphere(BoundedVolume<SDF_t> vol, float3 center, float r)
	{
		const int x = blockIdx.x*blockDim.x + threadIdx.x;
		const int y = blockIdx.y*blockDim.y + threadIdx.y;
		const int z = blockIdx.z*blockDim.z + threadIdx.z;

		const float3 pos = vol.VoxelPositionInUnits(x,y,z);
		const float dist = length(pos - center);
		const float sdf = dist - r;

		vol(x,y,z) = SDF_t(sdf); 
	}

	void SdfSphere(BoundedVolume<SDF_t> vol, float3 center, float r)
	{
		dim3 blockDim(8,8,8);
		dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y, vol.d / blockDim.z);

		KernSdfSphere<<<gridDim,blockDim>>>(vol, center, r);
		//GpuCheckErrors();
	}

	//////////////////////////////////////////////////////
	// Take SDF Difference to depthmap
	//////////////////////////////////////////////////////

	__global__ void KernSdfDistance(Image<float> dist, Image<float> depth, BoundedVolume<SDF_t> vol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float trunc_distance)
	{
		const int u = blockIdx.x*blockDim.x + threadIdx.x;
		const int v = blockIdx.y*blockDim.y + threadIdx.y;

		if( u < depth.w && v < depth.h ) {
			const float z = depth(u,v);
			const float3 p_c = z * K.Unproject(u,v);
			const float3 p_w = T_wc * p_c;

			const SDF_t sdf = vol.GetUnitsTrilinearClamped(p_w);
			dist(u,v) = sdf.val; //(sdf.val + trunc_distance) / (2* trunc_distance);
		}    
	}


	void SdfDistance(Image<float> dist, Image<float> depth, BoundedVolume<SDF_t> vol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float trunc_distance)
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim, gridDim, depth);

		KernSdfDistance<<<gridDim,blockDim>>>(dist, depth, vol, T_wc, K, trunc_distance);
		//GpuCheckErrors();
	}

}
