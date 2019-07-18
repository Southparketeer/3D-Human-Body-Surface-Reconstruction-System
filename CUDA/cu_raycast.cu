#include "cu_raycast.h"

#include "MatUtils.h"
#include "launch_utils.h"
#include "InvalidValue.h"

namespace loo
{

	//////////////////////////////////////////////////////
	// Phong shading.
	//////////////////////////////////////////////////////

	__host__ __device__ inline
		float PhongShade(const float3 p_c, const float3 n_c)
	{
		const float ambient = 0.2;
		const float diffuse = 0.4;
		const float specular = 0.4;
		const float3 eyedir = -1.0f * p_c / length(p_c);
		const float3 _lightdir = make_float3(0.4,0.4,-1);
		const float3 lightdir = _lightdir / length(_lightdir);
		const float ldotn = dot(lightdir,n_c);
		const float3 lightreflect = 2*ldotn*n_c + (-1.0) * lightdir;
		const float edotr = fmaxf(0,dot(eyedir,lightreflect));
		const float spec = edotr*edotr*edotr*edotr*edotr*edotr*edotr*edotr*edotr*edotr;
		return ambient + diffuse * ldotn  + specular * spec;
	}

	//////////////////////////////////////////////////////
	// Raycast SDF
	//////////////////////////////////////////////////////

	__global__ void KernRaycastSdf(Image<float> imgdepth, Image<float4> norm, Image<float> img, const BoundedVolume<SDF_t> vol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far, float trunc_dist, bool subpix )
	{
		const int u = blockIdx.x*blockDim.x + threadIdx.x;
		const int v = blockIdx.y*blockDim.y + threadIdx.y;

		if( u < img.w && v < img.h ) {
			const float3 c_w = SE3Translation(T_wc); //position of camera in world space
			const float3 ray_c = K.Unproject(u,v); 
			const float3 ray_w = mulSO3(T_wc, ray_c); //将camera射线转化到世界坐标系下

			// Raycast bounding box to find valid ray segment of sdf
			// http://www.cs.utah.edu/~awilliam/box/box.pdf
			const float3 tminbound = (vol.bbox.Min() - c_w) / ray_w; //(vol.bbox.Min() - c_w) 空间voxel的一角 与camera的连线 最小bound-->步长
			const float3 tmaxbound = (vol.bbox.Max() - c_w) / ray_w; ////(vol.bbox.Min() - c_w) 空间voxel的一角 与camera的连线 最大bound-->步长
			const float3 tmin = fminf(tminbound,tmaxbound);
			const float3 tmax = fmaxf(tminbound,tmaxbound);
			const float max_tmin = fmaxf(fmaxf(fmaxf(tmin.x, tmin.y), tmin.z), near); //1000
			const float min_tmax = fminf(fminf(fminf(tmax.x, tmax.y), tmax.z), far); // 2200
			float depth = 0.0f;

			// If ray intersects bounding box
			if(max_tmin < min_tmax ) {
				// Go between max_tmin and min_tmax
				float lambda = max_tmin;
				float last_sdf = InvalidValue<float>::Value();
				float min_delta_lambda = vol.VoxelSizeUnits().x;
				float delta_lambda = 0;

				// March through space
				while(lambda < min_tmax) {
					const float3 pos_w = c_w + lambda * ray_w;
					const float sdf = vol.GetUnitsTrilinearClamped(pos_w);

					if( sdf <= 0 ) {
						if( last_sdf > 0) {
							// surface!
							if(subpix) {
								lambda = lambda + delta_lambda * sdf / (last_sdf - sdf);
							}
							depth = lambda;
						}
						break;
					}
					delta_lambda = sdf > 0 ? fmaxf(sdf, min_delta_lambda) : trunc_dist;
					lambda += delta_lambda;
					last_sdf = sdf;
				}
			}

			// Compute normal
			const float3 pos_w = c_w + depth * ray_w;
			const float3 _n_w = vol.GetUnitsBackwardDiffDxDyDz(pos_w);
			const float len_n_w = length(_n_w);
			const float3 n_w = len_n_w > 0 ? _n_w / len_n_w : make_float3(0,0,1);
			const float3 n_c = mulSO3inv(T_wc,n_w);
			const float3 p_c = depth * ray_c;

			if(depth > 0 ) {
				imgdepth(u,v) = depth;
				img(u,v) = PhongShade(p_c, n_c);
				norm(u,v) = make_float4(n_c, 1);
			}else{
				imgdepth(u,v) = InvalidValue<float>::Value();
				img(u,v) = 0;
				norm(u,v) = make_float4(0,0,0,0);
			}
		}
	}

	void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img, const BoundedVolume<SDF_t> vol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far, float trunc_dist, bool subpix )
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim, gridDim, img);
		KernRaycastSdf<<<gridDim,blockDim>>>(depth, norm, img, vol, T_wc, K, near, far, trunc_dist, subpix);
		//GpuCheckErrors();
	}

	//////////////////////////////////////////////////////
	// Raycast Color SDF
	//////////////////////////////////////////////////////

	__global__ void KernRaycastSdf(Image<float> imgdepth, Image<float4> norm, Image<float> img, const BoundedVolume<SDF_t> vol, const BoundedVolume<float> colorVol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far, float trunc_dist, bool subpix )
	{
		const int u = blockIdx.x*blockDim.x + threadIdx.x;
		const int v = blockIdx.y*blockDim.y + threadIdx.y;

		if( u < img.w && v < img.h ) {
			const float3 c_w = SE3Translation(T_wc);
			const float3 ray_c = K.Unproject(u,v);
			const float3 ray_w = mulSO3(T_wc, ray_c);

			// Raycast bounding box to find valid ray segment of sdf
			// http://www.cs.utah.edu/~awilliam/box/box.pdf
			const float3 tminbound = (vol.bbox.Min() - c_w) / ray_w;
			const float3 tmaxbound = (vol.bbox.Max() - c_w) / ray_w;
			const float3 tmin = fminf(tminbound,tmaxbound);
			const float3 tmax = fmaxf(tminbound,tmaxbound);
			const float max_tmin = fmaxf(fmaxf(fmaxf(tmin.x, tmin.y), tmin.z), near);
			const float min_tmax = fminf(fminf(fminf(tmax.x, tmax.y), tmax.z), far);

			float depth = 0.0f;

			// If ray intersects bounding box
			if(max_tmin < min_tmax ) {
				// Go between max_tmin and min_tmax
				float lambda = max_tmin;
				float last_sdf = InvalidValue<float>::Value();
				float min_delta_lambda = vol.VoxelSizeUnits().x;
				float delta_lambda = 0;

				// March through space
				while(lambda < min_tmax) {
					const float3 pos_w = c_w + lambda * ray_w;
					const float sdf = vol.GetUnitsTrilinearClamped(pos_w);

					if( sdf <= 0 ) {
						if( last_sdf > 0) {
							// surface!
							if(subpix) {
								lambda = lambda + delta_lambda * sdf / (last_sdf - sdf);
							}
							depth = lambda;
						}
						break;
					}
					delta_lambda = sdf > 0 ? fmaxf(sdf, min_delta_lambda) : trunc_dist;
					lambda += delta_lambda;
					last_sdf = sdf;
				}
			}

			// Compute normal
			const float3 pos_w = c_w + depth * ray_w;
			const float3 _n_w = vol.GetUnitsBackwardDiffDxDyDz(pos_w);
			const float c = colorVol.GetUnitsTrilinearClamped(pos_w);
			const float len_n_w = length(_n_w);
			const float3 n_w = len_n_w > 0 ? _n_w / len_n_w : make_float3(0,0,1);
			const float3 n_c = mulSO3inv(T_wc,n_w);

			if(depth > 0 ) {
				imgdepth(u,v) = depth;
				img(u,v) = c;
				norm(u,v) = make_float4(n_c, 1);
			}else{
				imgdepth(u,v) = InvalidValue<float>::Value();
				img(u,v) = 0;
				norm(u,v) = make_float4(0,0,0,0);
			}
		}
	}

	void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img, const BoundedVolume<SDF_t> vol, const BoundedVolume<float> colorVol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far, float trunc_dist, bool subpix )
	{
		dim3 blockDim, gridDim;
		//    InitDimFromOutputImageOver(blockDim, gridDim, img, 16, 16);
		InitDimFromOutputImageOver(blockDim, gridDim, img);
		KernRaycastSdf<<<gridDim,blockDim>>>(depth, norm, img, vol, colorVol, T_wc, K, near, far, trunc_dist, subpix);
		//GpuCheckErrors();
	}

	//////////////////////////////////////////////////////
	// Raycast box
	//////////////////////////////////////////////////////

	__global__ void KernRaycastBox(Image<float> imgd, const Mat<float,3,4> T_wc, ImageIntrinsics K, const BoundingBox bbox )
	{
		const int u = blockIdx.x*blockDim.x + threadIdx.x;
		const int v = blockIdx.y*blockDim.y + threadIdx.y;

		if( u < imgd.w && v < imgd.h ) {
			const float3 c_w = SE3Translation(T_wc);
			const float3 ray_c = K.Unproject(u,v);
			const float3 ray_w = mulSO3(T_wc, ray_c);

			// Raycast bounding box to find valid ray segment of sdf
			// http://www.cs.utah.edu/~awilliam/box/box.pdf
			const float3 tminbound = (bbox.Min() - c_w) / ray_w;
			const float3 tmaxbound = (bbox.Max() - c_w) / ray_w;
			const float3 tmin = fminf(tminbound,tmaxbound);
			const float3 tmax = fmaxf(tminbound,tmaxbound);
			const float max_tmin = fmaxf(fmaxf(tmin.x, tmin.y), tmin.z);
			const float min_tmax = fminf(fminf(tmax.x, tmax.y), tmax.z);

			float d;

			// If ray intersects bounding box
			if(max_tmin < min_tmax ) {
				d = max_tmin;
			}else{
				d = InvalidValue<float>::Value();
			}

			imgd(u,v) = d;
		}
	}

	void RaycastBox(Image<float> imgd, const Mat<float,3,4> T_wc, ImageIntrinsics K, const BoundingBox bbox )
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim, gridDim, imgd);
		KernRaycastBox<<<gridDim,blockDim>>>(imgd, T_wc, K, bbox);
		//GpuCheckErrors();
	}

	//////////////////////////////////////////////////////
	// Raycast sphere
	//////////////////////////////////////////////////////


	__global__ void KernRaycastSphere(Image<float> imgd, Image<float> img, ImageIntrinsics K, float3 center_c, float r)
	{
		const int u = blockIdx.x*blockDim.x + threadIdx.x;
		const int v = blockIdx.y*blockDim.y + threadIdx.y;
		
		if( u < imgd.w && v < imgd.h ) {
			const float3 ray_c = K.Unproject(u,v);
			const float ldotc = dot(ray_c,center_c);
			const float lsq = dot(ray_c,ray_c);
			const float csq = dot(center_c,center_c);
			float depth = (ldotc - sqrt(ldotc*ldotc - lsq*(csq - r*r) )) / lsq;
			const float prev_depth = (imgd(u,v) == 0) ? INFINITY : imgd(u,v);
			if(depth > 0 && (depth < prev_depth || !isfinite(prev_depth)) ) {
				imgd(u,v) = depth;
				if(img.ptr) {
					const float3 p_c = depth * ray_c;           
					const float3 n_c = p_c - center_c;
					img(u,v) = PhongShade(p_c, n_c / length(n_c));
				}
			}
		}
	}

	void RaycastSphere(Image<float> imgd, Image<float> img, const Mat<float,3,4> T_wc, ImageIntrinsics K, float3 center, float r)
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim, gridDim, imgd);
		const float3 center_c = mulSE3inv(T_wc, center);    
		KernRaycastSphere<<<gridDim,blockDim>>>(imgd, img, K, center_c, r);
		//GpuCheckErrors();
	}

	//////////////////////////////////////////////////////
	// Raycast plane
	//////////////////////////////////////////////////////

	__global__ void KernRaycastPlane(Image<float> imgd, Image<float> img, ImageIntrinsics K, const float3 n_c)
	{
		const int u = blockIdx.x*blockDim.x + threadIdx.x;
		const int v = blockIdx.y*blockDim.y + threadIdx.y;

		if( u < img.w && v < img.h ) {
			const float3 ray_c = K.Unproject(u,v);
			const float depth = -1 / dot(n_c, ray_c);

			const float prev_depth = imgd(u,v);
			if(depth > 0 && (depth < prev_depth || !isfinite(prev_depth)) ) {
				const float3 p_c = depth * ray_c;
				img(u,v) = PhongShade(p_c, n_c / length(n_c) );
				imgd(u,v) = depth;
			}
		}
	}

	void RaycastPlane(Image<float> imgd, Image<float> img, const Mat<float,3,4> T_wc, ImageIntrinsics K, const float3 n_w )
	{
		const float3 n_c = Plane_b_from_a(T_wc, n_w);

		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim, gridDim, img);
		KernRaycastPlane<<<gridDim,blockDim>>>(imgd, img, K, n_c );
		//GpuCheckErrors();
	}


}
