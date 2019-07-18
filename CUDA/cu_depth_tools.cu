#include "cu_depth_tools.h"

#include "launch_utils.h"
#include "patch_score.h"
#include "MatUtils.h"
#include "InvalidValue.h"

namespace loo
{

	//////////////////////////////////////////////////////
	// Disparity to Depth Conversion
	//////////////////////////////////////////////////////

	__global__
		void KernDisp2Depth(const Image<float> dIn, Image<float> dOut, float fu, float fBaseline, float fMinDisp)
	{
		const int x = blockIdx.x*blockDim.x + threadIdx.x;
		const int y = blockIdx.y*blockDim.y + threadIdx.y;
		if( dOut.InBounds(x,y) ) {
			dOut(x,y) = dIn(x,y) >= fMinDisp ? fu * fBaseline / dIn(x,y) : InvalidValue<float>::Value();
		}
	}

	void Disp2Depth(Image<float> dIn, const Image<float> dOut, float fu, float fBaseline, float fMinDisp)
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim, gridDim, dOut);
		KernDisp2Depth<<<gridDim,blockDim>>>( dIn, dOut, fu, fBaseline, fMinDisp );
	}

	template<typename Tout, typename Tin>
	__global__ void KernFilterBadKinectData(Image<Tout> dFiltered, Image<Tin> dKinectDepth)
	{
		const int u = blockIdx.x*blockDim.x + threadIdx.x;
		const int v = blockIdx.y*blockDim.y + threadIdx.y;
		const float z_mm = dKinectDepth(u,v);
		dFiltered(u,v) = z_mm >= 200 ? z_mm : InvalidValue<float>::Value();
	}

	void FilterBadKinectData(Image<float> dFiltered, Image<unsigned short> dKinectDepth)
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImage(blockDim,gridDim, dFiltered);
		KernFilterBadKinectData<<<gridDim,blockDim>>>(dFiltered, dKinectDepth);
	}

	void FilterBadKinectData(Image<float> dFiltered, Image<float> dKinectDepth)
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImage(blockDim,gridDim, dFiltered);
		KernFilterBadKinectData<<<gridDim,blockDim>>>(dFiltered, dKinectDepth);
	}

	//////////////////////////////////////////////////////
	// Kinect depthmap to vertex array
	//////////////////////////////////////////////////////

	template<typename Ti>
	__global__ void KernDepthToVbo(
		Image<float4> dVbo, const Image<Ti> dDepth, ImageIntrinsics K, float depthscale
		) {
			const int u = blockIdx.x*blockDim.x + threadIdx.x;
			const int v = blockIdx.y*blockDim.y + threadIdx.y;
			const float kz = depthscale * dDepth(u,v);

			// (x,y,1) = kinv * (u,v,1)'
			const float3 P = K.Unproject(u,v,kz);
			dVbo(u,v) = make_float4(P.x,P.y,P.z,1);

	}

	template<typename T>
	void DepthToVbo(Image<float4> dVbo, const Image<T> dDepth, ImageIntrinsics K, float depthscale)
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImage(blockDim,gridDim, dVbo);
		KernDepthToVbo<T><<<gridDim,blockDim>>>(dVbo, dDepth, K, depthscale);
	}

	template<typename Ti>
	__global__ void KernDepthToVbofloat4( Image<float4> dVbo, const Image<Ti> dDepth, float depthscale ) 
	{
			const int u = blockIdx.x*blockDim.x + threadIdx.x;
			const int v = blockIdx.y*blockDim.y + threadIdx.y;
			const float kz = depthscale * dDepth(u,v);

			// (x,y,1) = kinv * (u,v,1)'
			const float3 p;// = K.Unproject(u,v,kz);
			p.z = kz;
			//p.x = (KI.x * p.x + KI.z) * p.z;
			//p.y = (KI.y * p.y + KI.w) * p.z;
			dVbo(u,v) = make_float4(p.x,p.y,p.z,1);
	}

	template<typename T>
	void DepthToVbofloat4(Image<float4> dVbo, const Image<T> dDepth, float depthscale)
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImage(blockDim,gridDim, dVbo);
		KernDepthToVbofloat4<T><<<gridDim,blockDim>>>(dVbo, dDepth, K, depthscale);
	}


	//////////////////////////////////////////////////////
	// Create cbo for vbo based on projection into image
	//////////////////////////////////////////////////////

	__global__ void KernColourVbo(
		Image<uchar4> dId, const Image<float4> dPd, const Image<uchar3> dIc,
		Mat<float,3,4> KT_cd
		) {
			const int u = blockIdx.x*blockDim.x + threadIdx.x;
			const int v = blockIdx.y*blockDim.y + threadIdx.y;

			if(u < dId.w && v < dId.h )
			{
				const float4 Pd4 = dPd(u,v);

				const Mat<float,4,1> Pd = {Pd4.x, Pd4.y, Pd4.z, 1};
				const Mat<float,3,1> KPc = KT_cd * Pd;

				const Mat<float,2,1> pc = { KPc(0) / KPc(2), KPc(1) / KPc(2) };

				uchar4 Id;
				if( dIc.InBounds(pc(0), pc(1), 1) ) {
					const float3 v = dIc.GetBilinear<float3>(pc(0), pc(1));
					Id = make_uchar4(v.x, v.y, v.z, 255);
				}else{
					Id = make_uchar4(0,0,0,0);
				}
				dId(u,v) = Id;
			}
	}

	void ColourVbo(Image<uchar4> dId, const Image<float4> dPd, const Image<uchar3> dIc, const Mat<float,3,4> KT_cd )
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim,gridDim, dId);
		KernColourVbo<<<gridDim,blockDim>>>(dId, dPd, dIc, KT_cd);
	}


	//////////////////////////////////////////////////////
	// Create textured view given depth image and keyframes
	//////////////////////////////////////////////////////

	template<typename Tout, typename Tin>
	__global__ void KernTextureDepth(Image<Tout> img, const ImageKeyframe<Tin> kf, const Image<float> depth, const Image<float4> norm, const Mat<float,3,4> T_wd, ImageIntrinsics Kdepth)
	{
		const int u = blockIdx.x*blockDim.x + threadIdx.x;
		const int v = blockIdx.y*blockDim.y + threadIdx.y;

		if(u < img.w && v < img.h )
		{
			const float d = depth(u,v);

			const float4 N_d = norm(u,v);
			const float3 N_w = mulSO3(T_wd, N_d);
			const float3 P_d = Kdepth.Unproject(u,v,d);
			const float3 P_w = T_wd * P_d;

			// project into kf
			const float2 p_kf = kf.Project(P_w);
			const float3 N_c = mulSO3(kf.T_iw,N_w);

			if(kf.img.InBounds(p_kf,2) && dot(N_c,make_float3(0,0,1)) < -0.2 ) {
				const float3 color = (1.0f/255.0f) * kf.img.template GetBilinear<float3>(p_kf);
				img(u,v) = make_float4(color,1);
			}else{
				img(u,v) = make_float4(0,0,0,1);
			}
		}
	}

	template<typename Tout, typename Tin>
	void TextureDepth(Image<Tout> img, const ImageKeyframe<Tin> kf, const Image<float> depth, const Image<float4> norm, const Mat<float,3,4> T_wd, ImageIntrinsics Kdepth)
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim,gridDim, img);
		KernTextureDepth<Tout,Tin><<<gridDim,blockDim>>>(img,kf,depth,norm, T_wd, Kdepth);
	}

	template void TextureDepth<float4,uchar3>(Image<float4> img, const ImageKeyframe<uchar3> kf, const Image<float> depth, const Image<float4> norm, const Mat<float,3,4> T_wd, ImageIntrinsics Kdepth);

	//////////////////////////////////////////////////////
	// Create textured view given depth image and keyframes
	//////////////////////////////////////////////////////

	template<typename Tout, typename Tin, size_t N>
	__global__ void KernTextureDepth(Image<Tout> img, const Mat<ImageKeyframe<Tin>,N> kfs, const Image<float> depth, const Image<float4> norm, const Image<float> phong, const Mat<float,3,4> T_wd, ImageIntrinsics Kdepth)
	{
		const int u = blockIdx.x*blockDim.x + threadIdx.x;
		const int v = blockIdx.y*blockDim.y + threadIdx.y;

		if(u < img.w && v < img.h )
		{
			const float d = depth(u,v);

			const float4 N_d = norm(u,v);
			const float3 N_w = mulSO3(T_wd, N_d);
			const float3 P_d = Kdepth.Unproject(u,v,d);
			const float3 P_w = T_wd * P_d;

			float w = 0;
			float3 color;

			// project into keyframes
			for(int k=0; k<N && kfs[k].img.ptr; ++k) {
				const ImageKeyframe<Tin>& kf = kfs[k];
				const float3 P_kf = kf.T_iw * P_w;
				const float2 p_kf = kf.K.Project(P_kf);
				const float3 N_c = mulSO3(kf.T_iw,N_w);
				const float ndot = dot(N_c,P_kf) / -length(P_kf);

				if(kf.img.InBounds(p_kf,2) && ndot > 0.1 && P_kf.z > 0 ) {
					color += (ndot/255.0f) * kf.img.template GetBilinear<float3>(p_kf);
					w += ndot;
				}
			}

			if(w == 0) {
				w = 1;
				color = make_float3(phong(u,v));
			}

			img(u,v) = make_float4(color / w, 1);
		}
	}

	template<typename Tout, typename Tin, size_t N>
	void TextureDepth(Image<Tout> img, const Mat<ImageKeyframe<Tin>,N> kfs, const Image<float> depth, const Image<float4> norm, const Image<float> phong, const Mat<float,3,4> T_wd, ImageIntrinsics Kdepth)
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim,gridDim, img);
		KernTextureDepth<Tout,Tin,N><<<gridDim,blockDim>>>(img,kfs,depth,norm,phong,T_wd,Kdepth);
	}

	template __declspec(dllexport) void TextureDepth<float4,uchar3,10>(Image<float4> img, const Mat<ImageKeyframe<uchar3>,10> kfs, const Image<float> depth, const Image<float4> norm, const Image<float> phong, const Mat<float,3,4> T_wd, ImageIntrinsics Kdepth);

	template __declspec(dllexport) void DepthToVbo<float>( Image<float4> dVbo, const Image<float> dKinectDepth, ImageIntrinsics K, float scale);
	template __declspec(dllexport) void DepthToVbo<unsigned short>( Image<float4> dVbo, const Image<unsigned short> dKinectDepth, ImageIntrinsics K, float scale);

}
