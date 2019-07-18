#pragma once

#include "Mat.h"
#include "Image.h"
#include "BoundedVolume.h"
#include "ImageIntrinsics.h"
#include "Sdf.h"

namespace loo
{
	__declspec(dllexport)
		void SdfFuse(BoundedVolume<SDF_t> vol, Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics K, float trunc_dist, float maxw, float mincostheta, unsigned char if_clip );

	__declspec(dllexport)
		void SdfFuse(BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol, Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics K, Image<uchar3> img, Mat<float,3,4> T_iw, ImageIntrinsics Kimg,float trunc_dist, float max_w, float mincostheta);

	__declspec(dllexport)
		void SdfReset(BoundedVolume<SDF_t> vol, float trunc_dist);

	__declspec(dllexport)
		void SdfReset(BoundedVolume<float> vol);

	__declspec(dllexport)
		void SdfSphere(BoundedVolume<SDF_t> vol, float3 center, float r);

	__declspec(dllexport)
		void SdfDistance(Image<float> dist, Image<float> depth, BoundedVolume<SDF_t> vol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float trunc_distance);
}