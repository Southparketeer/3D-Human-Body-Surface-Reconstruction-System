#pragma once

#include "Mat.h"
#include "Image.h"
#include "BoundedVolume.h"
#include "ImageIntrinsics.h"
#include "Sdf.h"

namespace loo
{

	__declspec(dllexport)
		void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img, const BoundedVolume<SDF_t> vol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far, float trunc_dist, bool subpix = true);

	__declspec(dllexport)
		void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img, const BoundedVolume<SDF_t> vol, const BoundedVolume<float> colorVol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far, float trunc_dist, bool subpix = true);

	__declspec(dllexport)
		void RaycastBox(Image<float> depth, const Mat<float,3,4> T_wc, ImageIntrinsics K, const BoundingBox bbox );

	__declspec(dllexport)
		void RaycastSphere(Image<float> depth, Image<float> img, const Mat<float,3,4> T_wc, ImageIntrinsics K, float3 center, float r);

	__declspec(dllexport)
		void RaycastPlane(Image<float> depth, Image<float> img, const Mat<float,3,4> T_wc, ImageIntrinsics K, const float3 n_w );

}
