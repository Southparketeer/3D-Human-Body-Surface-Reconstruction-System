#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CUDA/cu_undistort.h"
#include "CUDA/ImageIntrinsics.h"

class cvUndistort
{
public:
	inline static void mat_to_array(const cv::Mat & mat, float * arr, const int& w, const int& h)
	{
		if(mat.cols != w || mat.rows != h )
			return;
		float *a = (float *) mat.data;
		memcpy(arr, a, sizeof(float) * w * h);
		a = NULL;
	}

	inline static void mat_to_array(const cv::Mat & mat, BYTE * arr, const int& w, const int& h)
	{
		if(mat.cols != w || mat.rows != h )
			return;
		BYTE *a = (BYTE *) mat.data;
		memcpy(arr, a, sizeof(BYTE) * w * h * 3);
		a = NULL;
	}
	inline static void cv_undistort(float* src, float* des, const loo::ImageIntrinsics& K, const loo::distortParam & D, const int& w, const int& h)
	{
		cv::Mat source(h,w, CV_32FC1, src);
		cv::Mat destin(h,w, CV_32FC1);
		//cv::Mat destin_flip(h,w, CV_32FC1);
		float A[9] = 
		{
			K.fu,	0,		K.u0,
			0,		K.fv,	K.v0,
			0,		0,		1
		};
		float B[5] = {D.k1, D.k2, D.p1, D.p2, D.k3};
		cv::Mat Amat(3, 3, CV_32FC1, A);
		cv::Mat Bmat(5, 1, CV_32FC1, B);
		cv::undistort(source, destin, Amat, Bmat);
		//cv::flip(destin, destin_flip, 0);
		mat_to_array(destin, des, w, h);
	}
	inline static void cv_undistort(BYTE* src, BYTE* des, const loo::ImageIntrinsics& K, const loo::distortParam & D, const int& w, const int& h)
	{
		cv::Mat source(h,w, CV_8UC3, src);
		cv::Mat destin(h,w, CV_8UC3);
		//cv::Mat destin_flip(h,w, CV_32FC1);
		float A[9] = 
		{
			K.fu,	0,		K.u0,
			0,		K.fv,	K.v0,
			0,		0,		1
		};
		float B[5] = {D.k1, D.k2, D.p1, D.p2, D.k3};
		cv::Mat Amat(3, 3, CV_32FC1, A);
		cv::Mat Bmat(5, 1, CV_32FC1, B);
		cv::undistort(source, destin, Amat, Bmat);
		mat_to_array(destin, des, w, h);
	}
};