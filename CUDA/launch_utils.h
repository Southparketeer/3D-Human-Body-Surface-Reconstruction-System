#pragma once

#include <cstdio>

#include "Image.h"

#define GPU_CHECK_ERRORS
//#define GPU_CHECK_ERRORS_SYNC

// Based on cutil methods for error checking.
#define GpuCheckSuccess( err ) roo::__SuccessOrDie( err, __FILE__, __LINE__ )
#define GpuCheckErrors() roo::__CheckForErrorsDie( __FILE__, __LINE__ )

namespace loo
{

	// Based on cutil methods for error checking.
	inline void __SuccessOrDie( cudaError err, const char *file, const int line )
	{
#ifdef GPU_CHECK_ERRORS
		if ( cudaSuccess != err ) {
			fprintf( stderr, "cudaSafeCall() failed at %s:%i : %s\n", file, line, cudaGetErrorString( err ) );
			exit(-1);
		}
#endif // GPU_CHECK_ERRORS
	}

	// Based on cutil methods for error checking.
	inline void __CheckForErrorsDie(const char* file, const int line)
	{
#ifdef GPU_CHECK_ERRORS
		cudaError err = cudaGetLastError();
		if ( cudaSuccess != err ) {
			fprintf( stderr, "cudaCheckError() failed at %s:%i : %s\n", file, line, cudaGetErrorString( err ) );
			exit(-1);
		}

#ifdef GPU_CHECK_ERRORS_SYNC
		err = cudaDeviceSynchronize();
		if( cudaSuccess != err ) {
			fprintf( stderr, "cudaCheckError() with sync failed at %s:%i : %s\n", file, line, cudaGetErrorString( err ) );
			exit(-1);
		}
#endif // GPU_CHECK_ERRORS_SYNS

#endif // GPU_CHECK_ERRORS
	}

	// Euclids method
	// http://en.wikipedia.org/wiki/Greatest_common_divisor
	template<typename T>
	inline int Gcd(T a, T b)
	{
		const T amodb = a%b;
		return amodb ? Gcd(b, amodb) : b;
	}

	//! Utility for attempting to estimate safe block/grid dimensions from working image dimensions
	//! These are not necesserily optimal. Far from it.
	template<typename T, typename Target, typename Management>
	inline void InitDimFromOutputImage(dim3& blockDim, dim3& gridDim, const Image<T,Target,Management>& image, unsigned blockx = 32, unsigned blocky = 32)
	{
		blockDim = dim3( Gcd<unsigned>(image.w,blockx), Gcd<unsigned>(image.h,blocky), 1);
		gridDim =  dim3( image.w / blockDim.x, image.h / blockDim.y, 1);
	}

	//! Utility for attempting to estimate safe block/grid dimensions from working image dimensions
	//! These are not necesserily optimal. Far from it.
	template<typename T, typename Target, typename Management>
	inline void InitDimFromOutputImageOver(dim3& blockDim, dim3& gridDim, const Image<T,Target,Management>& image, int blockx = 32, int blocky = 32)
	{
		blockDim = dim3(blockx, blocky);
		gridDim =  dim3( ceil(image.w / (double)blockDim.x), ceil(image.h / (double)blockDim.y) );
	}

}
