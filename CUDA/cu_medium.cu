#include "cu_medium.h"
#include "launch_utils.h"
#include "InvalidValue.h"
#include "Image.h"
namespace loo
{
	template<typename To, typename Ti>
	__global__ void KernMediumFilter( Image<To> dOut, const Image<Ti> dIn,  int size, float near, float far) 
	{
		const uint x = blockIdx.x*blockDim.x + threadIdx.x;
		const uint y = blockIdx.y*blockDim.y + threadIdx.y;

		if( dOut.InBounds(x,y)) {
			float value = 0;
			for(int r = -size; r <= size; ++r ) {
				for(int c = -size; c <= size; ++c ) {
					const Ti q = dIn.GetWithClampedRange(x+c, y+r);
					value = (q != UINT16_MAX) ? (value + 1) : value;
				}
			}
			int ssize = size * 2 + 1;
			dOut(x,y) = value > ( ssize * ssize * 0.5 ) ? dIn(x,y) : (float) UINT16_MAX;
		}
	}

	template<typename To, typename Ti>
	void MediumFilter( Image<To> dOut, const Image<Ti> dIn, uint size, float near, float far) 
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim,gridDim, dOut);
		KernMediumFilter<To,Ti><<<gridDim,blockDim>>>(dOut, dIn, size, near, far);
	}

	template  __declspec(dllexport) void MediumFilter(Image<float>, const Image<float>, uint, float, float);
}