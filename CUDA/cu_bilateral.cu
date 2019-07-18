#include "cu_bilateral.h"
#include "launch_utils.h"
#include "InvalidValue.h"
#include "Image.h"
namespace loo
{
	template<typename To, typename Ti>
	__global__ void KernBilateralFilter(
		Image<To> dOut, const Image<Ti> dIn, float gs, float gr, int size
		) {
			const uint x = blockIdx.x*blockDim.x + threadIdx.x;
			const uint y = blockIdx.y*blockDim.y + threadIdx.y;
		//	dOut(x,y) = dIn(x,y);
			if( dOut.InBounds(x,y)) 
			{
				const Ti p = dIn(x,y) * 0.001;
				float sum = 0;
				float sumw = 0;

				for(int r = -size; r <= size; ++r ) {
					for(int c = -size; c <= size; ++c ) {
						const Ti q = dIn.GetWithClampedRange(x+c, y+r) * 0.001;
						const float sd2 = r*r + c*c;
						const float id = p-q;
						const float id2 = id*id;
						const float sw = __expf(-(sd2) / (2 * gs * gs));
						const float iw = __expf(-(id2) / (2 * gr * gr));
						const float w = sw*iw;
						sumw += w;
						sum += w * q;
					}
				}

				dOut(x,y) = (To)(sum / sumw) * 1000.;
			}
	}
	template<typename To, typename Ti>
	void BilateralFilter( Image<To> dOut, const Image<Ti> dIn, float gs, float gr, uint size) 
	{
		dim3 blockDim, gridDim;
		InitDimFromOutputImageOver(blockDim,gridDim, dOut);
		KernBilateralFilter<To,Ti><<<gridDim,blockDim>>>(dOut, dIn, gs, gr, size);
	}

	template  __declspec(dllexport) void BilateralFilter(Image<float>, const Image<float>, float, float, uint);
	template  __declspec(dllexport) void BilateralFilter(Image<float>, const Image<unsigned char>, float, float, uint);
}