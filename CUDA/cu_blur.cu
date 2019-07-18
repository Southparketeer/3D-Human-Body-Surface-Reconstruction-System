#include "cu_blur.h"

#include "launch_utils.h"
#include "ImageApron.h"

#ifndef M_PI
// Some trouble with Maths defines with MSVC
#define M_PI 3.14159265358979323846
#endif

namespace loo {

//////////////////////////////////////////////////////
// Small Radius Gaussian Blur
//////////////////////////////////////////////////////

template<typename TO, typename TI>
__global__ void KernBlurX(Image<TO> out, Image<TI> in)
{
    const unsigned x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x==0) {
        out(x,y) = (2*in(x,y) + in(x+1,y)) / 3.0f;
    }else if(x== in.w-1) {
        out(x,y) = (2*in(x,y) + in(x-1,y)) / 3.0f;
    }else{
        out(x,y) = (in(x-1,y) + 2*in(x,y) + in(x+1,y)) / 4.0f;
    }
}

template<typename TO, typename TI>
__global__ void KernBlurY(Image<TO> out, Image<TI> in)
{
    const unsigned x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned y = blockIdx.y*blockDim.y + threadIdx.y;

    if(y==0) {
        out(x,y) = (2*in(x,y) + in(x,y+1)) / 3.0f;
    }else if(y== in.h-1) {
        out(x,y) = (2*in(x,y) + in(x,y-1)) / 3.0f;
    }else{
        out(x,y) = (in(x,y-1) + 2*in(x,y) + in(x,y+1)) / 4.0f;
    }
}

void Blur(Image<unsigned char> out, Image<unsigned char> in, Image<unsigned char> temp )
{
    dim3 blockDim, gridDim;
    InitDimFromOutputImage(blockDim,gridDim, out, 16, 16);
    KernBlurX<unsigned char,unsigned char><<<gridDim,blockDim>>>(temp,in);
    KernBlurY<unsigned char,unsigned char><<<gridDim,blockDim>>>(out,temp);
}

//////////////////////////////////////////////////////
// Larger radius Gaussian Blur
// http://http.developer.nvidia.com/GPUGems3/gpugems3_ch40.html
//////////////////////////////////////////////////////

template<typename TO, typename TI, unsigned MAXBW, unsigned MAXBH, unsigned RAD>
__global__ void KernGaussianBlurX(Image<TO> out, Image<TI> in, float g0, float g1)
{
    const unsigned x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned y = blockIdx.y*blockDim.y + threadIdx.y;

    __shared__ ImageApronRows<TI,MAXBW,MAXBH,0> apron;
    apron.CacheImage(in);
    __syncthreads();

    if(out.InBounds(x,y)) {
        float pixsum = 0;
        float gsum = 0;
        float g2 = g1 * g1;
#pragma unroll
        for (int i = 0; i < RAD; i++) {
            // g0 is current gaussian coefficient for sample i
            gsum += g0;
            pixsum += g0 * apron.GetRelThreadClampX(i,0);
            pixsum += g0 * apron.GetRelThreadClampX(-i,0);
            g0 *= g1;
            g1 *= g2;
        }
        out(x,y) = max(0.0f,min(pixsum / (2*gsum),255.0f));
    }
}

template<typename TO, typename TI, unsigned MAXBW, unsigned MAXBH, unsigned RAD>
__global__ void KernGaussianBlurY(Image<TO> out, Image<TI> in, float g0, float g1)
{
    const unsigned x = blockIdx.x*blockDim.x + threadIdx.x;
    const unsigned y = blockIdx.y*blockDim.y + threadIdx.y;

    __shared__ ImageApronRows<TI,MAXBW,MAXBH,0> apron;
    apron.CacheImage(in);
    __syncthreads();

    if(out.InBounds(x,y)) {
        float pixsum = 0;
        float gsum = 0;
        float g2 = g1 * g1;
#pragma unroll
        for (int i = 0; i <= RAD; i++) {
            // g0 is current gaussian coefficient for sample i
            gsum += g0;
            pixsum += g0 * apron.GetRelThreadClampY(0, i);
            pixsum += g0 * apron.GetRelThreadClampY(0,-i);
            g0 *= g1;
            g1 *= g2;
        }
        out(x,y) = max(0.0f,min(pixsum / (2*gsum),255.0f));
    }
}

template<typename Tout, typename Tin, unsigned MAXRAD, unsigned MAXIMGDIM>
void GaussianBlur(Image<Tout> out, Image<Tin> in, Image<Tout> temp, float sigma)
{
    if(sigma == 0 ) {
        out.CopyFrom(in);
    }else{
        dim3 blockDim, gridDim;

        const float delta = 1;
        const float g0 = 1.0 / (sqrt(2.0 * M_PI) * sigma);
        const float g1 = exp(-0.5 * delta * delta / (sigma * sigma));

        InitDimFromOutputImageOver(blockDim,gridDim, out, out.w,1);
        KernGaussianBlurX<unsigned char, unsigned char, MAXIMGDIM, 1, MAXRAD><<<gridDim,blockDim>>>(temp,in, g0, g1);

        InitDimFromOutputImageOver(blockDim,gridDim, out, 1, out.h);
        KernGaussianBlurY<unsigned char, unsigned char, 1, MAXIMGDIM, MAXRAD><<<gridDim,blockDim>>>(out,temp, g0, g1);
    }
}

template  __declspec(dllexport) void GaussianBlur<unsigned char,unsigned char, 5,  1024>(Image<unsigned char>, Image<unsigned char>, Image<unsigned char>, float);
template  __declspec(dllexport) void GaussianBlur<unsigned char,unsigned char, 10, 1024>(Image<unsigned char>, Image<unsigned char>, Image<unsigned char>, float);
template  __declspec(dllexport) void GaussianBlur<unsigned char,unsigned char, 15, 1024>(Image<unsigned char>, Image<unsigned char>, Image<unsigned char>, float);
template  __declspec(dllexport) void GaussianBlur<unsigned char,unsigned char, 20, 1024>(Image<unsigned char>, Image<unsigned char>, Image<unsigned char>, float);


}
