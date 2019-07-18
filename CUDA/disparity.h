#pragma once

#include "Image.h"
#include "InvalidValue.h"

namespace loo
{

inline __device__ __host__
float4 DepthFromDisparity(float u, float v, float disp, float baseline, float fu, float fv, float u0, float v0, float minDisp = 0.0f )
{
    float4 P;
    P.z = disp >= minDisp ? fu * baseline / disp : InvalidValue<float>::Value();

    // (x,y,1) = kinv * (u,v,1)'
    P.x = P.z * (u-u0) / fu;
    P.y = P.z * (v-v0) / fv;
    P.w = 1;
    return P;
}

}
