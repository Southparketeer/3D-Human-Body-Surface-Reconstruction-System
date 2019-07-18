#include "cu_MCmask.h"
#include "Sdf.h"

namespace loo
{
	__device__ inline float fGetOffset(float fValue1, float fValue2, float fValueDesired)
	{
		const double fDelta = fValue2 - fValue1;
		if(fDelta == 0.0) {
			return 0.5;
		}
		return (fValueDesired - fValue1)/fDelta;
	}

	__device__ inline bool isfinite(float s)
	{
		// By IEEE 754 rule, 2*Inf equals Inf
		return (s == s) && ((s == 0) || (s != 2*s));
	}

	__global__ void KernMarchingCubeMaskGPU(BoundedVolume<SDF_t> vol, BoundedVolume<uchar2> vol_mask, float trunc_dist, float3 ray_K0, float3 ray_K1)
	{
		const int x = blockIdx.x*blockDim.x + threadIdx.x;
		const int y = blockIdx.y*blockDim.y + threadIdx.y;
		const int z = blockIdx.z*blockDim.z + threadIdx.z;
		const float fTargetValue = 0.0;
		float afCubeValue[8];
		int iFlagIndex = 0;
		int iFlagTriangle = 0;

		vol_mask(x,y,z) = make_uchar2(0,0);
		//if (vol(x,y,z).w < 1)	{ vol_mask(x,y,z) = make_uchar2(0,0);	return;	}
		for(int iVertex = 0 ; iVertex < 8 ; iVertex ++)
		{
			afCubeValue[iVertex] = vol.Get(x+a2fVertexOffset[iVertex][0],y+a2fVertexOffset[iVertex][1],z+a2fVertexOffset[iVertex][2]);
			
			if(afCubeValue[iVertex] >= 0.95 * (trunc_dist)) 	{ vol_mask(x,y,z) = make_uchar2(0,0);	return;	}
			if(afCubeValue[iVertex] <= -0.95 * (trunc_dist)) { vol_mask(x,y,z) = make_uchar2(0,0);	return;	}
			
		}
		for(int iVertexTest = 0 ; iVertexTest< 8 ; iVertexTest++)
		{
			if(afCubeValue[iVertexTest] <= fTargetValue)
				iFlagIndex |= 1<<iVertexTest;
		}

		int iEdgeFlags = CubeEdgeFlags[iFlagIndex];
		if( iEdgeFlags == 0 ) { vol_mask(x,y,z) = make_uchar2(0,0);	return; }

		//----------------------
		const float3 p = vol.VoxelPositionInUnits(x,y,z);
		const float3 fScale = vol.VoxelSizeUnits();

		//Find the point of intersection of the surface with each edge
		//Then find the normal to the surface at those points
		float3 asEdgeVertex[12];
		float3 asEdgeNorm[12];

		for(int iEdge = 0; iEdge < 12; iEdge++)
		{
			//if there is an intersection on this edge
			if(iEdgeFlags & (1<<iEdge))
			{
				//float afCubeValue[8];
				float fOffset = fGetOffset(afCubeValue[ a2iEdgeConnection[iEdge][0] ], afCubeValue[ a2iEdgeConnection[iEdge][1] ], fTargetValue);

				asEdgeVertex[iEdge] = make_float3(
					p.x + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][0]  +  fOffset * a2fEdgeDirection[iEdge][0]) * fScale.x,
					p.y + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][1]  +  fOffset * a2fEdgeDirection[iEdge][1]) * fScale.y,
					p.z + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][2]  +  fOffset * a2fEdgeDirection[iEdge][2]) * fScale.z
					);

				const float3 deriv = vol.GetUnitsBackwardDiffDxDyDz( asEdgeVertex[iEdge] );
				asEdgeNorm[iEdge] = deriv / length(deriv);
				if( !isfinite(asEdgeNorm[iEdge].x) || !isfinite(asEdgeNorm[iEdge].y) || !isfinite(asEdgeNorm[iEdge].z) ) {
					asEdgeNorm[iEdge] = make_float3(0,0,0);
				}
			}
		}

		for(int iTriangle = 0; iTriangle < 5; iTriangle++)
		{
			if(TriangleConnectionTable[iFlagIndex][3*iTriangle] < 0)
				break;

			for(int iCorner = 0; iCorner < 3; iCorner++)
			{
				int iVertex = TriangleConnectionTable[iFlagIndex][3*iTriangle+iCorner];
			}
			iFlagTriangle |= 1<<iTriangle;
		}
		if(iFlagTriangle == 0) 	{ vol_mask(x,y,z) = make_uchar2(0,0);	return;	}
		vol_mask(x,y,z) = make_uchar2((unsigned char)iFlagIndex,(unsigned char)iFlagTriangle);
		return;
	}

	void MarchingCubeMaskGPU(BoundedVolume<loo::SDF_t> vol, BoundedVolume<uchar2> vol_mask, float trunc_dist, float3 ray_K0, float3 ray_K1 )
	{
		dim3 blockDim(4,4,4);
		dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y, vol.d / blockDim.z);
		KernMarchingCubeMaskGPU<<<gridDim,blockDim>>>(vol, vol_mask, trunc_dist, ray_K0, ray_K1);
		//GpuCheckErrors();
	}
}
