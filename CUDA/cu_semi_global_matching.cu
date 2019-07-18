#include "cu_semi_global_matching.h"

#include "launch_utils.h"
#include "patch_score.h"
#include "disparity.h"
#include "InvalidValue.h"
#include "ImageApron.h"
#include "CostVolElem.h"

namespace loo
{

	//template<typename TH, typename TC>
	//__global__ void KernSemiGlobalMatchingFast(Volume<TH> volH, Volume<TC> volC, Image<unsigned char> left, int maxDispVal, float P1, float P2, int xoffset, int yoffset, int dx, int dy, unsigned pathlen)
	//{
	//    int x = xoffset + blockIdx.x*blockDim.x + threadIdx.x;
	//    int y = yoffset + blockIdx.y*blockDim.y + threadIdx.y;
	//    int d = yoffset + blockIdx.z*blockDim.z + threadIdx.z;
	//}

	template<typename TH, typename TC, typename Timg>
	__global__ void KernSemiGlobalMatching(Volume<TH> volH, Volume<TC> volC, Image<Timg> left, int maxDispVal, float P1, float P2, int xoffset, int yoffset, int dx, int dy, unsigned pathlen)
	{
		const float MAX_ERROR = 1E30;
		int x = xoffset + blockIdx.x*blockDim.x + threadIdx.x;
		int y = yoffset + blockIdx.y*blockDim.y + threadIdx.y;

		TH lastBestCr = 0;
		Timg last_c = left(x,y);

		const int maxDisp = min(maxDispVal,x+1);
		int lastMaxDisp = maxDisp;
		for(int d=0; d < maxDisp; ++d) {
			volH(x,y,d) += volC(x,y,d);
		}

		x += dx;
		y += dy;

		for(int r=1; r<pathlen; ++r)
		{
			const Timg c = left(x,y);
			const float diff = last_c-c;
			const float _P2 = P2 / (1.0f+fabs(diff) );
			TH bestCr = MAX_ERROR;
			const int maxDisp = min(maxDispVal,x+1);
#pragma unroll 128
			for(int d=0; d < maxDisp; ++d) {
				TH CM = lastBestCr + _P2;
				if(d<lastMaxDisp)     CM = min(CM,volH(x-dx,y-dy,d));
				if(d>0)               CM = min(CM,volH(x-dx,y-dy,d-1)+P1);
				if(d+1 < lastMaxDisp) CM = min(CM,volH(x-dx,y-dy,d+1)+P1);
				const TH Cr = CM + volC(x,y,d) - lastBestCr;
				bestCr = min(bestCr, Cr);
				volH(x,y,d) += Cr;
			}
			x += dx;
			y += dy;
			lastBestCr = bestCr;
			last_c = c;
			lastMaxDisp = maxDisp;
		}
	}

	template<typename TH, typename TC, typename Timg>
	void SemiGlobalMatching(Volume<TH> volH, Volume<TC> volC, Image<Timg> left, int maxDisp, float P1, float P2, bool dohoriz, bool dovert, bool doreverse)
	{
		volH.Memset(0);
		dim3 blockDim(volC.w, 1);
		dim3 gridDim(1, 1);
		if(dovert) {
			KernSemiGlobalMatching<<<gridDim,blockDim>>>(volH,volC,left,maxDisp,P1,P2,0,0,0,1,volC.h);
			if(doreverse) {
				KernSemiGlobalMatching<<<gridDim,blockDim>>>(volH,volC,left,maxDisp,P1,P2,0,volC.h-1,0,-1,volC.h);
			}
		}

		if(dohoriz) {
			dim3 blockDim2(1, volC.h);
			dim3 gridDim(1, 1);
			KernSemiGlobalMatching<<<gridDim,blockDim2>>>(volH,volC,left,maxDisp,P1,P2,0,0,1,0,volC.w);
			if(doreverse) {
				KernSemiGlobalMatching<<<gridDim,blockDim2>>>(volH,volC,left,maxDisp,P1,P2,volC.w-1,0,-1,0,volC.w);
			}
		}
	}

	template __declspec(dllexport) void SemiGlobalMatching(Volume<float> volH, Volume<CostVolElem> volC, Image<unsigned char> left, int maxDisp, float P1, float P2, bool dohoriz, bool dovert, bool doreverse);
	template __declspec(dllexport) void SemiGlobalMatching(Volume<float> volH, Volume<float> volC, Image<float> left, int maxDisp, float P1, float P2, bool dohoriz, bool dovert, bool doreverse);

}
