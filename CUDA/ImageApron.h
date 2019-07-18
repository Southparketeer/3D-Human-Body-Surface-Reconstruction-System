#pragma once

#include "Image.h"
#include "InvalidValue.h"

namespace loo
{

	template<typename T, int MAXBW, int MAXBH, int RAD>
	struct ImageApronRows
	{
		const static int ROWS = (2*RAD+1);
		const static int MAXEL = MAXBW * (ROWS*MAXBH);
		T cache[MAXEL];

		inline __device__
			void CacheImage(const Image<T>& img, unsigned x, unsigned y)
		{
#pragma unroll
			for(int r=0; r<ROWS; ++r) {
				const int roffset = r*blockDim.y + threadIdx.y;
				const int yimg = y - RAD + roffset;
				const T val = (0 <= yimg && yimg < img.h) ? img.Get(x+threadIdx.x, yimg ) : 0;
				GetRaw(threadIdx.x, roffset ) = val;
			}
		}

		inline __device__
			void CacheImage(const Image<T>& img)
		{
			CacheImage(img, blockIdx.x*blockDim.x, blockIdx.y*blockDim.y);
		}

		inline __device__
			T& GetRaw(int x, int y)
		{
			return cache[y*blockDim.x + x];
		}

		inline __device__
			T& GetRelBlock(int x, int y)
		{
			return GetRaw(x, y + blockDim.y*RAD);
		}

		inline __device__
			T& GetRelThread(int x, int y)
		{
			return GetRelBlock(threadIdx.x + x, threadIdx.y + y);
		}

		inline __device__
			T GetRelThreadClampX(int x, int y)
		{
			const int elx = threadIdx.x + x;
			const int ely = threadIdx.y + y;
			if( 0 <= elx && elx < blockDim.x) {
				return GetRelBlock(elx, ely);
			}else{
				return 0;
			}
		}

		inline __device__
			T GetRelThreadClampY(int x, int y)
		{
			const int elx = threadIdx.x + x;
			const int ely = threadIdx.y + y;
			if( 0 <= ely && ely < blockDim.y) {
				return GetRelBlock(elx, ely);
			}else{
				return 0;
			}
		}
	};

}
