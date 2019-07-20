#pragma once
#include "StdAfx.h"
#include "ppl.h"
class regiongrow
{
public:
	inline static bool check_depth(const float& a, const float& n, const float& f)
	{
		return (a >= n && a <= f);
	}
	static void RegionGrowW(float * src, float dn, float df, float gap, int w, int h)
	{
		bool * buffer = new bool[w * h];
		int * queue = new int[w * h];
		bool * mask = new bool[w * h];
		memset(buffer, false, sizeof(bool) * w * h);
		memset(mask, false, sizeof(bool) * w * h);

		for(int y = 0 ; y < h ; y++)
		{
			for(int x = 0 ; x < w ; x++)
			{
				int index = x + y * w;

				if(!check_depth(src[index], dn, df)) continue;
				if(buffer[index]) continue;

				int begin = 0; 
				int end = 0;
				queue[ end++ ] = index;
				buffer[ index ] = 1;
				while(begin < end)
				{
					int k = queue[ begin++ ];
					int x = k % w;
					int y = k / w;
					if(x == 0 || y == 0 || x == w-1 || y == h-1) continue;

					int n;
					//1
					n = (x - 1) + y * w;
					if(check_depth(src[n], dn, df) && !buffer[n] && fabs(src[n] - src[index]) < gap)
					{
						queue[ end++ ] = n;
						buffer[ n ] = true;
					}
					//2
					n = x + (y - 1) * w;
					if(check_depth(src[n], dn, df) && !buffer[n] && fabs(src[n] - src[index]) < gap)
					{
						queue[ end++ ] = n;
						buffer[ n ] = true;
					}
					//3
					n = (x + 1) + y * w;
					if(check_depth(src[n], dn, df) && !buffer[n] && fabs(src[n] - src[index]) < gap)
					{
						queue[ end++ ] = n;
						buffer[ n ] = true;
					}
					//4
					n = x + (y + 1) * w;
					if(check_depth(src[n], dn, df) && !buffer[n] && fabs(src[n] - src[index]) < gap)
					{
						queue[ end++ ] = n;
						buffer[ n ] = true;
					}
				}

				if ( end < 1000) {
					continue;
				}
				for ( int i = 0; i < end; i++ ) {
					int k = queue[ i ];
					mask[k] = 1;
				}
			}
		}
# if 0
		Concurrency::parallel_for(0, w * h, [&](int k){
			if(mask[k]) return;
			int x = k % w;
			int y = k / w;
			if(x == 0 || x == w - 1 || y == 0 || y == h - 1) 
			{
				src[k] = UINT16_MAX;
				return;
			}		
			int cut = 0;
			float sum = 0;
			for(int c = x - 1 ; c <= x + 1; c++)
			{
				for(int r = y - 1; r < y + 1 ; r++)
				{
					if(mask[r * w + c]) 
					{
						cut ++;
						sum += src[r * w + c];
					}

				}
			}

			if(cut == 0)
			{
				src[k] = UINT16_MAX;
				return;
			}
			else
			{
				src[k] = sum / (float) cut;
			}		
		});
#else
		Concurrency::parallel_for(0, DEPTH_RAW_WIDTH * DEPTH_RAW_HEIGHT, [&](int k){
			if(mask[k]) return;
			src[k] = UINT16_MAX;
		});

#endif

		SAFE_DELETE_ARRAY(buffer);
		SAFE_DELETE_ARRAY(queue);
		SAFE_DELETE_ARRAY(mask);
	}
};