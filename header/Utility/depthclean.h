#pragma once
#include "StdAfx.h"
#include "ppl.h"
#include "IMAGE_DISPLAY/png_io.h"
#include "Base.h"
using namespace std;
class depthclean
{
public:
	inline static bool bound(const float& x, const float & min, const float & max)
	{
		return ( x > min && x < max);
	}

	//return the up and lower shoulder points
	static void unwanted_mask(float* img, int idx_k, Vector2i& UpVPoint, Vector2i& LowVPoint, Vector2i& PivotPoint)
	{
		BYTE valid_depth = 1;
		Vector2i MidVPoint;
		UpVPoint.setZero();
		LowVPoint.setZero();
		MidVPoint.setZero();
		//1. region grow
		int h = DEPTH_RAW_HEIGHT;
		int w = DEPTH_RAW_WIDTH;
		BYTE*  mask = new BYTE[h * w];
		Concurrency::parallel_for(0u, (UINT) h * w, [&](UINT i){
			float count = 0;
			int x = i % w;
			int y = i / w;
			if(!bound(x , 0 , w)) return;
			if(!bound(y , 0 , h)) return;
			for(int r = -1; r < 1 ; r++)
				for(int c = -1; c < 1 ; c++)
				{
					if(bound(img[x + r + (y + c) * w], 500, 1700)) 
						count++;
				}
				mask[i] = (count > 0) ? valid_depth : 0;
		});


		//2. MidVPoint
		int x, y;
		for(x = 150; x < 512; x += 2)
		{
			int cnt = 0;
			int lasty = 0;
			vector<int> yy;
			yy.reserve(4);
			for(y = 10; y < 400; y +=2)
			{
				if((bool)(mask[y * w + x]) ^ (bool)(mask[(y + 2) * w + x]))
				{
					cnt++;
					yy.push_back(y);
				}
			}
			if(cnt == 2)
			{
				y = (yy[0] + yy[1]) / 2;
				break;
			}
		}

		if(x >= 512)
			MidVPoint.setZero();
		else
			MidVPoint = Vector2i(x, y);


		//3. max depth & min depth
		int val_max = INT_MIN;
		int val_min = INT_MAX;
		Vector2i Pos_max;
		Vector2i Pos_min;
		for(x = MidVPoint.x(); x < 512; x += 3)
		{
			for(y = 0; y < 424; y += 3)
			{
				if(!bound(img[x + y* w], 500, 1700))
					continue;
				int curr = img[x + y* w];
				if(curr > val_max)
				{
					val_max = curr;
					Pos_max = Vector2i(x , y);
				}
				if(curr < val_min)
				{
					val_min = curr;
					Pos_min = Vector2i(x , y);
				}
			}
		}

		Pos_min.x() += 15;
		Pos_max.x() += 15;

		if((!bound(Pos_min.x(), 0, 512)) || (!bound(Pos_min.y(), 0, 424)))
			Pos_min.setZero();
		if((!bound(Pos_max.x(), 0, 512)) || (!bound(Pos_max.y(), 0, 424)))
			Pos_max.setZero();

		//4. LowVPoint
		if(idx_k == 5)
		{
			x = Pos_min.x();
			for(; x < 500; x++)
			{
				int cnt = 0;
				for(y = MidVPoint.y(); y < 400; y++)
				{
					if((bool)(mask[y * w + x]) ^ (bool)(mask[(y + 1) * w + x]))
						cnt++;
				}
				if(cnt == 1) break;
			}

			int x_Lo = x;
			if(bound(x, 0, 512))
			{
				int xx = x_Lo - 3;
				int yy = MidVPoint.y();
				for(; yy < 424; yy++)
				{
					if((bool)(mask[yy * w + xx]) ^ (bool)(mask[(yy + 1) * w + xx]))	break;
				}
				if(bound(yy, 0, 424))
					LowVPoint = Vector2i(xx, yy);
			}

			/*
			float lk = tanf(-25 * 3.14 / 180);
			float lb = LowVPoint.y() - lk * LowVPoint.x();
			for(int xxx = 0; xxx < 512; xxx++)
			{
				int yyy = xxx * lk + lb;
				if(y > 0 && y < 424)
					mask[xxx + yyy * 512] = 7;
			}
			*/
		}
		else
		{
			LowVPoint = Vector2i(0, 423);
		}
		//5. UpVPoint
		if(idx_k == 3)
		{
			x = Pos_min.x();

			for(; x < 500; x ++)
			{
				int cnt = 0;
				for(y = MidVPoint.y(); y > 10; y--)
				{
					if((bool)(mask[y * w + x]) ^ (bool)(mask[(y - 1) * w + x]))
						cnt++;
				}
				if(cnt == 1)
					break;
			}

			int x_Up = x;

			if(bound(x_Up, 0, 512))
			{
				int xx = x_Up - 3;
				int yy = MidVPoint.y();
				for(; yy > 0; yy--)
				{
					if((bool)(mask[yy * w + xx]) ^ (bool)(mask[(yy - 1) * w + xx]))	break;
				}
				if(bound(yy, 0, 424))
					UpVPoint = Vector2i(xx, yy);
			}
			/*
			float lk = tanf(25 * 3.14 / 180);
			float lb = UpVPoint.y() - lk * UpVPoint.x();
			for(int xxx = 0; xxx < 512; xxx++)
			{
				int yyy = xxx * lk + lb;
				if(y > 0 && y < 424)
					mask[xxx + yyy * 512] = 2;
			}*/
		}
		else
		{
			UpVPoint = Vector2i(0,0);
		}

		mask[UpVPoint.x() + UpVPoint.y() * 512] =2;
		mask[LowVPoint.x() + LowVPoint.y() * 512] = 2;
		mask[MidVPoint.x() + MidVPoint.y() * 512] = 2;
		mask[Pos_min.x() + Pos_min.y() * 512] = 2;
		mask[Pos_max.x() + Pos_max.y() * 512] = 2;
		PivotPoint = MidVPoint;
		PNG_IO::BYTE_Mark8_array_to_PNG(mask, string(to_string(idx_k) +"_test.png").c_str(), w, h);
		delete [] mask;
	}

};
