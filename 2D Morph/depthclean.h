#pragma once

#include "StdAfx.h"
inline bool bound(const float& x, const float & min, const float & max)
{
	return ( x > min && x < max);
}
//clip_weight from 0 - 1
void unwanted_mask(float* img, int idx_k, float2& param_l0, float2& param_l1, float2& param_l)
{
	BYTE valid_depth = 1;
	BYTE Head_Top = 2;
	BYTE Up_Arm_Conner = 3;
	BYTE Low_Arm_Conner = 4;
	BYTE Up_Arm_Segline = 5;
	BYTE Low_Arm_Segline = 6;
	BYTE Head_segline = 7;
	BYTE Arm_Conner2Conner_segline = 8;
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
				if(bound(img[x + r + (y + c) * w], 500, 2500)) 
					count++;
			}
			mask[i] = (count > 0) ? valid_depth : 0;
	});
	//2. Y_Head_top
	int2 Pix_Head_top = make_int2(0,0);
	for(int x = w - 100 ; x > 0 ; x--)
	{
		bool state_change = false;
		for(int y = 100; y < 350; y++)
		{
			if((bool)(mask[x + y * w]) ^ (bool)(mask[x + (y - 1 ) * w]))
			{
				state_change = true;
				Pix_Head_top = make_int2(x, y);
				break;
			}
		}
		if(state_change)
			break;
	}
	for(int x = 0 ; x < w ; x++)
	{
		mask[x + Pix_Head_top.y * w] = Head_segline;
	}
	mask[Pix_Head_top.x + Pix_Head_top.y * w] = Head_Top;


	//3.1 Locate Upper arm cornner
	int2 Pix_ArmUp_Conner = make_int2(0,0);;
	for(int x = Pix_Head_top.x - 100; x > Pix_Head_top.x - 300 ; x-= 3)
	{
		int count = 0;
		int2 last_state_change_pix;

		for(int y = 20 ; y < Pix_Head_top.y; y++)
		{
			if((bool)(mask[x + y * w]) ^ (bool)(mask[x + (y - 1) * w]))
			{
				if((bool)(mask[x + y * w]) ^ (bool)(mask[x + (y - 3) * w]))
				{
					count++;
					last_state_change_pix = make_int2(x, y);
				}
			}
		}
		if(count == 3)
		{
			Pix_ArmUp_Conner = last_state_change_pix;
			break;
		}
	}

	mask[Pix_ArmUp_Conner.x + Pix_ArmUp_Conner.y * w] = Up_Arm_Conner;

	//3.2 Locate Lower arm conner
	int2 Pix_Armlow_Conner = make_int2(0,0);
	for(int x = Pix_Head_top.x - 100; x > Pix_Head_top.x - 300 ; x-= 3)
	{
		int count = 0;
		int2 last_state_change_pix;
		for(int y = h - 20 ; y > Pix_Head_top.y; y--)
		{
			if( (bool)(mask[x + y * w]) ^ (bool)(mask[x + (y + 1) * w]))
			{
				if((bool)(mask[x + y * w]) ^ (bool)(mask[x + (y + 3) * w]))
				{
					count++;
					last_state_change_pix = make_int2(x, y);
				}
			}
		}
		if(count == 3)
		{
			Pix_Armlow_Conner = last_state_change_pix;
			break;
		}
	}

	mask[Pix_Armlow_Conner.x + Pix_Armlow_Conner.y * w] = Low_Arm_Conner;

	int2 zero = make_int2(0,0);
	if(Pix_Armlow_Conner.x == 0 && Pix_ArmUp_Conner.x != 0)
	{
		Pix_Armlow_Conner = make_int2(Pix_ArmUp_Conner.x, 2 *Pix_Head_top.y - Pix_ArmUp_Conner.y);
	}
	if(Pix_ArmUp_Conner.x == 0 && Pix_Armlow_Conner.x != 0)
	{
		Pix_ArmUp_Conner = make_int2(Pix_Armlow_Conner.x, 2 *Pix_Head_top.y - Pix_Armlow_Conner.y);
	}

	//3.3 Draw Arm-Arm Segment Line
	float2 P0 = make_float2(Pix_ArmUp_Conner);
	float2 P1 = make_float2(Pix_Armlow_Conner);

	float k = P0.x == P1.x  ? 999 :(P0.y - P1.y) /(P0.x - P1.x); 
	float b = P0.y - k * P0.x;
	//y = kx + b;
	//x = (y - b )/ k
	for(int y = Pix_ArmUp_Conner.y; y < Pix_Armlow_Conner.y; y++)
	{
		int x = floor((y - b) / k);
		mask[y * w + x] = Arm_Conner2Conner_segline;
	}

	//4.1 Chose Up Segment Line
	float k0 = tanf(15 * 3.14 / 180);
	float b0 = P0.y - k0 * P0.x;

	for(int x = Pix_ArmUp_Conner.x; x > 0; x--)
	{
		int y = floor(k0 * x + b0);
		mask[y * w + x] = Up_Arm_Segline;
	}
	//4.2 Choose Low Segment Line
	float k1 = tanf(-15 * 3.14 / 180);
	float b1 = P1.y - k1 * P1.x;
	for(int x = Pix_Armlow_Conner.x; x > 0; x--)
	{
		int y = floor(k1 * x + b1);
		mask[y * w + x] = Low_Arm_Segline;
	}

	PNG_IO::BYTE_Mark8_array_to_PNG(mask, string(to_string(idx_k) +"_test.png").c_str(), w, h);
	delete [] mask;
	param_l0 = make_float2(k0, b0);
	param_l1 = make_float2(k1, b1);
	param_l  = make_float2(k, b);
}
//inline float line(int2 pose, float k, float b)
//{
//	return k *  pose.x + b - pose.y;
//}