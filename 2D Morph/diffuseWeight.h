#pragma once
#include "math.h"
#include <vector>
#include "..\IMAGE_DISPLAY/png_io.h"
#include "..\Base.h"

using namespace std;
using namespace vcg;
struct wcombo
{
	enum segment
	{
		torso = 0,
		head = 1,
		armLeft = 2,
		armRight = 3,
		legLeft = 4,
		legRight = 5
	};

	float w[6];

	wcombo()
	{
		for(auto& i : w)
		{
			i = 0;
		}
	}
	void assign(const wcombo& w2)
	{
		for(int i = 0 ; i < 6; i++)
		{
			w[i] = w2.w[i];
		}
	}
	void reset()
	{
		memset(w, 0, 6 * sizeof(float));
	}
	void sum(const wcombo& w2)
	{
		for(int i = 0 ; i < 6; i++)
		{
			w[i] += w2.w[i];
		}
	}

	void divC(const float& d)
	{
		for(auto& i : w)
		{
			i /= d;
		}
	}

	void mulC(const float& d)
	{
		for(auto& i : w)
		{
			i *= d;
		}
	}


	bool isUnset()
	{
		double sum = 0.0;
		for(auto& i : w)
		{
			sum += i;
		}
		return sum == 0;
	}

	void normalize()
	{
		if(isUnset())
			return;

		double sum = 0.0;
		for(auto& i : w)
			sum += i;
		for(auto& i : w)
			i /= sum;
	}
};

//input binary float
class diffuseWeight
{
public: 
	diffuseWeight()
	{
		weight.clear();
		image.clear();
		JointControl.clear();
		w = 0; 
		h = 0;
	}
	void init(char * file, int img_w, int img_h);
	double getDiffuseCoff(const Point2i& curr,const float& rad);
	void weightDiffuse(int iteration = 100);

	std::vector<wcombo> weight;
	vector<Point3f> image;
	vector<Point2i> JointControl;

	int w;
	int h;
};

void diffuseWeight::weightDiffuse(int iteration)
{
	if(weight.size() == 0)
		return;
	vector<wcombo> temp_weight(w * h);
	vector<float> diffusecoff(w * h);
	for(int i = 0; i < w * h ; i++)
	{
		diffusecoff[i] = getDiffuseCoff(Point2i(i % w, i / w), 20);
	}
	int k = 0;
	while(k < iteration)
	{
		for(int y = 1; y < h - 1; y++)
		{
			for(int x = 1; x < w - 1; x++)
			{

				float diffcoff = diffusecoff[x + y * w];//getDiffuseCoff(Point2i(x, y), 20);
				if(image[x + y * w] == Point3f(0,0,0))
					continue;
				wcombo neighbor;
				wcombo self = weight[x + y * w];
				int count = 0;
				for(int j = -1; j <= 1; j++)
				{
					for(int i = -1; i <= 1; i ++)
					{
						if(i == 0 && j == 0)
							continue;
						int xx = x + i;
						int yy = y + j;
						if(image[xx + yy * w] == Point3f(0,0,0))
							continue;
						count++;
						neighbor.sum(weight[xx + yy * w]);
					}
				}
				neighbor.divC((float)count);
				neighbor.mulC(diffcoff);
				self.mulC(1 - diffcoff);
				temp_weight[x + y * w].sum(neighbor);
				temp_weight[x + y * w].sum(self);
				temp_weight[x + y * w].normalize();

			}
		}


		for(int i = 0; i < w * h ; i++)
		{
			weight[i].assign(temp_weight[i]);
			temp_weight[i].reset();
		}
		k++;
	}
	////-----------------TEST---------------//
	//vector<BYTE> test_image_buff(w * h * 3);
	//vector<Point3f> ColorTable(6);
	//ColorTable[0] = Point3f(255,0,0);
	//ColorTable[1] = Point3f(0, 0, 0);
	//ColorTable[2] = Point3f(0,255,0);
	//ColorTable[3] = Point3f(0,0,255);
	//ColorTable[4] = Point3f(255,255,0);
	//ColorTable[5] = Point3f(0,255,255);
	//for(int i = 0; i < w * h ; i++)
	//{
	//	Point3f weightedColor(0,0,0);
	//	for(int k = 0 ; k < 6; k++)
	//	{
	//		weightedColor += ColorTable[k] * weight[i].w[k];
	//		test_image_buff[i * 3 + 0] = (BYTE)weightedColor.X();
	//		test_image_buff[i * 3 + 1] = (BYTE)weightedColor.Y();
	//		test_image_buff[i * 3 + 2] = (BYTE)weightedColor.Z();
	//	}
	//}
	//PNG_IO::BYTE_array_to_PNG(test_image_buff.data(), "Test_diffusion_multi.png", w, h);
}

double diffuseWeight::getDiffuseCoff(const Point2i& curr,const float& rad)
{
	if(JointControl.size() == 0)
		return 0;
	float min_val = rad;
	for(int i = 1; i < JointControl.size(); i++)
	{
		Point2f offset((curr - JointControl[i]).X(), (curr - JointControl[i]).Y());
		min_val = min(min_val, offset.Norm());
	}
	if(min_val == rad)
	{
		return 1.;
	}
	float rat = 1 - (rad - min_val) / rad;
	float w = rat > 1. ? 1. : 1 - (cos((2 * rat - rat * rat) * 3.14159) * .5 + .5);
	return w;
}

void diffuseWeight::init(char * file, int img_w, int img_h)
{
	w = img_w;
	h = img_h;
	weight.resize(w * h);
	image.resize(w * h);
	JointControl.resize(6);
	for(int k = 0; k < 6; k++)
	{
		JointControl[k].SetZero();
	}
	PNG_IO::PNG_to_array_BYTE(file, image, w, h);
	for(int y = 0 ; y < img_h; y++)
	{
		for(int x = 0; x < img_w; x++)
		{
			Point3f curr = image[y * w + x];
			if(curr == Point3f::Zero())
				continue;
			if(curr == Point3f(0,255,0))
				weight[y * w + x].w[wcombo::armLeft] = 1.0;
			else if(curr == Point3f(0,0,255))
				weight[y * w + x].w[wcombo::armRight] = 1.0;
			else if(curr == Point3f(255,255,0))
				weight[y * w + x].w[wcombo::legLeft] = 1.0;
			else if(curr == Point3f(0,255,255))
				weight[y * w + x].w[wcombo::legRight] = 1.0;
			else 
			{
				weight[y * w + x].w[wcombo::torso] = 1.0;
				if(curr != Point3f(255,0,0))
				{
					JointControl[(int)(curr.X() / 32)] = Point2i(x, y);
				}
			}
		}
	}
}