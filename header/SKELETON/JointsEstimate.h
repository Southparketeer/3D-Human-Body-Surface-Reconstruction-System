#pragma once
#include "glConfig.h"
#include "Model.h"
#include "LoadShader.h"
#include "..\Base.h"
#include "..\Helper.h"
#include <vector>
#include "..\IMAGE_DISPLAY\png_io.h"
#include "..\StdAfx.h"

class JointEstimate{
public:
	JointEstimate()
	{
		_cm = NULL;
		glconfig = NULL;
	}
	~JointEstimate()
	{
		SAFE_DELETE(glconfig);
	}
	void init(vector<CMeshO*>* mesh, int argc, char** argv, bool label_head = false);
	void estimateJ(	CMeshO& m, bool is_front, Point3f& P_torso, Point3f& P_head, Point3f& P_LeftArm, Point3f& P_RightArm, Point3f& P_LeftLeg, Point3f& P_RightLeg);
	void colorJ   (	CMeshO& m, const float& radius,
		const Point3f& P_Torso, const Point3f& P_Head, const Point3f& P_LeftArm, const Point3f& P_RightArm, const Point3f& P_LeftLeg, const Point3f& P_RightLeg,
		const Color4b& C_Torso, const Color4b& C_Head, const Color4b& C_LeftArm, const Color4b& C_RightArm, const Color4b& C_LeftLeg, const Color4b& C_RightLeg);
	void cpusmooth( float *img);
	void analysisJ( const float *img, bool if_front, int2& Px_Torso, int2& Px_Head, int2& Px_LeftArm, int2& Px_RightArm, int2& Px_LeftLeg, int2& Px_RightLeg);
	void estimateJoints(const Color4b& C_Torso, const Color4b& C_head, const Color4b& C_LeftArm, const Color4b& C_RightArm, const Color4b& C_LeftLeg, const Color4b& C_RightLeg);
	void estimateHands(const Color4b& C_LeftHand, const Color4b& C_RightHand);
	void estimateFeet(const Color4b& C_LeftFoot, const Color4b& C_RightFoot);
public:
	Point3f Joint[6];
	vector<CMeshO*>* _cm;
	glConfig * glconfig;
};

inline bool check(const float * img, const int& x, const int& y, const int& w, const int& size = 1)
{
	int count = 0;
	for(int r = y - size; r <= y + size; r++)
	{
		for(int c = x - size; c <= x + size; c++)
		{
			if(img[r * w + c])
				count ++;
		}
	}
	if(count > (size * 2 + 1) * (size * 2 + 1) * 0.5)
		return true;
	return false;
}
void  JointEstimate::cpusmooth( float *img)
{
	int w = glconfig->width;
	int h = glconfig->height;
	float * buffer = new float[ w * h];
	memcpy(buffer, img, w * h * sizeof(float));
	int size = 2;
	Concurrency::parallel_for(0, (int)w * h, [&](int k){
		int x = k % w;
		int y = k / w;
		if( x < size || x > w - size || y < size || y > h - size)
			return;
		img[y * w + x] = check(buffer, x, y, w, size) ? 1 : 0;
	});
	SAFE_DELETE_ARRAY(buffer);
}

inline int counter(vector<int>& m, const int2& Range)
{
	int count = 0;
	for(int i = 0; i < m.size(); i+= 2)
	{
		if(m[i] >= Range.x && m[i] <= Range.y && abs(m[i+1]) >= Range.x && abs(m[i+1]) <= Range.y)
		{
			count ++;
		}
	}
	return count;
}
inline void drawLabel(BYTE * img, const int2& xy, const int & size, int w = 512, int h = 424)
{
	if(xy.y < size || xy.y > h - size - 1 || xy.x < size || xy.x > w - size - 1)
		return;
	for(int r = xy.y - size; r < xy.y + size; r++)
	{
		for(int c = xy.x - size; c < xy.x + size; c++)
		{
			int k = r * w + c;
			img[k * 4 + 0] = 255;
			img[k * 4 + 1] = 0;
			img[k * 4 + 2] = 0;
			img[k * 4 + 3] = 255;
		}
	}
}

#define MANIQ 1
void JointEstimate::analysisJ( const float *img, bool if_front, int2& Px_Torso, int2& Px_Head, int2& Px_LeftArm, int2& Px_RightArm, int2& Px_LeftLeg, int2& Px_RightLeg)
{
	int w = glconfig->width;
	int h = glconfig->height;
	BYTE * img_label = new BYTE[ w * h * 4];
	memset(img_label, 0, w * h * 4 * sizeof(BYTE));

	int step = 1;
	vector<vector<int>> record(h / step);


	for(int y = 0 ; y < h ; y += step)
	{
		bool stage = false;
		const float * ptr = img + y * w;
		for(int x = 1 ; x < w; x++)
		{
			float v0 = *(ptr + x - 1);
			float v1 = *(ptr + x );
			if(v0 == v1) continue;
			int a = v0 < v1 ? x : -x;
			record[y / step].push_back(a);
		}
	}

	//Y position for the feet point(detect 2 feet)
	int Y_feet = 50;
#if MANIQ
	//Y position for the leg Joint
	int Y_hip;
	for(int y = Y_feet; y < h / 2 ; y++)
	{
		if(record[y].size() == 2)
		{
			Y_hip = y - 5;
			break;
		}
	}
#else
	int y_max = DEPTH_RAW_HEIGHT, y_min = 0;
	for(int y = 0 ; y < DEPTH_RAW_HEIGHT; y++)
	{
		if(record[y].size() != 0)
			break;
		if(record[y].size() == 0)
			y_max = y;
	}
	for(int y = DEPTH_RAW_HEIGHT - 1 ; y > 0; y--)
	{
		if(record[y].size() != 0)
			break;
		if(record[y].size() == 0)
			y_max = y;
	}

	int x_max = DEPTH_RAW_WIDTH, x_min = 0;
	for(int x = 0 ; x < DEPTH_RAW_WIDTH; x++)
	{
		bool is_empty = true;
		for(int y = 0; y < DEPTH_RAW_HEIGHT; y++)
		{
			int base = y * w + x;
			if(img[base] != 0)
			{
				is_empty = false;
				break;
			}
		}
		if(is_empty)
		{
			x_min = x;
		}
		else 
		{
			break;
		}
	}

	for(int x = DEPTH_RAW_WIDTH - 1; x > 0; x--)
	{
		bool is_empty = true;
		for(int y = 0; y < DEPTH_RAW_HEIGHT; y++)
		{
			int base = y * w + x;
			if(img[base] != 0)
			{
				is_empty = false;
				break;
			}
		}
		if(is_empty)
		{
			x_max = x;
		}
		else
		{
			break;
		}
	}

	int2 Center = make_int2((x_max + x_min) / 2 , (y_max + y_min) / 2);
#endif
	//Y position for the head topmost possible pos
	int Y_head_up;
	for(int y = h - 1; y > 0; y--)
	{
		if(record[y].size() == 2)
		{
			Y_head_up = y;
			break;
		}
	}
	//Y position for the head lowestmost possible pos
	int Y_head_down;
	for(int y = Y_head_up - 10; y > 0; y--)
	{
		if(record[y].size() != 2)
		{
			Y_head_down = y + 1;
			break;
		}
	}
	//Y position for the Arm Joint
	int Y_arm;
	for(int y = Y_head_down; y > 1; y--)
	{
		if(record[y].size() == 6)
		{
			int x1 = abs(record[y][1]) - abs(record[y][0]);
			int x2 = abs(record[y][3]) - abs(record[y][2]);
			int x3 = abs(record[y][5]) - abs(record[y][4]);
			if(x2 > x1 && x2 > x3)
			{
				Y_arm = y;
				break;
			}
		}
	}

	//Refine Arm & Leg Joints Position
	int2 JointA = make_int2(abs(record[Y_arm][2]), Y_arm);
	int2 JointB = make_int2(abs(record[Y_arm][3]), Y_arm);
	JointA.y += 15;
	JointB.y += 15;

	int hip_width;
#if MANIQ
	int2 JointC = make_int2(abs(record[Y_hip][1]) / 2 + abs(record[Y_hip][2] / 2 ), Y_hip + 25);
	if(record[JointC.y - 20].size() == 2)
		hip_width = abs(record[JointC.y - 20][1]) - abs(record[JointC.y - 20][0]);
	if(record[JointC.y - 20].size() == 4)
		hip_width = abs(record[JointC.y - 20][3]) - abs(record[JointC.y - 20][0]);
#else
	int2 JointC = Center;
	if(record[JointC.y].size() == 6)
		hip_width = abs(record[JointC.y][3]) - abs(record[JointC.y][2]);
	else if(record[JointC.y].size() == 2)
		hip_width = abs(record[JointC.y][1]) - abs(record[JointC.y][0]);
	else if(record[JointC.y].size() == 4)
	{
		JointC.y = 10;
		if(record[JointC.y].size() == 6)
			hip_width = abs(record[JointC.y][3]) - abs(record[JointC.y][2]);
		else if(record[JointC.y].size() == 2)
			hip_width = abs(record[JointC.y][1]) - abs(record[JointC.y][0]);
	}
#endif

	hip_width /= 3.5;
	int2 JointCS =make_int2(JointC.x - hip_width, JointC.y - 20);
	int2 JointCL =make_int2(JointC.x + hip_width, JointC.y - 20);

	Px_Torso = make_int2(JointC.x, JointC.y);

	if(if_front)
	{
		Px_RightArm = JointA;
		Px_LeftArm = JointB;
		Px_RightLeg = JointCS;
		Px_LeftLeg = JointCL;
	}
	else
	{
		Px_RightArm = JointB;
		Px_LeftArm = JointA;
		Px_RightLeg = JointCL;
		Px_LeftLeg = JointCS;
	}
	//Calculate Head Joint Position
	vector<int> head_width((Y_head_up - Y_head_down) / 5);
	for(int y = Y_head_down, c = 0; y + 4 < Y_head_up; y+= 5, c++)
	{
		head_width[c] = abs(record[y][1]) -  abs(record[y][0]);
	}
	int max_dif = 0;
	int max_y = 0;
	for(int i = 1; i < head_width.size(); i++)
	{
		int a = abs(head_width[i] - head_width[i - 1]);
		if(a > max_dif)
		{
			max_dif = a;
			max_y = Y_head_down + 5 * i + 2;
		}
	}
	Px_Head.y = max_y;
	Px_Head.x = ( abs(record[max_y][1]) + abs(record[max_y][0])) * 0.5;

	Concurrency::parallel_for(0, (int)w * h, [&](int k){
		img_label[k * 4 + 3] = 255;
		if(img[k] != 0) 
		{
			img_label[k * 4 + 0] = 50;
			img_label[k * 4 + 1] = 50;
			img_label[k * 4 + 2] = 50;
		}
	});

	drawLabel(img_label, JointA, 2);
	drawLabel(img_label, JointB, 2);
	drawLabel(img_label, JointCS, 2);
	drawLabel(img_label, JointCL, 2);
	drawLabel(img_label, Px_Head, 2);
	drawLabel(img_label, Px_Torso, 2);

	if(if_front)
		PNG_IO::depth_array_to_PNG(img_label, "label_F.png", w, h);
	else
		PNG_IO::depth_array_to_PNG(img_label, "label_B.png", w, h);
	SAFE_DELETE_ARRAY(img_label);
}

Color4b inline CompareColor( Color4b c1)
{
	int thr = 100;
	if(c1.X() > thr  && c1.Y() < thr * 0.5 && c1.Z() < thr * 0.5)
		return Color4b::Red;
	if(c1.X() < thr && c1.Y() > thr && c1.Z() < thr )
		return Color4b::Green;
	if(c1.X() > thr && c1.Y() > thr && c1.Z() < thr)
		return Color4b::Yellow;
	return Color4b::White;
}
void JointEstimate::init(vector<CMeshO*>* mesh, int argc, char** argv, bool label_head)
{
	_cm = mesh;
	if(label_head)
	{
		for(int k = 0 ; k < _cm->size() ; k++)
		{
			//1. find bounding box
			vcg::tri::UpdateBounding<CMeshO>::Box(*_cm->at(k));
			vcg::tri::UpdateFlags<CMeshO>::VertexClearS(*_cm->at(k));
			vcg::tri::UpdateFlags<CMeshO>::VertexClearB(*_cm->at(k));
			//2. travesal the y_max to y_max - 100 vertex
			Concurrency::parallel_for(0u, (UINT)_cm->at(k)->vert.size(), [&](UINT i){
				if(_cm->at(k)->vert[i].P().Y() < _cm->at(k)->bbox.max.Y() - 120)
				{
					return;
				}
				else
				{
					if(CompareColor(_cm->at(k)->vert[i].C()).Equal((Color4b) Color4b::Red))
					{
						_cm->at(k)->vert[i].SetB(); //Red
					}
					else if(CompareColor(_cm->at(k)->vert[i].C()).Equal((Color4b) Color4b::Green))
					{
						_cm->at(k)->vert[i].SetS(); // Green or yellow 
					}
				}
			});

		}
	}
	for(int k = 0 ; k < _cm->size() ; k++)
	{
		vcg::tri::UpdateColor<CMeshO>::PerVertexConstant(*_cm->at(k), Color4b::White);
		if(label_head)
		{
			Concurrency::parallel_for(0u, (UINT)_cm->at(k)->vert.size(), [&](UINT i){
				if(!_cm->at(k)->vert[i].IsB() && !_cm->at(k)->vert[i].IsS())
					return;
				else if(_cm->at(k)->vert[i].IsB())
				{
					_cm->at(k)->vert[i].C() = Color4b::Red;
				}
				else if(_cm->at(k)->vert[i].IsS())
				{
					_cm->at(k)->vert[i].C() = Color4b::Green;
				}
			});
		}
		vcg::tri::UpdateFlags<CMeshO>::VertexClearB(*_cm->at(k));
		vcg::tri::UpdateFlags<CMeshO>::VertexClearV(*_cm->at(k));
	}

	glconfig = new glConfig();
	glconfig->init(argc, argv);
	for(int i = 0 ; i < 6 ; i++)
	{
		Joint[i] = Point3f(FLT_MIN,FLT_MIN,FLT_MIN);
	}
}

void JointEstimate::estimateJ(CMeshO& m, bool is_front, Point3f& P_Torso, Point3f& P_Head, Point3f& P_LeftArm, Point3f& P_RightArm, Point3f& P_LeftLeg, Point3f& P_RightLeg)
{
	float* img_depth = new float[glconfig->width * glconfig->height];
	float* img_mask = new float[glconfig->width * glconfig->height];
	memset(img_depth, 0, glconfig->width * glconfig->height * sizeof(float));
	memset(img_mask, 0, glconfig->width * glconfig->height * sizeof(float));
	Model* mod = new Model;
	mod->init();
	mod->fillModel(m);
	glconfig->getDepth(mod, img_depth, is_front);
	Concurrency::parallel_for(0, (int) glconfig->width * glconfig->height, [&](int k){
		if( img_depth[k] == 0) return;
		img_mask[k] = 1;
	});
	cpusmooth(img_mask);

	//analysis joint from silhottue
	int2 Pix_Joint[6];
	analysisJ(img_mask,is_front, Pix_Joint[0], Pix_Joint[1], Pix_Joint[2],Pix_Joint[3], Pix_Joint[4], Pix_Joint[5]);

	//back project joint pix to space
	float4 KI = glconfig->Kinv;
	Point3f P_Joint[6];
	for(int i = 0; i < 6; i++)
	{
		P_Joint[i].Z() = img_depth[Pix_Joint[i].x + Pix_Joint[i].y * glconfig->width];
		float fj_x = ((is_front) ? (float)(glconfig->width - Pix_Joint[i].x ) : ((float)Pix_Joint[i].x ))/ (float) glconfig->width;
		float fj_y = (float)Pix_Joint[i].y / (float) glconfig->height;
		P_Joint[i].X() = (KI.x * fj_x + KI.z) * P_Joint[i].Z();
		P_Joint[i].Y() = (KI.y * fj_y + KI.w) * P_Joint[i].Z();
		if(is_front)
			P_Joint[i].Z() = P_Joint[i].Z() - 2000.;
		else
			P_Joint[i].Z() = 2000. - P_Joint[i].Z();
		P_Joint[i].Y() += 1000.;
	}
	P_Torso = P_Joint[0];
	P_Head = P_Joint[1];
	P_LeftArm = P_Joint[2];
	P_RightArm = P_Joint[3];
	P_LeftLeg = P_Joint[4];
	P_RightLeg = P_Joint[5];


	SAFE_DELETE_ARRAY(img_mask);
	SAFE_DELETE_ARRAY(img_depth);
	SAFE_DELETE(mod);
	return;
}

void JointEstimate::colorJ(CMeshO& m, const float& radius,
						   const Point3f& P_Torso, const Point3f& P_Head, const Point3f& P_LeftArm, const Point3f& P_RightArm, const Point3f& P_LeftLeg, const Point3f& P_RightLeg,
						   const Color4b& C_Torso, const Color4b& C_Head, const Color4b& C_LeftArm, const Color4b& C_RightArm, const Color4b& C_LeftLeg, const Color4b& C_RightLeg)
{
	Concurrency::parallel_for(0, (int) m.vert.size(), [&](int k){
		Point3f currP = m.vert[k].P();
		if((currP - P_Torso).Norm() <= radius)
		{
			m.vert[k].C() = C_Torso;
			return;
		}
		if((currP - P_Head).Norm() <= radius)
		{
			m.vert[k].C() = C_Head;
			return;
		}
		if((currP - P_LeftArm).Norm() <= radius)
		{
			m.vert[k].C() = C_LeftArm;
			return;
		}
		if((currP - P_RightArm).Norm() <= radius)
		{
			m.vert[k].C() = C_RightArm;
			return;
		}
		if((currP - P_LeftLeg).Norm() <= radius)
		{
			m.vert[k].C() = C_LeftLeg;
			return;
		}
		if((currP - P_RightLeg).Norm() <= radius)
		{
			m.vert[k].C() = C_RightLeg;
			return;
		}
	});
}

void JointEstimate::estimateJoints(const Color4b& C_Torso, const Color4b& C_Head, const Color4b& C_LeftArm, const Color4b& C_RightArm, const Color4b& C_LeftLeg, const Color4b& C_RightLeg)
{
	Point3f FP_Torso, FP_Head, FP_LeftArm, FP_RightArm, FP_LeftLeg, FP_RightLeg;
	estimateJ(	*_cm->at(0), true,
		FP_Torso, FP_Head, FP_LeftArm, FP_RightArm, FP_LeftLeg, FP_RightLeg);

	colorJ(	*_cm->at(0), 20, 
		FP_Torso, FP_Head, FP_LeftArm, FP_RightArm, FP_LeftLeg, FP_RightLeg,
		C_Torso, C_Head, C_LeftArm, C_RightArm, C_LeftLeg, C_RightLeg);
	Log::LogMesh_CMeshO(_cm->at(0), "Joint_F.ply", true, true);

	Point3f BP_Torso, BP_Head, BP_LeftArm, BP_RightArm, BP_LeftLeg, BP_RightLeg;
	estimateJ(	*_cm->at(4), false,
		BP_Torso, BP_Head, BP_LeftArm, BP_RightArm, BP_LeftLeg, BP_RightLeg);

	colorJ(	*_cm->at(4), 20, 
		BP_Torso, BP_Head, BP_LeftArm, BP_RightArm, BP_LeftLeg, BP_RightLeg,
		C_Torso, C_Head, C_LeftArm, C_RightArm, C_LeftLeg, C_RightLeg);
	Log::LogMesh_CMeshO(_cm->at(3), "Joint_B.ply", true, true);

	Joint[0] = (FP_Torso + BP_Torso) / 2.;
	Joint[1] = (FP_Head + BP_Head) / 2.;
	Joint[2] = (FP_LeftArm + BP_LeftArm) / 2.;
	Joint[3] = (FP_RightArm + BP_RightArm) / 2.;
	Joint[4] = (FP_LeftLeg + BP_LeftLeg) / 2.;
	Joint[5] = (FP_RightLeg + BP_RightLeg) / 2.;
}

void JointEstimate::estimateHands(const Color4b& C_LeftHand, const Color4b& C_RightHand)
{
	assert(Joint[0].X() > FLT_MIN);

	for(int k = 0 ; k < NUM_FACE ; k++)
	{
		Point3f Max_L(FLT_MAX, FLT_MAX, FLT_MAX), Max_R(FLT_MIN, FLT_MIN, FLT_MIN);
		vcg::tri::UpdateBounding<CMeshO>::Box(*_cm->at(k));

		for(int i = 0 ; i < (*_cm).at(k)->vert.size() ; i+= 10)
		{
			Point3f currP =  _cm->at(k)->vert[i].P();
			if(Max_L.X() > currP.X())
				Max_L = currP;
			if(Max_R.X() < currP.X())
				Max_R = currP;
		}
		Point3f center = _cm->at(k)->bbox.Center();
		if(Max_R.X() < center.X() + 200 ||Max_R.X() <  + 200|| Max_R.Y() < center.Y() - 200) 
			Max_R.X() = FLT_MAX;
		if(Max_L.X() > center.X() - 200 ||Max_L.X() >  - 200 ||Max_L.Y() < center.Y() - 200) 
			Max_L.X() = FLT_MIN;
		for(int i = 0 ; i < (*_cm).at(k)->vert.size() ; i++)
		{
			Point3f currP =  _cm->at(k)->vert[i].P();
			if(currP.Y() < center.Y() - 100)
				continue;
			float NormR = Norm(currP - Max_R);
			if(NormR < 100 && k != 2)
				_cm->at(k)->vert[i].C() = C_RightHand;
			float NormL = Norm(currP - Max_L);
			if(NormL < 100 && k != 6)
				_cm->at(k)->vert[i].C() = C_LeftHand;
		}
	}
}

void JointEstimate::estimateFeet(const Color4b& C_LeftFoot, const Color4b& C_RightFoot)
{
	assert(Joint[4].Y() > FLT_MIN);
	assert(Joint[5].Y() > FLT_MIN);
	for(int k = 0 ; k < NUM_FACE ; k++)
	{
		float MinY_L = FLT_MAX;
		float MinY_R = FLT_MAX;
		int index_L = 0;
		int index_R = 0;
		Point3f LegMid = Joint[0];

		for(int i = 0 ; i < _cm->at(k)->vert.size() ; i += 10)
		{
			Point3f currP =  _cm->at(k)->vert[i].P();
			if(currP.Y() > LegMid.Y()) continue;
			MinY_L = (currP.X() < LegMid.X() - 100 && currP.Y() < MinY_L ) ? currP.Y(): MinY_L;
			MinY_R = (currP.X() > LegMid.X() + 100 && currP.Y() < MinY_R ) ? currP.Y(): MinY_R;
		}

		for(int i = 0 ; i < _cm->at(k)->vert.size() ; i++)
		{

			Point3f currP = _cm->at(k)->vert[i].P();
			if(currP.Y() < MinY_L + 200 &&  currP.Y() > MinY_L + 150 && currP.X() < LegMid.X() + 100)
			{
				_cm->at(k)->vert[i].C() = C_LeftFoot;
			}
			if(currP.Y() < MinY_R + 200 && currP.Y() > MinY_R + 150 && currP.X() > LegMid.X() + 100)
			{
				_cm->at(k)->vert[i].C() = C_RightFoot;
			}
		}
	}
}