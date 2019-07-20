#pragma once
#include "Model.h"
#include "..\Base.h"
#include "..\Helper.h"
#include <vector>
#include "..\IMAGE_DISPLAY\png_io.h"
#include "..\StdAfx.h"
#include "..\Nanoknn.h"

class JointEstimate2{
public:
	enum JointsName
	{
		head,
		torso,
		low_abodmen,
		Left_Up_Arm,
		Left_Low_Arm,
		Left_Hand,
		Right_Up_Arm,
		Right_Low_Arm,
		Right_Hand,
		Left_Thigh,
		Left_Calf,
		Left_Feet,
		Right_Thigh,
		Right_Calf,
		Right_Feet
	};
	JointEstimate2()
	{
		int n = Right_Feet + 1;
		vec_PC.resize(n);
		vec_PC[head] = pair<int, int>(torso, head);
		vec_PC[torso] = pair<int, int>(torso, head);
		vec_PC[low_abodmen] = pair<int, int>(head, torso);
		vec_PC[Left_Up_Arm] =  pair<int, int>(torso, Left_Low_Arm);
		vec_PC[Left_Low_Arm] = pair<int, int>(Left_Up_Arm, Left_Hand);
		vec_PC[Left_Hand] = pair<int, int>(Left_Low_Arm, Left_Hand);
		vec_PC[Right_Up_Arm] = pair<int, int>(torso, Right_Low_Arm);
		vec_PC[Right_Low_Arm] = pair<int, int>(Right_Up_Arm, Right_Hand);
		vec_PC[Right_Hand] = pair<int, int>(Right_Low_Arm, Right_Hand);
		vec_PC[Left_Thigh] = pair<int, int>(torso, Left_Calf);
		vec_PC[Left_Calf] = pair<int, int>(Left_Thigh, Left_Calf);
		vec_PC[Left_Feet] = pair<int, int>(Left_Calf, Left_Feet);
		vec_PC[Right_Thigh] = pair<int, int>(torso, Right_Calf);
		vec_PC[Right_Calf] = pair<int, int>(Right_Thigh, Right_Feet);
		vec_PC[Right_Feet] = pair<int, int>(Right_Calf, Right_Feet);
	}

	Color4b Label_to_Color(int Label);
	void init(vector<CMeshO*>* mesh, CMeshO& JointFace_Front, CMeshO& JointFace_Back);
	void SortJoint(vector<Point3f>& JointBuff);
	void AutoSegment_Simple();
	void AutoSegment_Complete();
	void LogCurrentStatus(string File)
	{
		for(int k = 0; k < _cm->size(); k++)
		{
			string sn = to_string(k);
			Log::LogMesh_CMeshO(_cm->at(k), string(File + "LogMark_" + sn + ".ply").c_str(),true, true, true);
		}
	}

	vector<Point3f> vec_JointP3d;
	vector<CMeshO*>* _cm;
	vector<pair<int, int>> vec_PC; //idx = curr node, first = parent node, second = child node
};

void JointEstimate2::AutoSegment_Simple()
{
	Point3f Normal_Head = vec_JointP3d[this->vec_PC[head].second] -  vec_JointP3d[this->vec_PC[head].first];
	Point3f Normal_Left_Arm_Up =  (vec_JointP3d[this->vec_PC[Left_Up_Arm].second] -  vec_JointP3d[Left_Up_Arm]).Normalize() * 0.95
		- (vec_JointP3d[this->vec_PC[Left_Up_Arm].first]  -  vec_JointP3d[Left_Up_Arm]).Normalize();
	Point3f Normal_Right_Arm_Up =  (vec_JointP3d[this->vec_PC[Right_Up_Arm].second] -  vec_JointP3d[Right_Up_Arm]).Normalize() * 0.95
		- (vec_JointP3d[this->vec_PC[Right_Up_Arm].first]  -  vec_JointP3d[Right_Up_Arm]).Normalize();
	Point3f Normal_Left_Leg_Calf = vec_JointP3d[this->vec_PC[Left_Calf].second] - vec_JointP3d[this->vec_PC[Left_Calf].first];
	Point3f Normal_Right_Leg_Calf = vec_JointP3d[this->vec_PC[Right_Calf].second] - vec_JointP3d[this->vec_PC[Right_Calf].first];

	Point3f Joint_Head = this->vec_JointP3d[head] + Point3f(0,15,0);
	Point3f Joint_Left_Arm_Up = this->vec_JointP3d[Left_Up_Arm];
	Point3f Joint_Right_Arm_Up = this->vec_JointP3d[Right_Up_Arm];
	Point3f Joint_Left_Leg_Calf = this->vec_JointP3d[Left_Calf]  + Point3f(0,15,0);
	Point3f Joint_Right_Leg_Calf = this->vec_JointP3d[Right_Calf];

	Normal_Head += Point3f(-0.05, 0,-1) * 250;
	Normal_Head.Normalize();

	Normal_Left_Arm_Up.Normalize();
	Normal_Right_Arm_Up.Normalize();
	Normal_Left_Leg_Calf.Normalize();
	Normal_Right_Leg_Calf.Normalize();

	Point4f Plane_Head;
	Point4f Plane_Left_Arm_Up;
	Point4f Plane_Right_Arm_Up;
	Point4f Plane_Left_Leg_Calf;
	Point4f Plane_Right_Leg_Calf;

	float d = 0;
	d = -Normal_Head.dot(Joint_Head);
	Plane_Head = Point4f(Normal_Head.X(), Normal_Head.Y(), Normal_Head.Z(), d);

	d = -Normal_Left_Arm_Up.dot(Joint_Left_Arm_Up);
	Plane_Left_Arm_Up = Point4f(Normal_Left_Arm_Up.X(), Normal_Left_Arm_Up.Y(), Normal_Left_Arm_Up.Z(), d);

	d = -Normal_Right_Arm_Up.dot(Joint_Right_Arm_Up);
	Plane_Right_Arm_Up = Point4f(Normal_Right_Arm_Up.X(), Normal_Right_Arm_Up.Y(), Normal_Right_Arm_Up.Z(), d);

	d = -Normal_Left_Leg_Calf.dot(Joint_Left_Leg_Calf);
	Plane_Left_Leg_Calf = Point4f(Normal_Left_Leg_Calf.X(), Normal_Left_Leg_Calf.Y(), Normal_Left_Leg_Calf.Z(), d);

	d = -Normal_Right_Leg_Calf.dot(Joint_Right_Leg_Calf);
	Plane_Right_Leg_Calf = Point4f(Normal_Right_Leg_Calf.X(), Normal_Right_Leg_Calf.Y(), Normal_Right_Leg_Calf.Z(), d);

	for(int k = 0 ; k < this->_cm->size(); k++)
	{
		for(auto& v : this->_cm->at(k)->vert)
		{
			float p1 = Plane_Head.X() * v.P().X() + Plane_Head.Y() * v.P().Y() + Plane_Head.Z() * v.P().Z() + Plane_Head.W();
			float p2 = Plane_Left_Arm_Up.X() * v.P().X() + Plane_Left_Arm_Up.Y() * v.P().Y() + Plane_Left_Arm_Up.Z() * v.P().Z() + Plane_Left_Arm_Up.W();
			float p3 = Plane_Right_Arm_Up.X() * v.P().X() + Plane_Right_Arm_Up.Y() * v.P().Y() + Plane_Right_Arm_Up.Z() * v.P().Z() + Plane_Right_Arm_Up.W();
			float p4 = Plane_Left_Leg_Calf.X() * v.P().X() + Plane_Left_Leg_Calf.Y() * v.P().Y() + Plane_Left_Leg_Calf.Z() * v.P().Z() + Plane_Left_Leg_Calf.W();
			float p5 = Plane_Right_Leg_Calf.X() * v.P().X() + Plane_Right_Leg_Calf.Y() * v.P().Y() + Plane_Right_Leg_Calf.Z() * v.P().Z() + Plane_Right_Leg_Calf.W();
			
			if(p1 > 0)
			{
				v.C() = Label_to_Color(head);
				continue;
			}
			else if(p4 > 0 && v.P().X() < vec_JointP3d[torso].X())
			{
				v.C() = Label_to_Color(Left_Calf);
				continue;
			}
			else if(p5 > 0 && v.P().X() > vec_JointP3d[torso].X())
			{
				v.C() = Label_to_Color(Right_Calf);
				continue;
			}
			else if(p2 > 0)
			{
				v.C() = Label_to_Color(Left_Up_Arm);
				continue;
			}
			else if(p3 > 0)
			{
				v.C() = Label_to_Color(Right_Up_Arm);
				continue;
			}
			else
			{
				v.C() = Label_to_Color(torso);
				continue;
			}
		}
	}
}

void JointEstimate2::AutoSegment_Complete()
{
	AutoSegment_Simple();
	Point3f N_arm_left = vec_JointP3d[Left_Hand] - vec_JointP3d[Left_Up_Arm];
	Point3f N_arm_right = vec_JointP3d[Right_Hand] - vec_JointP3d[Right_Up_Arm];
	Point3f N_leg_left = vec_JointP3d[Left_Feet] - vec_JointP3d[Left_Thigh];
	Point3f N_leg_right = vec_JointP3d[Right_Feet] - vec_JointP3d[Right_Thigh];
	Point3f N_low_abodmen = Point3f(0,-1,0);
	N_arm_left.Normalize();
	float d_al_1 = -N_arm_left.dot(vec_JointP3d[Left_Low_Arm]);
	float d_al_2 = -N_arm_left.dot(vec_JointP3d[Left_Hand]);
	N_arm_right.Normalize();
	float d_ar_1 = -N_arm_right.dot(vec_JointP3d[Right_Low_Arm]);
	float d_ar_2 = -N_arm_right.dot(vec_JointP3d[Right_Hand]);
	N_leg_left.Normalize();
	float d_ll = -N_leg_left.dot(vec_JointP3d[Left_Feet]);
	float d_tl = -N_leg_left.dot(vec_JointP3d[Left_Thigh]);
	N_leg_right.Normalize();
	float d_lr = -N_leg_right.dot(vec_JointP3d[Right_Feet]);
	float d_tr = -N_leg_right.dot(vec_JointP3d[Right_Thigh]);
	N_low_abodmen.Normalize();
	float d_la = -N_low_abodmen.dot(vec_JointP3d[torso]);

	for(int k = 0 ; k < this->_cm->size(); k++)
	{
		for(auto& v : this->_cm->at(k)->vert)
		{
			//head
			if(v.C().Equal(Label_to_Color(head)))
			{
				continue;
			}
			//arm left -> up, low, hand
			else if(v.C().Equal(Label_to_Color(Left_Up_Arm)))
			{
				float dis1 = N_arm_left.dot(v.P()) + d_al_1;
				float dis2 = N_arm_left.dot(v.P()) + d_al_2;
				if(dis2 > 0)
				{
					v.C() = Label_to_Color(Left_Hand);
					continue;
				}
				if(dis1 > 0)
				{
					v.C() = Label_to_Color(Left_Low_Arm);
					continue;
				}
			}
			//arm right -> up, low, hand
			else if(v.C().Equal(Label_to_Color(Right_Up_Arm)))
			{
				float dis1 = N_arm_right.dot(v.P()) + d_ar_1;
				float dis2 = N_arm_right.dot(v.P()) + d_ar_2;
				if(dis2 > 0)
				{
					v.C() = Label_to_Color(Right_Hand);
					continue;
				}
				if(dis1 > 0)
				{
					v.C() = Label_to_Color(Right_Low_Arm);
					continue;
				}

			}
			//calf left -> calf, feet
			else if(v.C().Equal(Label_to_Color(Left_Calf)))
			{
				float dis = N_leg_left.dot(v.P()) + d_ll;
				if(dis > 0)
					v.C() = Label_to_Color(Left_Feet);
			}
			//calf right -> calf, feet
			else if(v.C().Equal(Label_to_Color(Right_Calf)))
			{
				float dis = N_leg_right.dot(v.P()) + d_lr;
				if(dis > 0)
					v.C() = Label_to_Color(Right_Feet);
			}
			//torso -> torso, low abdomen, thigh L, thigh R
			else
			{
				float dis_tl = N_leg_left.dot(v.P()) + d_tl;
				float dis_tr = N_leg_right.dot(v.P()) + d_tr;
				float dis_la = N_low_abodmen.dot(v.P()) + d_la;
				if(dis_la < 0)
				{
					v.C() = Label_to_Color(torso);
					continue;
				}
				else if(dis_tr < 0 && dis_tl < 0)
				{
					v.C() = Label_to_Color(low_abodmen);
					continue;
				}
				else if(v.P().X() > vec_JointP3d[low_abodmen].X())
				{
					v.C() = Label_to_Color(Right_Thigh);
					continue;
				}
				else
				{
					v.C() = Label_to_Color(Left_Thigh);
				}
				
			}
		}
	}
}

Color4b JointEstimate2::Label_to_Color(int Label)
{
	//http://htmlcolorcodes.com/
	Color4b Color;
	switch (Label)
	{
	case torso:
		Color = Color4b(192, 57, 43, 255);
		break;
	case low_abodmen:
		Color = Color4b(236, 240, 129, 255);
		break;
	case head:
		Color = Color4b(181, 173, 210, 255);
		break;
	case Left_Up_Arm:
		Color = Color4b(235, 68, 36, 255);
		break;
	case Left_Low_Arm:
		Color = Color4b(170, 234, 137, 255);
		break;
	case Left_Hand:
		Color = Color4b(244, 129, 36, 255);
		break;
	case Right_Up_Arm:
		Color = Color4b(247, 166, 25, 255);
		break;
	case Right_Low_Arm:
		Color = Color4b(253, 240, 0, 255);
		break;
	case Right_Hand:
		Color = Color4b(111, 190, 65, 255);
		break;
	case Left_Thigh:
		Color = Color4b(0, 164, 193, 255);
		break;
	case Left_Calf:
		Color = Color4b(85, 195, 196, 255);
		break;
	case Left_Feet:
		Color = Color4b(2, 102, 177, 255);
		break;
	case Right_Thigh:
		Color = Color4b(30, 64, 153, 255);
		break;
	case Right_Calf:
		Color = Color4b(85, 44, 140, 255);
		break;
	case Right_Feet:
		Color = Color4b(162, 35, 142, 255);

	default:
		break;
	}
	return Color;
}
//Color4b JointEstimate2::Label_to_Color(int Label)
//{
//	Color4b Color;
//	switch (Label)
//	{
//	case torso:
//		Color = Color4b::Red;
//		break;
//	case low_abodmen:
//		Color = Color4b::DarkRed;
//		break;
//	case head:
//		Color = Color4b:: Yellow;
//		break;
//	case Left_Up_Arm:
//		Color = Color4b::LightBlue;
//		break;
//	case Left_Low_Arm:
//		Color = Color4b::Blue;
//		break;
//	case Left_Hand:
//		Color = Color4b::DarkDarkBlue;
//		break;
//	case Right_Up_Arm:
//		Color = Color4b::LightGreen;
//		break;
//	case Right_Low_Arm:
//		Color = Color4b::Green;
//		break;
//	case Right_Hand:
//		Color = Color4b::DarkGreen;
//		break;
//	case Left_Thigh:
//		Color = Color4b::LightPurple;
//		break;
//	case Left_Calf:
//		Color = Color4b::Magenta;
//		break;
//	case Left_Feet:
//		Color = Color4b::DarkDarkPurple;
//		break;
//	case Right_Thigh:
//		Color = Color4b::LightGray;
//		break;
//	case Right_Calf:
//		Color = Color4b::Gray;
//		break;
//	case Right_Feet:
//		Color = Color4b::DarkGray;
//		break;
//
//	default:
//		break;
//	}
//	return Color;
//}

void JointEstimate2::init(vector<CMeshO*>* mesh, CMeshO& JointFace_Front, CMeshO& JointFace_Back)
{
	_cm = mesh;
	//init 8 meshes
	for(int k = 0 ; k < _cm->size() ; k++)
	{
		vcg::tri::UpdateColor<CMeshO>::PerVertexConstant(*_cm->at(k), Color4b::Gray150);
		vcg::tri::UpdateFlags<CMeshO>::VertexClearB(*_cm->at(k));
		vcg::tri::UpdateFlags<CMeshO>::VertexClearV(*_cm->at(k));
	}

	//Copy Joint Marker
	for(auto& v : JointFace_Front.vert)
	{
		if(v.C().Equal((Color4b)(Color4b::Red)))
		{
			_cm->at(0)->vert.at(&v - JointFace_Front.vert.data()).C() = Color4b::Red;
		}
	}

	for(auto& v : JointFace_Back.vert)
	{
		if(v.C().Equal((Color4b)(Color4b::Red)))
		{
			_cm->at(4)->vert.at(&v - JointFace_Back.vert.data()).C() = Color4b::Red;
		}
	}

	//Redefine Front & Back
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(JointFace_Front, *_cm->at(0));
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(JointFace_Back, *_cm->at(4));

	//Calculate Joint position
	Nanoknn KnnFront(JointFace_Front);
	Nanoknn KnnBack(JointFace_Back);
	KnnFront.buildNano();
	KnnBack.buildNano();
	vector<Point3f> JointPosition_Front;
	vector<int> JointMarkerIdx_Front;
	JointMarkerIdx_Front.reserve(1000);

	for(auto& v : JointFace_Front.vert)
	{
		if(v.C().Equal((Color4b)(Color4b::Red)))
		{
			JointMarkerIdx_Front.push_back(&v - JointFace_Front.vert.data());
		}
	}
	for(int i = 0; i < JointMarkerIdx_Front.size(); i++)
	{
		int idx = JointMarkerIdx_Front[i];
		if(JointFace_Front.vert[idx].C().Equal( (Color4b) Color4b::Red))
		{
			Point3f SumJoint(0,0,0);
			int Count = 0;
			vector<int> knnidx;
			int knn = 600;
			KnnFront.findKNN(JointFace_Front.vert[idx], knn, knnidx);
			for(int k = 0; k < knn; k++)
			{
				int curr_idx = knnidx[k];
				if(JointFace_Front.vert[curr_idx].C().Equal((Color4b) Color4b::Red))
				{
					Count++;
					SumJoint += JointFace_Front.vert[curr_idx].P();
					JointFace_Front.vert[curr_idx].C() = Color4b::Gray150;
				}
			}
			JointPosition_Front.push_back(SumJoint / Count);
		}
	}

	vector<Point3f> JointPosition_Back;
	vector<int> JointMarkerIdx_Back;
	JointMarkerIdx_Back.reserve(1000);

	for(auto& v : JointFace_Back.vert)
	{
		if(v.C().Equal((Color4b)(Color4b::Red)))
		{
			JointMarkerIdx_Back.push_back(&v - JointFace_Back.vert.data());
		}
	}

	for(int i = 0; i < JointMarkerIdx_Back.size(); i++)
	{
		int idx = JointMarkerIdx_Back[i];
		if(JointFace_Back.vert[idx].C().Equal( (Color4b) Color4b::Red))
		{
			Point3f SumJoint(0,0,0);
			int Count = 0;
			vector<int> knnidx;
			int knn = 600;
			KnnBack.findKNN(JointFace_Back.vert[idx], knn, knnidx);
			for(int k = 0; k < knn; k++)
			{
				int curr_idx = knnidx[k];
				if(JointFace_Back.vert[curr_idx].C().Equal((Color4b) Color4b::Red))
				{
					Count++;
					SumJoint += JointFace_Back.vert[curr_idx].P();
					JointFace_Back.vert[curr_idx].C() = Color4b::Gray150;
				}
			}
			JointPosition_Back.push_back(SumJoint / Count);
		}
	}



	assert(JointPosition_Back.size() == JointPosition_Front.size());
	SortJoint(JointPosition_Back);
	SortJoint(JointPosition_Front);

	for(int k = 0 ; k < JointPosition_Front.size(); k++)
	{
		int knn = 70;
		vector<int> knnidx;
		CVertexO currV;
		currV.P() = JointPosition_Front[k];
		currV.N() = Point3f(0,0,-1);
		KnnFront.findKNN(currV, knn, knnidx);
		for(int i = 0; i < knn; i++)
		{
			int curridx = knnidx[i];
			JointFace_Front.vert[curridx].C() = Color4b::Blue;//Label_to_Color(k);
		}
	}
	Log::LogMesh_CMeshO(&JointFace_Front, "Data//DATA TEST6//JointFace_Front_Map.ply", true, true, true);

	for(int k = 0 ; k < JointPosition_Back.size(); k++)
	{
		int knn = 70;
		vector<int> knnidx;
		CVertexO currV;
		currV.P() = JointPosition_Back[k];
		currV.N() = Point3f(0,0,-1);
		KnnBack.findKNN(currV, knn, knnidx);
		for(int i = 0; i < knn; i++)
		{
			int curridx = knnidx[i];
			JointFace_Back.vert[curridx].C() = Color4b::Blue;//Label_to_Color(k);
		}
	}
	Log::LogMesh_CMeshO(&JointFace_Back, "Data//DATA TEST6//JointFace_Back_Map.ply", true, true, true);



	vec_JointP3d.resize(JointPosition_Front.size());
	for(int i = 0; i < vec_JointP3d.size(); i++)
	{
		vec_JointP3d[i] = JointPosition_Back[i] * .5 + JointPosition_Front[i] * .5;
	}
}

void JointEstimate2::SortJoint(vector<Point3f>& JointBuff)
{
	assert(JointBuff.size() == this->Right_Feet);
	std::sort(JointBuff.begin(), JointBuff.end(), [](const Point3f& a, const Point3f& b) -> bool{ return a.Y() > b.Y();});
	Point3f Pivot_Front = JointBuff[0];
	for(int i =0 ; i < JointBuff.size(); i++)
	{
		JointBuff[i] -= Pivot_Front;
	}
	if(JointBuff[1].X() > 0)
		std::swap(JointBuff[1], JointBuff[2]);
	if(JointBuff[3].X()  > 0)
		std::swap(JointBuff[3], JointBuff[4]);
	if(JointBuff[10].X()  > 0)
		std::swap(JointBuff[10], JointBuff[11]);
	if(JointBuff[12].X()  > 0)
		std::swap(JointBuff[12], JointBuff[13]);
	std::sort(JointBuff.begin() + 5, JointBuff.begin() + 10, [](const Point3f& a, const Point3f& b)->bool{ return a.X() < b.X();});
	vector<Point3f> JointBuff_Temp(this->Right_Feet + 1);
	JointBuff_Temp[this->head] = JointBuff[0];
	JointBuff_Temp[this->torso] = JointBuff_Temp[this->low_abodmen] = JointBuff[7];
	JointBuff_Temp[this->Left_Up_Arm] = JointBuff[1];
	JointBuff_Temp[this->Left_Low_Arm] = JointBuff[3];
	JointBuff_Temp[this->Left_Hand] = JointBuff[5];
	JointBuff_Temp[this->Right_Up_Arm] = JointBuff[2];
	JointBuff_Temp[this->Right_Low_Arm] = JointBuff[4];
	JointBuff_Temp[this->Right_Hand] = JointBuff[9];
	JointBuff_Temp[this->Left_Thigh] = JointBuff[6];
	JointBuff_Temp[this->Left_Calf] = JointBuff[10];
	JointBuff_Temp[this->Left_Feet] = JointBuff[12];
	JointBuff_Temp[this->Right_Thigh] = JointBuff[8];
	JointBuff_Temp[this->Right_Calf] = JointBuff[11];
	JointBuff_Temp[this->Right_Feet] = JointBuff[13];
	JointBuff.resize(JointBuff_Temp.size());
	for(int k = 0; k < JointBuff.size(); k++)
	{
		JointBuff[k] = JointBuff_Temp[k] + Pivot_Front;
	}
}





