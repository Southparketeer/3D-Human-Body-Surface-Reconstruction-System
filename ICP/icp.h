#pragma once
#include "..\StdAfx.h"
#include "..\Base.h"
#include <map>
#include <vector>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/position.h>
#include <vcg/complex/algorithms/update/color.h>
#include<vcg/complex/algorithms/update/topology.h>
#include "AlignPair.h"
#include "OccupancyGrid.h"
#include "..\Helper.h"
#include <Eigen/Dense>
#include "meshtree.h"
#include "ArticulateGlobalAlign.h"
using namespace Eigen;
class ICP
{
public:
	ICP(){};
	~ICP(){};
	void icpAlignPair(vcg::CMeshO& source, vcg::CMeshO& target, int max_iter, float Min_Distance, float Sample_size, float thr_color);
};

class PoseInit
{
public:
	PoseInit()
	{
		_cmv.resize(NUM_FACE);
		memset(_cmv.data(), NULL, sizeof(CMeshO*) * NUM_FACE);
		mat.resize(NUM_FACE);
	}
	~PoseInit()
	{
		for(int i = 0 ; i < _cmv.size() ; i++ )
			SAFE_DELETE(_cmv[i]);
	}
	void init(string s);
	Mat4 PCAReferencePose(CMeshO * cm);
	void poseProcess();


public:
	vector<CMeshO *>_cmv;
	vector<Mat4, Eigen::aligned_allocator<Mat4>> mat;
};


///////////////////////////////////////
//Implementations
///////////////////////////////////////

void PoseInit::init(string s)
{
	Point3f MoveReference;
	for(int k = 0 ; k < NUM_FACE ; k++)
	{
		_cmv[k] = new CMeshO;
		Log::LoadMesh(string(s + to_string(k) + ".ply").c_str(), *_cmv[k]);
		int dup = tri::Clean<CMeshO>::RemoveDuplicateVertex(*_cmv[k]);
		int unref = tri::Clean<CMeshO>::RemoveUnreferencedVertex(*_cmv[k]);
		tri::UpdateNormal<CMeshO>::NormalizePerVertex(*_cmv[k]);
		tri::UpdateTopology<CMeshO>::FaceFace(*_cmv[k]);
		tri::Clean<CMeshO>::RemoveSmallConnectedComponentsSize(*_cmv[k], 1000);
		if(k == 0)
		{
			tri::UpdateBounding<CMeshO>::Box(*_cmv[k]);
			MoveReference = (_cmv[k]->bbox.max * 0.5 + _cmv[k]->bbox.min * 0.5) * -1;
			MoveReference.Y() = -_cmv[k]->bbox.min.Y();
		}
		tri::UpdatePosition<CMeshO>::Translate(*_cmv[k], MoveReference);
		printf("Removed %i duplicate and %i unreferenced vertices from mesh %i\n",dup,unref, k);
		printf( "Mesh has %i vert and %i faces\n", _cmv[k]->VN(), _cmv[k]->FN());
		mat[k].setIdentity();
	}
}

#undef max
void PoseInit::poseProcess()
{
	vector<CMeshO*> MeshSimplify(NUM_FACE);
	for(int k = 0 ; k < NUM_FACE ; k ++)
	{
		float angle = -(PI * 2.0 /(float) NUM_FACE ) * k;
		Matrix44f Tr;
		Tr.FromEulerAngles(0, angle, 0);
		Tr.ToEigenMatrix(mat[k]);

		MeshSimplify[k] = new CMeshO;
		CMeshO Temp_copy;
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(Temp_copy, *_cmv[k]);
		vcg::tri::Clean<CMeshO>::MergeCloseVertex(Temp_copy, 8);
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(*MeshSimplify[k], Temp_copy);
		vcg::tri::UpdatePosition<CMeshO>::Matrix(*MeshSimplify[k], Tr);
		MeshSimplify[k]->Tr.SetIdentity();
		vcg::tri::UpdateNormal<CMeshO>::PerVertex(*MeshSimplify[k]);
		vcg::tri::UpdateNormal<CMeshO>::NormalizePerVertex(*MeshSimplify[k]);
		vcg::tri::UpdateBounding<CMeshO>::Box(*MeshSimplify[k]);
		cout<<"Process "<<k<<endl;
	}

	vector<Box3f> lowbox(NUM_FACE);
	for(int k = 0 ; k < NUM_FACE ; k ++)
	{
		//calculate vertex bbox y = 0 - 500 mm;
		Point3f max(FLT_MIN, FLT_MIN, FLT_MIN);
		Point3f min(FLT_MAX, FLT_MAX, FLT_MAX);
		for(int i = 0 ; i < MeshSimplify[k]->vert.size() ; i ++)
		{
			Point3f curr =  MeshSimplify[k]->vert.at(i).P();
			{
				if(curr.Y() >= 200 && curr.Y() <= 600)
				{
					max.X() = curr.X() > max.X() ? curr.X() : max.X();
					max.Y() = curr.Y() > max.Y() ? curr.Y() : max.Y();
					max.Z() = curr.Z() > max.Z() ? curr.Z() : max.Z();
					min.X() = curr.X() < min.X() ? curr.X() : min.X();
					min.Y() = curr.Y() < min.Y() ? curr.Y() : min.Y();
					min.Z() = curr.Z() < min.Z() ? curr.Z() : min.Z();
				}
			}
		}
		lowbox[k].max = max;
		lowbox[k].min = min;
	}

	for(int k = 0 ; k < NUM_FACE ; k ++)
	{
		Box3f curr = lowbox[k];
		Box3f anchor = lowbox[0];
		Point3f move(0,0,0);

		if(k == 2)
		{
			Point3f anchor_center = (anchor.max + anchor.min) * 0.5;
			Point3f anchor_min = anchor.min;
			anchor_min.Y() = 0.;
			Point3f curr_min = curr.min;
			curr_min.Y() = 0.;
			move = anchor_min - curr_min + Point3f(0,0,0) - Point3f(anchor_center.X(), anchor.min.Y(), anchor_center.Z());
		}
		else if(k == 6)
		{
			Point3f anchor_center = (anchor.max + anchor.min) * 0.5;
			Point3f anchor_max = anchor.max;
			anchor_max.Y() = 0.;
			Point3f curr_max = curr.max;
			curr_max.Y() = 0.;
			move = anchor_max - curr_max + Point3f(0,0,0) - Point3f(anchor_center.X(), anchor.min.Y(), anchor_center.Z());
		}
		else
		{
			Point3f anchor_center = (anchor.max + anchor.min) * 0.5;
			anchor_center.Y() = 0.;
			Point3f curr_center = (curr.max + curr.min) * 0.5;
			curr_center.Y() = 0.;
			move = anchor_center - curr_center + Point3f(0,0,0) - Point3f(anchor_center.X(), anchor.min.Y(), anchor_center.Z());
		}

		vcg::tri::UpdatePosition<CMeshO>::Translate(*MeshSimplify.at(k), move);
		mat[k].block<3,1>(0,3) = Vet3(move.X(), move.Y(), move.Z());

		vcg::tri::UpdateBounding<CMeshO>::Box(*MeshSimplify.at(k));
		MeshSimplify[k]->Tr.SetIdentity();
		Log::LogMesh_CMeshO(MeshSimplify.at(k), string(to_string(k)+".ply").c_str());
	}

	//--------------------------CPU GLOBAL RIGID ALIGNMENT-----------------------------//
	vector<Mat4, Eigen::aligned_allocator<Mat4>> mat_ICP(NUM_FACE);
	for(int k = 0 ; k < NUM_FACE ; k++)
	{
		mat_ICP[k].setIdentity();
	}
	ArticulateGlobalAlign::Param param_AlginPose(10,2000,50,0.05,100);
	ArticulateGlobalAlign * GAlign_Pose = new ArticulateGlobalAlign(param_AlginPose);
	GAlign_Pose->init(MeshSimplify);
	GAlign_Pose->processGlobalAlignment(false);
	GAlign_Pose->getResult(mat_ICP,false);
	SAFE_DELETE(GAlign_Pose);

	for(int k = 1 ; k < NUM_FACE ; k++)
	{
		MeshSimplify[k]->Tr.SetIdentity();
		Matrix44f Trk;
		Trk.FromEigenMatrix(mat_ICP[0].inverse() * mat_ICP[k]);
		_cmv[k]->Tr.SetIdentity();
		vcg::tri::UpdatePosition<CMeshO>::Matrix(*MeshSimplify[k] , Trk);
		mat[k] = mat_ICP[0].inverse() * mat_ICP[k] * mat[k];
	}

	for(int k = 0 ; k < NUM_FACE; k++)
	{
		mat_ICP[k].setIdentity();
	}

	ArticulateGlobalAlign::Param param_AlginPose2(5,1000,150,0.20,100);
	ArticulateGlobalAlign * GAlign_Pose2 = new ArticulateGlobalAlign(param_AlginPose2);
	GAlign_Pose2->init(MeshSimplify);
	GAlign_Pose2->processGlobalAlignment(false);
	GAlign_Pose2->getResult(mat_ICP,false);
	SAFE_DELETE(GAlign_Pose2);

	for(int k = 1 ; k < NUM_FACE ; k++)
	{
		MeshSimplify[k]->Tr.SetIdentity();
		mat[k] = mat_ICP[0].inverse() * mat_ICP[k] * mat[k];
	}

	for(int k = 0 ; k <NUM_FACE; k++)
	{
		Matrix44f Trk;
		Trk.FromEigenMatrix(mat[k]);
		_cmv[k]->Tr.SetIdentity();
		vcg::tri::UpdatePosition<CMeshO>::Matrix(*_cmv[k] , Trk);
	}



	return;
}


Mat4 PoseInit::PCAReferencePose(CMeshO * cm)
{
	Eigen::MatrixXf A(cm->vert.size(), 3);
	Eigen::MatrixXf B(cm->vert.size(), 1);
	for(int i = 0 ; i < cm->vert.size(); i++)
	{
		A(i,0) = cm->vert.at(i).P().X();
		A(i,1) = cm->vert.at(i).P().Y();
		A(i,2) = 1;
		B(i,0) = cm->vert.at(i).P().Z();
	}
	Eigen::MatrixXf AtA = A.transpose() * A;
	Eigen::MatrixXf AtB = A.transpose() * B;
	LLT<Matrix3f> llt;
	llt.compute(AtA);
	Eigen::MatrixXf X = llt.solve(AtB);
	Vet3 n(X(0,0), X(1,0), -1);
	n.normalize();
	cout<<n;
	Vet3 xA = n;
	Vet3 xB(0,0,-1);
	Vet3 x = xA.cross(xB);
	float s = x.norm();
	float cc = xA.dot(xB);
	Mat3 mA;
	mA.setZero();
	mA(0,1) = -x.z();
	mA(0,2) =  x.y();
	mA(1,0) =  x.z();
	mA(1,2) = -x.x();
	mA(2,0) = -x.y();
	mA(2,1) =  x.x();
	Mat4 R;
	R.setIdentity();
	R.block<3,3>(0,0) += mA + mA * mA * (1 - cc) / s * s;
	vcg::tri::UpdateBounding<CMeshO>::Box(*cm);
	Point3f p_mid = cm->bbox.min * 0.5 + cm->bbox.max * 0.5;
	Vet3 M(-p_mid.X(), 0, -p_mid.Z());
	Mat4 RT;
	RT.setIdentity();
	RT.block<3,1>(0,3) = M;
	RT = R * RT;
	return RT;
}

void ICP::icpAlignPair(vcg::CMeshO& source, vcg::CMeshO& target, int max_iter, float Min_Distance = 50, float Sample_size = 500, float thr_color = 173)
{
	Matrix44f M, Tr_copy;
	M.SetIdentity();
	source.Tr.SetIdentity();
	target.Tr.SetIdentity();
	Tr_copy.Import(source.Tr);

	float area = 0;
	vcg::OccupancyGrid OG;

	tri::UpdatePosition<CMeshO>::Matrix(target,target.Tr);
	tri::UpdatePosition<CMeshO>::Matrix(source,source.Tr);
	tri::UpdateBounding<CMeshO>::Box(source);
	tri::UpdateBounding<CMeshO>::Box(target);
	source.Tr.Import(M); //if updated pos, than set Tr to Identity

	Box3f FullBBox;
	FullBBox.Add(target.bbox);
	FullBBox.Add(source.bbox);
	OG.Init(2, FullBBox, 5000);
	OG.AddMesh( target, M, 0); //target = 0
	OG.AddMesh( source, M, 1); //source = 1
	OG.Compute();
	OG.Dump(area);

	if(area < 0.2) return;

	//--------------- ICP alignment---------------//
	Matrix44f Tr_ICP; Tr_ICP.SetIdentity();
	bool stop = false;
	int k = 0;
	while(k < max_iter && stop == false)
	{ 
		k++;
		cout<<k<<"---------->"<<endl;
		vcg::AlignPair::Result result;
		//Param(int Sample_Num, int Max_Iter, float Min_Distance)
		vcg::AlignPair::Param ap(Sample_size,100,Min_Distance, thr_color);

		vcg::AlignPair::A2Mesh Fix;
		vcg::AlignPair aa;
		target.face.EnableMark();
		aa.ConvertMesh<CMeshO>(target, Fix);

		vcg::AlignPair::A2Grid UG;
		vcg::AlignPair::A2GridVert VG;

		if(target.fn==0 || ap.UseVertexOnly)
		{
			Fix.InitVert(vcg::Matrix44f::Identity());
			vcg::AlignPair::InitFixVert(&Fix,ap,VG);
		}
		else
		{
			Fix.Init(vcg::Matrix44f::Identity());
			vcg::AlignPair::InitFix(&Fix, ap, UG);
		}

		source.face.EnableMark();
		std::vector<vcg::AlignPair::A2Vertex> tmpmv;
		aa.ConvertVertex(source.vert,tmpmv);
		aa.SampleMovVert(tmpmv, ap.SampleNum, ap.SampleMode);

		aa.mov=&tmpmv;
		aa.fix=&Fix;
		aa.ap = ap;

		vcg::Matrix44f In;
		In.SetIdentity();

		aa.Align(In,UG,VG,result);
		result.FixName=0;
		result.MovName=1;

		result.as.Dump();
		float initialErr = result.as.FirstPc150();
		float lastErr = result.as.LastPcl50();
		if(lastErr - initialErr > 0.5)
		{
			stop = true; 
			continue;
		}
	/*	else if(fabs(lastErr - initialErr) < 0.0001)
		{
			stop = true;
		}*/
		tri::UpdatePosition<CMeshO>::Matrix(source, result.Tr);
		Tr_ICP = result.Tr * Tr_ICP;
	}
	Tr_ICP.print();
	Tr_copy = Tr_ICP * Tr_copy;
	
	Matrix4d e_tr_copy, e_tr_copy_inv;
	Matrix44f Tr_copy_inv; 
	Tr_copy.ToEigenMatrix(e_tr_copy);
	e_tr_copy_inv = e_tr_copy.inverse();
	Tr_copy_inv.FromEigenMatrix( e_tr_copy_inv);

	tri::UpdatePosition<CMeshO>::Matrix(source,  Tr_copy_inv);
	source.Tr.Import(Tr_copy); //update source tr
}