#pragma once
#include "meshtree.h"
#include "StdAfx.h"

class ArticulateGlobalAlign
{
public:
	class Param
	{
	public:
		int GlobalAlignIteration;
		int SampleNum;
		int MaxIterNum;
		double arcThreshold;
		double MinDistAbs;

		Param(int GlobalAlignIteration = 4, int SampleNum = 1000, int MaxIterNum = 50, double arcThreshold = 0.1, double MinDistAbs = 100)
		{
			this->GlobalAlignIteration = GlobalAlignIteration;
			this->SampleNum = SampleNum;
			this->MaxIterNum = MaxIterNum;
			this->arcThreshold = arcThreshold;
			this->MinDistAbs = MinDistAbs;
		}
	};

	ArticulateGlobalAlign(ArticulateGlobalAlign::Param& param);
	~ArticulateGlobalAlign();
	void init(vector<CMeshO*> mesh);
	void processGlobalAlignment(bool if_control = true);
	void NormalizeTr();
	void getResult(vector<Mat4, aligned_allocator<Mat4>>& vecMat, const vector<int>& mask, bool b_Normalize = true);
	void getResult(vector<Mat4, aligned_allocator<Mat4>>& vecMat, bool b_Normalize = true);
	void setPose(vector<Mat4, aligned_allocator<Mat4>>& vecMat);
	void updatePose(vector<Mat4, aligned_allocator<Mat4>>& vecMat, bool b_setIdentity = false, bool b_Normalize = true);
	void updatePose(vector<Mat4, aligned_allocator<Mat4>>& vecMat, vector<int>& vecMask, bool b_setIdentity = false, bool b_Normalize = true);
	void outputMesh(string fName);

public:
	MeshTree *_meshtree;
	ArticulateGlobalAlign::Param param;
};



//---------------------------- Implementation --------------------------------------//

ArticulateGlobalAlign::ArticulateGlobalAlign(ArticulateGlobalAlign::Param& param):param(param)
{
	_meshtree = NULL;
}
ArticulateGlobalAlign::~ArticulateGlobalAlign()
{
	SAFE_DELETE(_meshtree);
}
void ArticulateGlobalAlign::init(vector<CMeshO*> mesh)
{
	_meshtree = new MeshTree();
	_meshtree->addMeshtoMeshTreeFromVectorCMeshO(mesh);
}

void ArticulateGlobalAlign::processGlobalAlignment(bool if_control)
{
	MeshTree::Param meshtree_param(50000, param.arcThreshold, 0.3f);
	vcg::AlignPair::Param align_param(param.SampleNum, param.MaxIterNum, param.MinDistAbs);

	vector<double> record(param.GlobalAlignIteration);
	memset( record.data(),FLT_MAX, sizeof(double) * param.GlobalAlignIteration);
	vector<Mat4, aligned_allocator<Mat4>> mat_last(this->_meshtree->nodeList.size());

	int k = 0;
	bool b_Stop = false;
	bool b_RollBack = false;


	while(k < param.GlobalAlignIteration && !b_Stop)
	{
		cout<<endl<<"**************iteration *******************"<<k<<endl;
		record.at(k) = _meshtree->Process( align_param, meshtree_param, false);
		getResult(mat_last, false);
		for(int i = 0 ; i < mat_last.size(); i++)
		{
			if(!mat_last[i].allFinite())
			{
				mat_last[i].setIdentity();
			}
		}

		if(k > 1)
		{
			if(if_control)
			{
				if(record.at(k) < 2)
				{
					if(record.at(k) > record.at(k-1))
					{
						b_Stop = true;
						b_RollBack = true;
					}
					else if(record.at(k) == record.at(k-1) || record.at(k) <= 0.128)
					{
						b_Stop = true;
						b_RollBack = false;
					}
				}
			}
		}
		k ++;
	}

	if(b_RollBack == true)
	{
		setPose(mat_last);
	}

}

void ArticulateGlobalAlign::NormalizeTr()
{
	vector<CMeshO *> vecMesh;
	_meshtree->getMeshvector(vecMesh);
	Mat4 mat_first;
	vecMesh.at(0)->Tr.ToEigenMatrix(mat_first);
	if(mat_first.isIdentity())
		return;
	Mat4 mat_first_inv = mat_first.inverse();
	Matrix44f vcg_first_inv;
	vcg_first_inv.FromEigenMatrix(mat_first_inv);

	for(int i = 0 ; i < vecMesh.size() ; i++)
	{
		vecMesh.at(i)->Tr.Import(vecMesh.at(i)->Tr * vcg_first_inv);
	}
}

void ArticulateGlobalAlign::getResult(vector<Mat4, aligned_allocator<Mat4>>& vecMat, const vector<int>& mask, bool b_Normalize)
{
	if(b_Normalize)
		NormalizeTr();
	vector<CMeshO *> vecMesh;
	_meshtree->getMeshvector(vecMesh);

	for(int k = 0, i = 0; k < mask.size() ; k++)
	{
		if(mask.at(k) < 0)
			vecMat.at(k).setIdentity();
		else
		{
			Mat4 mat;
			vecMesh.at(i)->Tr.ToEigenMatrix(mat);
			vecMat.at(k) = mat * vecMat.at(k);
			i++;
		}
	}
}

void ArticulateGlobalAlign::getResult(vector<Mat4, aligned_allocator<Mat4>>& vecMat, bool b_Normalize)
{
	vector<CMeshO *> vecMesh;
	_meshtree->getMeshvector(vecMesh);

	for(int k = 0; k < vecMesh.size(); k++)
	{
		Mat4 mat;
		vecMesh.at(k)->Tr.ToEigenMatrix(mat);
		vecMat.at(k) = mat * vecMat.at(k);
	}
}

void ArticulateGlobalAlign::setPose(vector<Mat4, aligned_allocator<Mat4>>& vecMat)
{
	vector<CMeshO *> vecMesh;
	_meshtree->getMeshvector(vecMesh);
	assert(vecMesh.size() == vecMat.size());
	for(int i = 0 ; i < vecMesh.size() ; i++)
	{
		Matrix44f mat;
		mat.FromEigenMatrix(vecMat.at(i));
		vecMesh.at(i)->Tr.Import(mat);
	}
}

void ArticulateGlobalAlign::updatePose(vector<Mat4, aligned_allocator<Mat4>>& vecMat, bool b_setIdentity, bool b_Normalize)
{
	if(b_Normalize)
		NormalizeTr();
	vector<CMeshO *> vecMesh;
	_meshtree->getMeshvector(vecMesh);   
	assert(vecMesh.size() == vecMat.size());
	for(int i = 0 ; i < vecMesh.size() ; i++)
	{
		Matrix44f mat;
		mat.FromEigenMatrix(vecMat.at(i));
		vcg::tri::UpdatePosition<CMeshO>::Matrix(*vecMesh.at(i), mat, true);
		vcg::tri::UpdateBounding<CMeshO>::Box(*vecMesh.at(i));
		if(b_setIdentity)
			vecMesh.at(i)->Tr.SetIdentity();
	}
}

void ArticulateGlobalAlign::updatePose(vector<Mat4, aligned_allocator<Mat4>>& vecMat, vector<int>& vecMask, bool b_setIdentity, bool b_Normalize)
{
	if(b_Normalize)
		NormalizeTr();
	vector<CMeshO *> vecMesh;
	_meshtree->getMeshvector(vecMesh);
	assert(vecMat.size() == vecMask.size());

	for(int i = 0 , k = 0; i < vecMat.size() ; i++)
	{
		if(vecMask.at(i) < 0)
			continue;

		Matrix44f mat;
		mat.FromEigenMatrix(vecMat.at(i));
		vcg::tri::UpdatePosition<CMeshO>::Matrix(*vecMesh.at(k), mat, true);
		vcg::tri::UpdateBounding<CMeshO>::Box(*vecMesh.at(k));
		if(b_setIdentity)
			vecMesh.at(k)->Tr.SetIdentity();
		k++;
	}
}

void ArticulateGlobalAlign::outputMesh(string fName)
{
	NormalizeTr();
	_meshtree->outputMeshTreePLY(fName);
}