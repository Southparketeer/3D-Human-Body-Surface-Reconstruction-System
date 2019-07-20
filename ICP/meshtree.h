#ifndef EDITALIGN_MESHTREE_H
#define EDITALIGN_MESHTREE_H

#include <list>

#include "meshmodel.h"

#include <wrap/gui/trackball.h>
#include "ICP/OccupancyGrid.h"
#include "ICP/AlignPair.h"
#include "ICP/AlignGlobal.h"

class MeshNode
{
public:
	MeshNode(MeshModel *_m)
	{
		m=_m;
		glued = true;
	}

	bool glued;
	MeshModel *m;
	Matrix44m &tr() {return m->cm.Tr;}
	const Box3m &bbox() const {return m->cm.bbox;}
	int Id() {return m->id();}
	std::string Label() {return m->label();}
	~MeshNode()
	{
		delete m;
		m = nullptr;
	}
};

class MeshTree
{
public:
	class Param
	{
	public:
		int OGSize;
		double arcThreshold;
		double recalcThreshold;
		Param(double OG_size, double arc_Threshold, double recalc_Threshold )
		{
			OGSize = OG_size;
			arcThreshold = arc_Threshold;
			recalcThreshold = recalc_Threshold;
		}
	};

	MeshTree();
	~MeshTree();

	void getMeshvector(vector<CMeshO*>& mesh);
	void addMeshtoMeshTree(std::string name);
	void addMeshtoMeshTree(CMeshO& m, string name);
	void addMeshtoMeshTreeFromFile(std::string name, int num);
	void addMeshtoMeshTreeFromVectorCMeshO(vector<CMeshO*>& m);
	void outputMeshTreePLY(string s);
	void applyTrOnEachMesh();

	MeshModel *MM(unsigned int i);
	void clear();
	vcg::AlignPair::Result * findResult(int id1,int id2);
	void deleteResult(MeshNode *mp);
	MeshNode *find(int id);
	MeshNode *find(MeshModel *m);
	int gluedNum();
	double Process(vcg::AlignPair::Param &ap, Param &mtp, bool if_loop);
	double ProcessGlobal(vcg::AlignPair::Param &ap);
	void ProcessArc(int fixId, int movId, vcg::AlignPair::Result &result, vcg::AlignPair::Param ap);
	void ProcessArc(int fixId, int movId, vcg::Matrix44f &MovToFix, vcg::AlignPair::Result &result, vcg::AlignPair::Param ap);
	inline Box3m bbox();
	inline Box3m gluedBBox();

public:
	std::list<MeshNode *> nodeList;
	std::list<vcg::AlignPair::Result> resultList;
	vcg::OccupancyGrid OG;
};




//---------------------------- Implementation --------------------------------------//

using namespace vcg;
MeshTree::MeshTree()
{
	clear();
}

MeshTree::~MeshTree()
{
	std::list<MeshNode *>::iterator iter = this->nodeList.begin();
	for(; iter!= this->nodeList.end();  iter++)
	{
		SAFE_DELETE((*iter));
	}
	this->nodeList.clear();
}

void MeshTree::getMeshvector(vector<CMeshO*>& mesh)
{
	mesh.resize(nodeList.size());
	std::list<MeshNode *>::iterator iter = this->nodeList.begin();
	for(int i = 0 ; i < mesh.size() ;i++, iter++)
	{
		mesh.at(i) = &(*iter)->m->cm;
	}
}
void MeshTree::addMeshtoMeshTree(std::string name)
{
	int id = nodeList.size();
	//load mesh in meshmodel
	MeshModel *newMM = new MeshModel(id, name);
	MeshNode *newMN = new MeshNode(newMM);
	nodeList.push_back(newMN);
	printf("Add one Mesh Node into the MeshTree, the id is %i\n", id);
}


void MeshTree::addMeshtoMeshTree(CMeshO& m, string name)
{
	int id = nodeList.size();
	//load mesh in meshmodel
	MeshModel *newMM = new MeshModel(id, m, name);
	MeshNode *newMN = new MeshNode(newMM);
	nodeList.push_back(newMN);
	printf("Add one Mesh Node into the MeshTree, the id is %i\n", id);
}

void MeshTree::addMeshtoMeshTreeFromFile(std::string name, int num)
{
	string format(".ply");
	for(int k = 0 ; k < num ; k++)
	{
		int id = k;
		MeshModel *newMM = new MeshModel(id, name + to_string(id) + format);
		MeshNode *newMN = new MeshNode(newMM);
		nodeList.push_back(newMN);
		printf("Add one Mesh Node into the MeshTree, the id is %i\n", id);
	}
}

void MeshTree::addMeshtoMeshTreeFromVectorCMeshO(vector<CMeshO*>& m)
{
	for(int k = 0 , i = 0; k < m.size() ; k++)
	{
		if(m.at(k)->vert.size() != 0)
		{
			int id = i;
			MeshModel *newMM = new MeshModel(id, *m.at(k), to_string(id));
			MeshNode *newMN = new MeshNode(newMM);
			nodeList.push_back(newMN);
			printf("Add one Mesh Node into the MeshTree, the id is %i\n", id);
			i++;
		}
	}
}

void MeshTree::outputMeshTreePLY(string s)
{
	int i = 0 ;
	for(std::list<MeshNode *>::iterator iter = nodeList.begin(); iter != nodeList.end(); iter++, i++)
	{
		MeshNode * currMN = *iter;
		std::string outputname = s;
		outputname.append(to_string(i));
		outputname.append(".ply");
		tri::UpdatePosition<CMeshO>::Matrix(currMN->m->cm,currMN->m->cm.Tr);
		tri::io::ExporterPLY<CMeshO>::Save(currMN->m->cm,outputname.c_str());
		printf("Output %s\n", outputname.c_str());
	}
}

void MeshTree::applyTrOnEachMesh()
{
	for(std::list<MeshNode *>::iterator iter = nodeList.begin(); iter != nodeList.end(); iter++)
	{
		MeshNode * currMN = *iter;
		tri::UpdatePosition<CMeshO>::Matrix(currMN->m->cm,currMN->m->cm.Tr);
	}
}

int MeshTree::gluedNum()
{
	int cnt=0;
	std::list<MeshNode *> ::iterator iter = nodeList.begin();
	for(; iter != nodeList.end(); iter++)
	{
		MeshNode *mn = *iter;
		if(mn->glued) ++cnt;
	}
	return cnt;
}

void MeshTree::ProcessArc(int fixId, int movId, vcg::AlignPair::Result &result, vcg::AlignPair::Param ap)
{
	// aligner global change various matrices basic position of the mesh
	// for this reason we expect the points in the reference system of local mesh fix
	// They make all the accounts with respect to the local reference system of mesh fix
	vcg::Matrix44f FixM=vcg::Matrix44f::Construct(find(fixId)->tr());
	vcg::Matrix44f MovM=vcg::Matrix44f::Construct(find(movId)->tr());
	vcg::Matrix44f MovToFix = Inverse(FixM) * MovM;

	ProcessArc(fixId,movId,MovToFix,result,ap);
}
/** This is the main alignment function.
It takes two meshtree nodes and align the second node on the first one according to the parameters stored in <ap>
*/
void MeshTree::ProcessArc(int fixId, int movId, vcg::Matrix44f &MovM, vcg::AlignPair::Result &result, vcg::AlignPair::Param ap)
{
	vcg::AlignPair::A2Mesh Fix;
	vcg::AlignPair aa;

	// 1) Convert fixed mesh and put it into the grid.
	MM(fixId)->updateDataMask(MeshModel::MM_FACEMARK);
	aa.ConvertMesh<vcg::CMeshO>(MM(fixId)->cm,Fix);

	vcg::AlignPair::A2Grid UG;
	vcg::AlignPair::A2GridVert VG;

	if(MM(fixId)->cm.fn==0 || ap.UseVertexOnly)
	{
		Fix.InitVert(vcg::Matrix44f::Identity());
		vcg::AlignPair::InitFixVert(&Fix,ap,VG);
	}
	else
	{
		Fix.Init(vcg::Matrix44f::Identity());
		vcg::AlignPair::InitFix(&Fix, ap, UG);
	}
	// 2) Convert the second mesh and sample a <ap.SampleNum> points on it.
	MM(movId)->updateDataMask(MeshModel::MM_FACEMARK);
	std::vector<vcg::AlignPair::A2Vertex> tmpmv;
	aa.ConvertVertex(MM(movId)->cm.vert,tmpmv);
	aa.SampleMovVert(tmpmv, ap.SampleNum, ap.SampleMode);

	aa.mov=&tmpmv;
	aa.fix=&Fix;
	aa.ap = ap;

	vcg::Matrix44f In=MovM;
	// Perform the ICP algorithm
	aa.Align(In,UG,VG,result);

	result.FixName=fixId;
	result.MovName=movId;
	result.as.Dump();
}


// The main processing function
// 1) determine what AlignmentPair must be computed
// 2) do all the needed mesh-mesh alignment
// 3) do global alignment.
double MeshTree::Process(vcg::AlignPair::Param &ap, MeshTree::Param &mtp, bool if_loop = false)
{
	printf("\nStarting Processing of %i glued meshes out of %i meshes\n",gluedNum(),nodeList.size());
	printf("Computing Overlaps %i glued meshes...\n",gluedNum());
	/////////**********Test 1**********/////////
	OG.Init(nodeList.size(), vcg::Box3f::Construct(gluedBBox()), mtp.OGSize);
	std::list<MeshNode *> ::iterator iter = nodeList.begin();
	for(; iter != nodeList.end(); iter++)
	{
		MeshNode *mn = *iter;
		vcg::tri::UpdateBounding<CMeshO>::Box(mn->m->cm);
		if(mn->glued)	OG.AddMesh<vcg::CMeshO>(mn->m->cm, vcg::Matrix44f::Construct(mn->tr()), mn->Id());
	}

	OG.Compute();
	OG.Dump();
	if(if_loop)
	{
		float adjust_thr = 0.1;
		int total = nodeList.size();
		vector<int> record(0);
		record.reserve(total);
		for(int i = 0 ; i < OG.SVA.size(); i++)
		{
			if(abs(OG.SVA[i].s - OG.SVA[i].t) == 1)
			{
				record.push_back(min(OG.SVA[i].s, OG.SVA[i].t));
			}
			else if(abs(OG.SVA[i].s - OG.SVA[i].t) == total - 1 )
			{
				record.push_back(max(OG.SVA[i].s, OG.SVA[i].t));
			}
			if(record.size() == total) 
			{
				adjust_thr = OG.SVA[i].norm_area - 0.000005;
				break;
			}
		}
		if(adjust_thr < 0.2)
		{
			mtp.arcThreshold = adjust_thr;
		}
		else
		{
			mtp.arcThreshold = 0.2;
		}
		printf("\n-----------For Loop, adjust the adjust_thr = %3.4f ==> %3.4g-----------\n", adjust_thr, mtp.arcThreshold);
	}
	/////////**********Test 2**********/////////
	double percentileThr=0;
	if(resultList.size()>0)
	{
		vcg::Distribution<double> H;
		for(std::list<vcg::AlignPair::Result>::iterator li=resultList.begin();li!=resultList.end();++li)
			H.Add(li->err);
		percentileThr= H.Percentile(1.0f-mtp.recalcThreshold);
	}

	int totalArcNum=0;
	int preservedArcNum=0,recalcArcNum=0;

	while(totalArcNum<OG.SVA.size() && OG.SVA[totalArcNum].norm_area > mtp.arcThreshold) //SVA.size = 27
	{
		AlignPair::Result *curResult=findResult(OG.SVA[totalArcNum].s,OG.SVA[totalArcNum].t);
		if(curResult)
		{
			if(curResult->err < percentileThr)
				++preservedArcNum;
			else ++recalcArcNum;
		}
		++totalArcNum;
	}

	printf("Arc with good overlap %6i (on  %6lu)\n",totalArcNum,OG.SVA.size());
	printf(" %6i preserved %i Recalc \n",preservedArcNum,recalcArcNum);


	/////////**********Test 3**********/////////
	for(size_t i=0;i<OG.SVA.size() && OG.SVA[i].norm_area > mtp.arcThreshold; ++i)
	{
		//fprintf(stdout,"%4i -> %4i Area:%5i NormArea:%5.3f\n",OG.SVA[i].s,OG.SVA[i].t,OG.SVA[i].area,OG.SVA[i].norm_area);


		AlignPair::Result *curResult=findResult(OG.SVA[i].s,OG.SVA[i].t);
		if(curResult==0 || curResult->err >= percentileThr) // missing arc and arc with great error must be recomputed.
		{
			if(curResult==0) {
				resultList.push_back(AlignPair::Result());
				curResult= &resultList.back();
			}
			printf("(%3i/%3i) %2i->%2i----",int(i+1),totalArcNum,OG.SVA[i].s,OG.SVA[i].t);
			ProcessArc(OG.SVA[i].s, OG.SVA[i].t, *curResult, ap);//here we do the pairwise ICP
			curResult->area= OG.SVA[i].norm_area;

			if( curResult->IsValid() )
			{
				std::pair<double,double> dd=curResult->ComputeAvgErr();
				//printf("(%3i/%3i) %2i -> %2i Aligned AvgErr dd=%f -> dd=%f \n",int(i+1),totalArcNum,OG.SVA[i].s,OG.SVA[i].t,dd.first,dd.second);
			}
			else
			{
				//printf( "(%3i/%3i) %2i -> %2i Failed Alignment of one arc %s\n",int(i+1),totalArcNum,OG.SVA[i].s,OG.SVA[i].t,vcg::AlignPair::ErrorMsg(curResult->status));
				resultList.pop_back();
			}
		}
	}

	if(totalArcNum==0) {
		printf("\n Failure. There are no overlapping meshes?\n No candidate alignment arcs. Nothing Done.\n");
		return FLT_MAX;
	}
	if(resultList.empty()) {
		printf("\n Failure. No succesful arc among candidate Alignment arcs. Nothing Done.\n");
		return FLT_MAX;
	}

	vcg::Distribution<double> H; 
	for(std::list<vcg::AlignPair::Result>::iterator li=resultList.begin();li!=resultList.end();++li)
		H.Add(li->err);

	/////////**********Test 4**********/////////
	printf("\nCompleted Mesh-Mesh Alignment: Avg Err %5.3f Median %5.3f 0.90 %5.3f\n",H.Avg(),H.Percentile(0.5f),H.Percentile(0.9f));
	return ProcessGlobal(ap);
}

double MeshTree::ProcessGlobal(vcg::AlignPair::Param &ap)
{
	//QString buf;
	/************** Preparing Matrices for global alignment *************/

	vcg::Matrix44f Zero44; Zero44.SetZero();
	std::vector<vcg::Matrix44f> PaddedTrVec(nodeList.size(),Zero44);
	// matrix trv[i] is relative to mesh with id IdVec[i]
	// if all the mesh are glued GluedIdVec=={1,2,3,4...}
	std::vector<int> GluedIdVec;
	std::vector<vcg::Matrix44f> GluedTrVec;

	std::map<int,std::string> names;

	std::list<MeshNode *> ::iterator iter = nodeList.begin();
	for(; iter != nodeList.end(); iter++)
	{
		MeshNode *mn = *iter;
		if(mn->glued)
		{
			GluedIdVec.push_back(mn->Id());
			GluedTrVec.push_back(vcg::Matrix44f::Construct(mn->tr()));
			PaddedTrVec[mn->Id()]=GluedTrVec.back();
			names[mn->Id()]=std::string(mn->m->label());
		}
	}

	vcg::AlignGlobal AG;
	std::vector<vcg::AlignPair::Result *> ResVecPtr;
	for(std::list<vcg::AlignPair::Result>::iterator li=resultList.begin();li!=resultList.end();++li)
		ResVecPtr.push_back(&*li);
	AG.BuildGraph(ResVecPtr, GluedTrVec, GluedIdVec);

	double StartGlobErr = 0.001f;
	while(!AG.GlobalAlign(names, 	StartGlobErr, 100, ap.MatchMode==vcg::AlignPair::Param::MMRigid, stdout)){
		StartGlobErr*=2;
		AG.BuildGraph(ResVecPtr,GluedTrVec, GluedIdVec);
	}

	std::vector<vcg::Matrix44f> GluedTrVecOut(GluedTrVec.size());
	AG.GetMatrixVector(GluedTrVecOut,GluedIdVec);

	//Now get back the results!
	for(int ii=0;ii<GluedTrVecOut.size();++ii)
	{
		Mat4 check;
		GluedTrVecOut[ii].ToEigenMatrix(check);
		if(check.allFinite())
		{
			MM(GluedIdVec[ii])->cm.Tr.Import(GluedTrVecOut[ii]);
		}
	}

	printf("Completed Global Alignment (error bound %6.4f)\n",StartGlobErr);
	printf("----------------Global Alignment Finish----------------\n\n");
	return StartGlobErr;
}

MeshModel* MeshTree::MM(unsigned int i) {
	std::list<MeshNode *> ::iterator iter = nodeList.begin();
	for(int a = 0 ; a < i ; a++)	iter++;
	MeshNode * MN= (*iter);
	return MN->m;
}

void MeshTree::clear()
{
	if(nodeList.size() != 0)
	{
		for(std::list<MeshNode *>::iterator iter = nodeList.begin(); iter!= nodeList.end(); iter++)
			delete (*iter);
	}
	nodeList.clear();
	resultList.clear();
}

vcg::AlignPair::Result* MeshTree::findResult(int id1,int id2)
{
	for(std::list<vcg::AlignPair::Result>::iterator li=resultList.begin();li!=resultList.end();++li)
		if((li->MovName==id1 && li->FixName==id2) ||
			(li->MovName==id2 && li->FixName==id1) ) return &*li;

	//assert("You are trying to find an unexistent result"==0);
	return 0;
}

void MeshTree::deleteResult(MeshNode *mp)
{
	std::list<vcg::AlignPair::Result>::iterator li=resultList.begin();
	while(li!=resultList.end())
	{
		if(li->MovName==mp->Id() || li->FixName==mp->Id())
			li=resultList.erase(li);
		else ++li;
	}
}

MeshNode* MeshTree::find(int id)
{
	std::list<MeshNode *>::iterator iter = nodeList.begin();
	for( ; iter != nodeList.end() ; iter++)
	{
		MeshNode *mp = *iter;
		if(mp->Id()==id) return mp;
	}
	assert("You are trying to find an unexistent mesh"==0);
	return 0;
}

MeshNode* MeshTree::find(MeshModel *m)
{
	std::list<MeshNode *>::iterator iter = nodeList.begin();
	for( ; iter != nodeList.end() ; iter++)
	{
		MeshNode *mp = *iter;
		if(mp->m==m) return mp;
	}
	assert("You are trying to find an unexistent mesh"==0);
	return 0;
}

inline Box3m MeshTree::bbox() {
	Box3m FullBBox;
	std::list<MeshNode *>::iterator iter = nodeList.begin();
	for( ; iter != nodeList.end() ; iter++)
	{
		MeshNode *mp = *iter;
		FullBBox.Add(Matrix44m::Construct(mp->tr()),mp->bbox());
	}
	return FullBBox;
}

inline Box3m MeshTree::gluedBBox() {
	Box3m FullBBox;
	std::list<MeshNode *>::iterator iter = nodeList.begin();
	for( ; iter != nodeList.end() ; iter++)
	{
		MeshNode *mp = *iter;
		if(mp->glued)
			FullBBox.Add(Matrix44m::Construct(mp->tr()),mp->bbox());
	}
	return FullBBox;

}
#endif