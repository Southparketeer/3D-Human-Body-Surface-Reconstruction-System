#pragma once
#include "StdAfx.h"
#include "base.h"

#include <wrap/ply/plylib.h>
#include <wrap/ply/plystuff.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>
#include <wrap/ply/plylib.cpp>
#include "SKELETON/vec.h"

using namespace Eigen;
using namespace vcg;
using namespace LUY_VEC;

class Helper
{
public:

	inline static Vector3f Point3f2Vector3f( const Point3f & p)
	{
		return Vector3f(p.X(), p.Y(), p.Z());
	}

	inline static Vector3f Vector3f2Vector3f( const Vector3f & p)
	{
		return Vector3f(p.x(), p.y(), p.z());
	}


	inline static Point3f Vector3f2Point3f( const Vector3f & p)
	{
		return Point3f(p.x(), p.y(), p.z());
	}

	inline static vec3f Point3f2vec3f( const Point3f & p)
	{
		return vec3f((float)p.X(), (float)p.Y(), (float)p.Z());
	};

	inline static Point3f vec3f2Point3f( const vec3f & p)
	{
		return Point3f(p.x, p.y, p.z);
	};

	inline static LUY_VEC::Vertex Vertex2CVertexO(const CVertexO & p)
	{
		LUY_VEC::Vertex v;
		v.pos = Point3f2vec3f(p.P());
		v.nor =Point3f2vec3f(p.N());
		return v;
	}

	inline static CVertexO CVertexO2Vertex(const LUY_VEC::Vertex & p)
	{
		CVertexO v;
		v.P() =  vec3f2Point3f(p.pos);
		v.N() =  vec3f2Point3f(p.nor);
		return v;
	}

	inline static void copy(CVertexO& a, CVertexO& b)
	{
		a.P() = b.P();
		a.N() = b.N();
		a.C() = b.C();
	}
	inline static void copyVector(vector<CVertexO>* des, vector<CVertexO>* src)
	{
		assert(des != NULL);
		assert(src != NULL);
		if(src->size() == 0) return;
		des->resize(src->size());
		for(int i = 0 ; i < src->size() ; i++)
		{
			copy(des->at(i), src->at(i)); 
		}
	}
};

class Log
{
public:

	inline static void LoadMesh(const char* filename, CMeshO & m)
	{
		int err=vcg::tri::io::Importer<vcg::CMeshO>::Open(m, filename);
		if(err)	{
			std::cerr << "Unable to open mesh " << filename << " : " << vcg::tri::io::Importer<vcg::CMeshO>::ErrorMsg(err) << std::endl;
			exit(-1);
		}

		tri::UpdateNormal<CMeshO>::PerVertex(m);
		tri::UpdateNormal<CMeshO>::NormalizePerVertex(m);

		printf( "%s------> has %i vert and %i faces\n", filename , m.VN(),  m.FN());
	}


	static void LogGNdeform(vector<Mat4, Eigen::aligned_allocator<Mat4>>* buff_mat, const char* filename)
	{
		FILE *fc = fopen(filename, "w+");	
		for(int i = 0 ; i < buff_mat->size() ; i++)
		{
			fprintf( fc, "%d\n",i);
			Mat4 mat = buff_mat->at(i);
			fprintf(fc, "%f %f %f %f\n", mat(0,0), mat(0,1), mat(0,2), mat(0,3));
			fprintf(fc, "%f %f %f %f\n", mat(1,0), mat(1,1), mat(1,2), mat(1,3));
			fprintf(fc, "%f %f %f %f\n", mat(2,0), mat(2,1), mat(2,2), mat(2,3));
			fprintf(fc, "%f %f %f %f\n", mat(3,0), mat(3,1), mat(3,2), mat(3,3));
		}
		fclose (fc);
		cout<<"Log  graph node deformation matrix "<<filename<<" Success!"<<endl;
	}




	static void LogGNdeformd(vector<Mat4, Eigen::aligned_allocator<Mat4>>* buff_mat, const char* filename)
	{
		FILE *fc = fopen(filename, "w+");	
		for(int i = 0 ; i < buff_mat->size() ; i++)
		{
			fprintf( fc, "%d\n",i);
			Mat4 mat = buff_mat->at(i);
			fprintf(fc, "%5.3f %5.3f %5.3f %5.3f\n", mat(0,0), mat(0,1), mat(0,2), mat(0,3));
			fprintf(fc, "%5.3f %5.3f %5.3f %5.3f\n", mat(1,0), mat(1,1), mat(1,2), mat(1,3));
			fprintf(fc, "%5.3f %5.3f %5.3f %5.3f\n", mat(2,0), mat(2,1), mat(2,2), mat(2,3));
			fprintf(fc, "%5.3f %5.3f %5.3f %5.3f\n", mat(3,0), mat(3,1), mat(3,2), mat(3,3));
		}
		fclose (fc);
		cout<<"Log  graph node deformation matrix "<<filename<<" Success!"<<endl;
	}

	static void LoadGNdeform(const char* filename, vector<Mat4, Eigen::aligned_allocator<Mat4>>* buff_mat)
	{
		ifstream file (filename);
		assert(file.is_open());
		int k = 0;
		buff_mat->clear();
		do
		{
			int idx;
			Mat4 mat;

			file>>idx;

			if(idx != k) {
				std::cout<<"index wrong!"<<endl;
				break;
			}

			file>> mat(0,0)>>mat(0,1)>>mat(0,2)>>mat(0,3);
			file>> mat(1,0)>>mat(1,1)>>mat(1,2)>>mat(1,3);
			file>> mat(2,0)>>mat(2,1)>>mat(2,2)>>mat(2,3);
			file>> mat(3,0)>>mat(3,1)>>mat(3,2)>>mat(3,3);
			buff_mat->push_back(mat);
			k++;
		}while((!file.eof()));
	}


	static void Logconnectbuff(vector<vector<int>>* buff_conn, const char* filename)
	{
		FILE *fc = fopen(filename, "w+");		
		for(int i = 0 ; i <  buff_conn->size(); i++)
		{
			fprintf( fc, "%d (%d) : ",i,  buff_conn->at(i).size());
			for(int k = 0 ; k < buff_conn->at(i).size(); k++)
			{
				fprintf( fc, "%d ", buff_conn->at(i).at(k));
			}
			fprintf( fc, "\n");
		}
		fclose (fc);
		cout<<"Log  connect buff "<<filename<<" Success!"<<endl;
	}

	static void Logmatrix(MatrixXf & X, const char* filename)
	{
		int width = 4;
		int height = 3;
		FILE *fc = fopen(filename, "w+");

		for(int k = 0 ; k < X.rows() / 12 ; k++)
		{
			MatrixXf Xsub = X.block<12,1>(k * 12 , 0);
			fprintf( fc, "(%i)\n%5.3f %5.3f %5.3f\n%5.3f %5.3f %5.3f\n%5.3f %5.3f %5.3f\n%5.3f %5.3f %5.3f\n\n",	k, 
				Xsub(0,0), Xsub(1,0), Xsub(2,0), 
				Xsub(3,0), Xsub(4,0), Xsub(5,0), 
				Xsub(6,0), Xsub(7,0), Xsub(8,0), 
				Xsub(9,0), Xsub(10,0), Xsub(11,0));
		}
		fclose (fc);
		cout<<"Log  connect buff "<<filename<<" Success!"<<endl;
	}

	static void Loadmatrix(const char* filename, MatrixXf& Xa)
	{
		ifstream file (filename);
		assert(file.is_open());
		int num;
		file>>num;
		MatrixXf X(num*12, 1);
		for(int k = 0 ; k < num; k++)
		{
			char a;
			int id;
			file>>a>>id>>a;
			assert(id == k);
			file>>(X)(k * 12 + 0)>>(X)(k * 12 +  1)>>(X)(k * 12 +  2)
				>>(X)(k * 12 + 3)>>(X)(k * 12 +  4)>>(X)(k * 12 +  5)
				>>(X)(k * 12 + 6)>>(X)(k * 12 +  7)>>(X)(k * 12 +  8)
				>>(X)(k * 12 + 9)>>(X)(k * 12 + 10)>>(X)(k * 12 + 11);
		}
		Xa = X;
		file.close();
	}

	//static void LogGlobalCorr(vector<GlobalCorr> * GC, const char* filename)
	//{
	//	FILE *fc = fopen(filename, "w+");
	//	for(int j = 0 ; j < GC->size() ; j++)
	//	{
	//		GlobalCorr gc = GC->at(j);
	//		fprintf(fc, "%i %i %i %i %5.3f %5.3f\n", gc.MeshId_S, gc.MeshId_T, gc.VexId_S, gc.Corr_T.idx, gc.Corr_T.u, gc.Corr_T.v);
	//	}
	//	fclose (fc);
	//	cout<<"Log  connect buff "<<filename<<" Success!"<<endl;
	//}


	static void LogMesh_Vertex(vector<LUY_VEC::Vertex>* bv, vector<vec3i>* be, const char* filename)
	{
		FILE *fc = fopen(filename, "w+");		
		fprintf(fc,"ply\n");
		fprintf(fc,"format ascii 1.0\n");
		fprintf(fc,"comment : created from Kinect depth image\n");
		fprintf(fc,"element vertex %d\n", bv->size());
		fprintf(fc,"property float x\n");
		fprintf(fc,"property float y\n");
		fprintf(fc,"property float z\n");

		if(be)	
		{
			fprintf(fc,"element face %d\n", be->size());  
			fprintf(fc,"property list uchar int vertex_indices\n");
		}
		fprintf(fc,"end_header\n");

		vector<LUY_VEC::Vertex>::iterator _iter1 = bv->begin();
		for( ; _iter1 != bv->end() ; _iter1 ++)
		{
			fprintf( fc, "%f %f %f ", _iter1->pos.x , _iter1->pos.y , _iter1->pos.z);
		}
		if(be)	
		{
			vector<vec3i>::iterator _iter2 = be->begin();
			for( ; _iter2 != be->end() ; _iter2 ++)
			{
				int a = 3;
				fprintf( fc, "%d %d %d %d\n", a , _iter2->x, _iter2->y , _iter2->z);
			}
		}
		fclose (fc);
		cout<<"Log PLY Model "<<filename<<" Success!"<<endl;
	}
	static void LogMesh_CMeshO(CMeshO * m, const char * filename, bool ifNormal = false, bool ifColor = false, bool if_binary = true)
	{
		vcg::tri::UpdateNormal<CMeshO>::NormalizePerVertex(*m);
		tri::io::ExporterPLY<CMeshO>::Save(*m, filename, vcg::tri::io::Mask::IOM_VERTNORMAL+vcg::tri::io::Mask::IOM_VERTCOLOR, if_binary);
	}


	static void LogPC_Vertex(vector<LUY_VEC::Vertex>* bv, const char* filename)
	{
		FILE *fc = fopen(filename, "w+");		
		fprintf(fc,"ply\n");
		fprintf(fc,"format ascii 1.0\n");
		fprintf(fc,"comment : created from Kinect depth image\n");
		fprintf(fc,"element vertex %d\n", bv->size());
		fprintf(fc,"property float x\n");
		fprintf(fc,"property float y\n");
		fprintf(fc,"property float z\n");
		fprintf(fc,"property float nx\n");
		fprintf(fc,"property float ny\n");
		fprintf(fc,"property float nz\n");
		fprintf(fc,"property uchar red\n");
		fprintf(fc,"property uchar green\n");
		fprintf(fc,"property uchar blue\n");

		fprintf(fc,"end_header\n");

		vector<LUY_VEC::Vertex>::iterator _iter1 = bv->begin();
		for( ; _iter1 != bv->end() ; _iter1 ++)
		{ 
			if(_iter1->pos.length()< 100000)
			{
				fprintf( fc, "%3.4f %3.4f %3.4f ", _iter1->pos.x , _iter1->pos.y , _iter1->pos.z);
				fprintf( fc, "%3.4f %3.4f %3.4f ", _iter1->nor.x , _iter1->nor.y , _iter1->nor.z);
				fprintf( fc, "%d %d %d\n", (int)_iter1->col.x , (int)_iter1->col.y , (int)_iter1->col.z);
			}
			else
			{
				fprintf( fc, "%3.4f %3.4f %3.4f ", 0 , 0 , 0);
				fprintf( fc, "%3.4f %3.4f %3.4f ", _iter1->nor.x , _iter1->nor.y , _iter1->nor.z);
				fprintf( fc, "%d %d %d\n",0, 0, 0);
			}
		}

		fclose (fc);
		cout<<"Log PLY Model "<<filename<<" Success!"<<endl;
	}

	static void LogPC_CVertexO(const char* filename, vector<CVertexO>* bv)
	{
		FILE *fc = fopen(filename, "w+");		
		fprintf(fc,"ply\n");
		fprintf(fc,"format ascii 1.0\n");
		fprintf(fc,"comment : created from Kinect depth image\n");
		fprintf(fc,"element vertex %d\n", bv->size());
		fprintf(fc,"property float x\n");
		fprintf(fc,"property float y\n");
		fprintf(fc,"property float z\n");
		fprintf(fc,"property float nx\n");
		fprintf(fc,"property float ny\n");
		fprintf(fc,"property float nz\n");
		fprintf(fc,"property uchar red\n");
		fprintf(fc,"property uchar green\n");
		fprintf(fc,"property uchar blue\n");

		fprintf(fc,"end_header\n");

		vector<CVertexO>::iterator _iter1 = bv->begin();
		for( ; _iter1 != bv->end() ; _iter1 ++)
		{ 
			if(_iter1->P().Norm()< 100000)
			{
				fprintf( fc, "%3.4f %3.4f %3.4f ", (float)_iter1->P().X() , (float)_iter1->P().Y(), (float)_iter1->P().Z());
				fprintf( fc, "%3.4f %3.4f %3.4f ", (float)_iter1->N().X() , (float)_iter1->N().Y(), (float)_iter1->N().Z());
				fprintf( fc, "%d %d %d\n", (int)_iter1->C().X() , (int)_iter1->C().Y()  , (int)_iter1->C().Z());
			}
			else
			{
				fprintf( fc, "%3.4f %3.4f %3.4f ", (float)0 , (float)0 , (float)0);
				fprintf( fc, "%3.4f %3.4f %3.4f ", (float)0 , (float)0, (float)1);
				fprintf( fc, "%d %d %d\n",0, 0, 0);
			}
		}

		fclose (fc);
		cout<<"Log PLY Model "<<filename<<" Success!"<<endl;
	}

	static void LoadPC_CVertexO(const char* filename, vector<CVertexO> * buff_pc)
	{
		vcg::CMeshO m;
		int err=vcg::tri::io::Importer<vcg::CMeshO>::Open( m, filename);
		if(err)	{
			std::cerr << "Unable to open mesh " << filename << " : " << vcg::tri::io::Importer<vcg::CMeshO>::ErrorMsg(err) << std::endl;
			exit(-1);
		}
		buff_pc->resize(m.vert.size());
		for(int i =0 ; i < m.vert.size() ; i++)
		{
			Helper::copy(buff_pc->at(i), m.vert.at(i));
		}
	}
	static void LoadPC_Point3f(const char* filename, vector<Point3f> * buff_pc)
	{
		vcg::CMeshO m;
		int err=vcg::tri::io::Importer<vcg::CMeshO>::Open( m, filename);
		if(err)	{
			std::cerr << "Unable to open mesh " << filename << " : " << vcg::tri::io::Importer<vcg::CMeshO>::ErrorMsg(err) << std::endl;
			exit(-1);
		}
		buff_pc->resize(m.vert.size());
		for(int i =0 ; i < m.vert.size() ; i++)
		{
			buff_pc->at(i) = m.vert.at(i).P();
		}
	}

	//static void LoadGlobalCorr(const char* filename, vector<GlobalCorr> * GC)
	//{
	//	ifstream file (filename);
	//	assert(file.is_open());
	//	int k = 0;
	//	GC->clear();
	//	GC->reserve(2000);
	//	do
	//	{
	//		GlobalCorr gc;
	//		file>> gc.MeshId_S >> gc.MeshId_T >> gc.VexId_S >> gc.Corr_T.idx >> gc.Corr_T.u >> gc.Corr_T.v;
	//		GC->push_back(gc);
	//	}while((!file.eof()));
	//	GC->reserve(GC->size());
	//	printf("Load GCorrespondence %s with %i samples. \n",filename, GC->size());
	//}

};

class Camera_Helper
{
public:
	static void setVMatrix(float3 eyepose, float3 eyecenter, float3 upvector, Mat4 & Vmatrix)
	{	
		float3 P0 = eyepose;
		float3 Pref = eyecenter;
		// here we inverse the direction of z
		//i.e k0 coordintate == world space coordinate
		float3 n = -normalize(P0 - Pref);
		float3 V = upvector;
		float3 u = normalize(cross(V , n));
		float3 v = normalize(cross(n , u));
		Mat4 VmatrixT;
		VmatrixT.setIdentity();
		VmatrixT(0,3) = -P0.x;
		VmatrixT(1,3) = -P0.y;
		VmatrixT(2,3) = -P0.z;
		Mat4 VmatrixR;
		VmatrixR.setIdentity();
		VmatrixR(0,0) = u.x;
		VmatrixR(0,1) = u.y;
		VmatrixR(0,2) = u.z;
		VmatrixR(1,0) = v.x;
		VmatrixR(1,1) = v.y;
		VmatrixR(1,2) = v.z;
		VmatrixR(2,0) = n.x;
		VmatrixR(2,1) = n.y;
		VmatrixR(2,2) = n.z;
		Vmatrix = (VmatrixR * VmatrixT);
	}
};

class Geometry_Helper
{
public:
	static void PointClouldPCA(const vector<Point3f>& PointCould, Point4f& PlaneParam)
	{
		//regression for Ax + By + Cz + D = 0 plane;
		Eigen::MatrixXf A(PointCould.size(), 3);
		Eigen::MatrixXf B(PointCould.size(), 1);
		for(int i = 0 ; i < PointCould.size(); i++)
		{
			A(i,0) = PointCould.at(i).X();
			A(i,1) = PointCould.at(i).Y();
			A(i,2) = 1;
			B(i,0) = PointCould.at(i).Z();
		}

		Eigen::MatrixXf AtA = A.transpose() * A;
		Eigen::MatrixXf AtB = A.transpose() * B;
		LLT<Matrix3f> llt;
		llt.compute(AtA);
		Eigen::MatrixXf X = llt.solve(AtB);
		cout <<X<<endl;
		PlaneParam = Point4f(X(0,0), X(1,0), -1, X(2,0));
	}
};
