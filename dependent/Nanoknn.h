#pragma once
#include "nanoflann.h"
#include "Base.h"
#include "StdAfx.h"
#include "Helper.h"
#include "timer.h"
#include <Eigen/Dense>
using namespace std;
using namespace vcg;

template <typename T>
struct PointCloud
{
	struct Point
	{
		T  x,y,z;
	};

	std::vector<Point>  pts;

	inline size_t kdtree_get_point_count() const { return pts.size(); }

	inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t size) const
	{
		const T d0=p1[0]-pts[idx_p2].x;
		const T d1=p1[1]-pts[idx_p2].y;
		const T d2=p1[2]-pts[idx_p2].z;
		return d0*d0+d1*d1+d2*d2;
	}

	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim==0) return pts[idx].x;
		else if (dim==1) return pts[idx].y;
		else return pts[idx].z;
	}

	template <class BBOX>
	bool kdtree_get_bbox(BBOX &bb) const { return false; }

};

typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud<float> > ,	PointCloud<float>,	3 /* dim */	> my_kd_tree_t;

class Nanoknn
{
public:
	Nanoknn(CMeshO& _mod);
	Nanoknn(vector<CVertexO>& _vec);
	Nanoknn(vector<Vet3>& _vet);
	~Nanoknn()
	{
		SAFE_DELETE(_PC);
		SAFE_DELETE(_cvertexo);
		SAFE_DELETE(_kdtree);
	}
	void buildNano();
	inline void findNN(const CVertexO& v, int res_idx);
	inline void findKNN(const CVertexO& v, const int& k, vector<int>& knnidx);
	inline void findKNNNoSelf(const CVertexO& v, const int& k, vector<int>& knnidx);
	inline void findKNNwithW(const CVertexO& v, const int& k, vector<int>& knnidx, vector<float>& knnw);
	inline int  findBestMesh(const CVertexO& v, const float& weight_N, const vector<int>& knnidx);
	float correspondence(vector<CVertexO>& CorrS, vector<CVertexO>& CorrT, float weight_N = 0, float avgTimes = 20);
	inline float calcEnergy(const CVertexO & p1, const CVertexO & p2, const float& weight_N);
	void  weightAveragedeformMesh(CMeshO & mesh, const vector<Mat4, Eigen::aligned_allocator<Mat4>>& X_acc);

private:
	PointCloud<float> *_PC;
	vector<CVertexO> *_cvertexo;
	my_kd_tree_t * _kdtree;
};




//---------------------------- Implementation --------------------------------------//

Nanoknn::Nanoknn(CMeshO& _mod)
{
	_cvertexo = new vector<CVertexO>;
	_cvertexo->resize(_mod.vert.size());
	Concurrency::parallel_for(0, (int)_mod.vert.size(), [&](int k){
		Helper::copy(_cvertexo->at(k), _mod.vert.at(k));
	});

	_kdtree = NULL;
	_PC = NULL;
}

Nanoknn::Nanoknn(vector<CVertexO>& _vec)
{
	_cvertexo = new vector<CVertexO>;
	_cvertexo->resize(_vec.size());
	Concurrency::parallel_for(0, (int)_vec.size(), [&](int k){
		Helper::copy(_cvertexo->at(k), _vec.at(k));
	});

	_kdtree = NULL;
	_PC = NULL;
}

Nanoknn::Nanoknn(vector<Vet3>& _vet)
{
	_cvertexo = new vector<CVertexO>;
	_cvertexo->resize(_vet.size());
	Concurrency::parallel_for(0, (int)_vet.size(), [&](int k){
		_cvertexo->at(k).P() = Point3f(_vet.at(k).x(), _vet.at(k).y(), _vet.at(k).z());
		_cvertexo->at(k).N() = Point3f(0,0,1);
	});

	_kdtree = NULL;
	_PC = NULL;
}

inline void Nanoknn::buildNano()
{
	assert(_PC == NULL);
	Timer time;
	float globalTime = time.GlobalTime();

	_PC = new PointCloud<float>;
	_PC->pts.resize(_cvertexo->size());
	for (size_t i=0; i < _cvertexo->size(); i++)
	{
		_PC->pts[i].x = _cvertexo->at(i).P().X();
		_PC->pts[i].y = _cvertexo->at(i).P().Y();
		_PC->pts[i].z = _cvertexo->at(i).P().Z();
	}

	_kdtree = new my_kd_tree_t(3, *_PC, nanoflann::KDTreeSingleIndexAdaptorParams(16) );
	_kdtree->buildIndex();
	//std::cout << "Build nanoFlann: " << time.GlobalTime() - globalTime <<  " s" << std::endl;
}

inline void Nanoknn::findNN(const CVertexO& v, int res_idx)
{
	std::vector<size_t>   ret_index_clos(1);
	std::vector<float> out_dist_sqr_clos(1);
	_kdtree->knnSearch(v.P().V(), 1, &ret_index_clos[0], &out_dist_sqr_clos[0]);
	res_idx = static_cast<int>(ret_index_clos[0]);
}

inline void Nanoknn::findKNN(const CVertexO& v, const int& k, vector<int>& knnidx)
{
	std::vector<size_t>   ret_index(k);
	std::vector<float>   out_dist_sqr(k);
	_kdtree->knnSearch(v.P().V(), k, &ret_index[0], &out_dist_sqr[0]);
	knnidx.resize(k);
	for(int i = 0 ; i < k ; i ++)
	{
		knnidx[i] = static_cast<int>(ret_index[i]);
	}
}


inline void Nanoknn::findKNNNoSelf(const CVertexO& v, const int& k, vector<int>& knnidx)
{
	findKNN(v, k + 1, knnidx);
	for(int i = 0 ; i < k ; i++)
	{
		if(v.P() == _cvertexo->at(knnidx.at(i)).P())
		{
			knnidx.at(i) = knnidx.at(k);
			break;
		}
	}
	knnidx.resize(k);
}

inline void Nanoknn::findKNNwithW(const CVertexO& v, const int& k, vector<int>& knnidx, vector<float>& knnw)
{
	assert(knnw.size() == k);
	assert(knnidx.size() == k);
	std::vector<size_t>   ret_index(k + 1);
	std::vector<float>   out_dist_sqr(k + 1);
	_kdtree->knnSearch(v.P().V(), k + 1, &ret_index[0], &out_dist_sqr[0]);

	float dmax = out_dist_sqr[k];
	int idmax = k;

	dmax = sqrtf(dmax);
	if(dmax == 0) dmax = 0.001;

	float sum_w = 0;
	for(int i = 0 ; i < k ; i++)
	{
		if(out_dist_sqr[i] == 0) 
			out_dist_sqr[i] = 0.001;
		float d = sqrtf(out_dist_sqr[i]);
		float weight = (1 - (d)/(dmax));
		knnw[i] = weight;
		sum_w +=  weight;
	}

	for(int i = 0 ; i < k ; i++ )
	{
		knnw.at(i)/=sum_w;
		knnidx.at(i) = ret_index.at(i);
	}
}

inline float Nanoknn::calcEnergy(const CVertexO & p1, const CVertexO & p2, const float& weight_N)
{
	float d = (p1.P() - p2.P()).Norm();
	float n = p1.N().dot(p2.N());
	return d + n * weight_N;
};


inline int  Nanoknn::findBestMesh(const CVertexO& v, const float& weight_N, const vector<int>& knnidx)
{
	float min = FLT_MAX;
	int k = -1;
	for(int i = 0 ; i < knnidx.size(); i++)
	{
		float e =calcEnergy(v, _cvertexo->at(knnidx.at(i)), weight_N);
		( e < min ) ? min = e, k = i : min = min, k = k;
	}
	return knnidx.at(k);
}

void  Nanoknn::weightAveragedeformMesh(CMeshO & mesh, const vector<Mat4, Eigen::aligned_allocator<Mat4>>& X_acc)
{
	int knn = 4;
	vector<vector<float>> knnw(mesh.vert.size());
	vector<vector<int>> knnidx(mesh.vert.size());

	Concurrency::parallel_for(0,(int)mesh.vert.size(), [&](int i){

		knnidx.at(i).resize(knn);
		knnw.at(i).resize(knn);
		findKNNwithW(mesh.vert.at(i), knn, knnidx.at(i), knnw.at(i));}
	);

	Concurrency::parallel_for(0,(int)mesh.vert.size(), [&](int k){
		Vet3 v_avg(0,0,0) , n_avg(0,0,0);

		Vet3 p(mesh.vert.at(k).P().X(),mesh.vert.at(k).P().Y(),mesh.vert.at(k).P().Z());
		Vet3 n(mesh.vert.at(k).N().X(), mesh.vert.at(k).N().Y(),mesh.vert.at(k).N().Z());

		for(int i = 0 ; i < knn ; i++)
		{
			int idx = knnidx.at(k).at(i);
			float w = knnw.at(k).at(i);
			Vet3 x(_cvertexo->at(idx).P().X(),_cvertexo->at(idx).P().Y(),_cvertexo->at(idx).P().Z() );
			Mat3 A = X_acc.at(idx).block<3,3>(0,0);
			Vet3 b = X_acc.at(idx).block<3,1>(0,3);
			Vet3 pd = A * (p - x) + x + b;
			Vet3 nd = (A.inverse()).transpose() * n;
			nd.normalize();
			v_avg += pd * w;
			n_avg += nd * w;
		}
		mesh.vert.at(k).P() = Point3f(v_avg.x(), v_avg.y(), v_avg.z());
		mesh.vert.at(k).N() = Point3f(v_avg.x(), v_avg.y(), v_avg.z());
	});
}

float  Nanoknn::correspondence(vector<CVertexO>& CorrS, vector<CVertexO>& CorrT, float weight_N, float avgTimes)
{
	float error = 0;
	vector<int> knnidx;
	CorrT.clear();
	CorrT.reserve(CorrS.size());
	vector<CVertexO> ::iterator iter = CorrS.begin();

	CVertexO outlier;
	outlier.P() = Point3f(FLT_MAX, FLT_MAX, FLT_MAX);
	outlier.N() = Point3f(0,0,1);

	int rej_by_normal = 0;
	int rej_by_distance = 0;
	int rej_by_sourceboundary = 0;
	int rej_bt_targetboundary = 0;
	int stay= 0;

	for(; iter != CorrS.end(); iter++)
	{
		if(iter->N().Norm() != 1) 
			iter->N().Normalize();
		if(iter->P().Norm() > 1000000) 
		{
			rej_by_distance++;
			CorrT.push_back(outlier);
			continue;
		}
		knnidx.clear();
		this->findKNN((*iter), 10 , knnidx);
		int id_bm = this->findBestMesh((*iter), weight_N, knnidx);
		CVertexO bm = _cvertexo->at(id_bm);

		if(bm.IsV())
		{
			rej_bt_targetboundary++;
			CorrT.push_back(outlier);
			continue;
		}
		CorrT.push_back(bm);
		error += (bm.P() - (*iter).P()).Norm();
	}
	float average = error / (float) CorrS.size();

	for(int i = 0 ; i < CorrS.size(); i++ )
	{
		Point3f source_pos = CorrS.at(i).P();
		Point3f source_nor = CorrS.at(i).N();
		Point3f target_pos = CorrT.at(i).P();
		Point3f target_nor = CorrT.at(i).N();
		if(target_pos.Norm() >  1000000)
			continue;
		//check
		//if(CorrS.at(i).C()==(Color4b(0,255,0,255)))
		if(CorrS.at(i).IsV())
		{
			CorrT.at(i) = CorrS.at(i);
			stay ++;
			continue;
		}
		if(target_pos == outlier.P()) continue;
		if(source_nor.dot(target_nor) < 0.707)
		{
			CorrT.at(i) = outlier;
			rej_by_normal++;
			continue;
		}

		if((source_pos-target_pos).Norm() > avgTimes* average)
		{
			CorrT.at(i) = outlier;
			rej_by_distance++;
			continue;
		}
	}
	return error;
}