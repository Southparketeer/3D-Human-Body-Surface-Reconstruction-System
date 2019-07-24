#pragma once
#include <bitset>
#include <vcg/space/index/grid_static_obj.h>

#include <vcg/math/histogram.h>
#include <vcg/math/matrix44.h>
#include <vcg/space/index/grid_static_ptr.h>
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/component_ep.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/component_ep.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/ply/plystuff.h>
#include <vcg/complex/algorithms/clean.h>

namespace vcg
{

	/* Occupancy Grid
	* Used to find the mesh pairs (arcs) to be used for ICP
	* It build a grid and for each cell count how many meshes passes through that cell
	* It compute the overlaps and returns a set of arcs with overlap greater than a given threshold.
	*/
	class OccupancyGrid
	{
	public:
		class AlignPair
		{
		public:
			class A2Vertex;

			class A2Face ;

			class A2UsedTypes: public vcg::UsedTypes < vcg::Use<A2Vertex>::AsVertexType,
				vcg::Use<A2Face>::AsFaceType >{};

			class A2Vertex   : public vcg::Vertex<A2UsedTypes,vcg::vertex::Coord3d,vcg::vertex::Normal3d,vcg::vertex::BitFlags> {};
			class A2Face     : public vcg::Face< A2UsedTypes,vcg::face::VertexRef, vcg::face::Normal3d,vcg::face::Mark,vcg::face::BitFlags> {};
			class A2Mesh     : public vcg::tri::TriMesh< std::vector<A2Vertex>, std::vector<A2Face> > 
			{ 
			public:
				bool Import(const char *filename) { Matrix44f Tr; Tr.SetIdentity(); return Import(filename,Tr);} 
				bool Import(const char *filename, Matrix44f &Tr);
				bool InitVert(const Matrix44f &Tr);
				bool Init(const Matrix44f &Tr);
			};
		};
	public:
		//typedef AlignPair::A2Mesh A2Mesh;

		// Class to keep for each voxel the id of the mesh passing throuhg it.
		// based on sorted vectors
		class MeshCounterV
		{
#define _MAX_MCV_SIZE 63 // Max number of meshes passing through a single cell.
			short last;
			short cnt[_MAX_MCV_SIZE];
		public:
			MeshCounterV(){last=0;cnt[last]=-1;}
			inline bool Empty() const {return last==0;}
			inline void Clear() {last=0;}
			inline bool IsSet(short const i) const
			{
				if(last==0) return false;
				const short *pt=std::lower_bound(cnt,cnt+last,i);
				return pt != (cnt+last);
				//	  return *pt==i;
			}

			int Count() const { return last; }

			void Set(int i) {
				assert(last>=0);
				if(last==0) {
					cnt[0]=i;
					++last;
					cnt[last]=-1;
					return;
				}
				short *pt=std::lower_bound(cnt,cnt+last,i);
				if(*pt==i) return;
				if(pt-cnt<last)
					memmove(pt+1,pt,(pt-cnt)*sizeof(short));
				*pt=i;
				++last;
				cnt[last]=-1;
				assert(last>=0);

				if(last>=_MAX_MCV_SIZE) 
				{
					abort();
				}
			}

			void UnSet(int i) 
			{
				if(last==0)	return;
				short *pt=std::lower_bound(cnt,cnt+last,i);
				if(*pt!=i) return;
				memmove(pt,pt+1,((cnt+last)-pt)*2);
				--last;
			}
			void Pack(std::vector<int> &v) const
			{
				v.resize(last);
				for(int i=0;i<last;++i)
				{
					v[i]=cnt[i];
				}
			}
			static int MaxVal() {return 32767;}
		};

		// Class to keep for each voxel the id of the mesh passing throuhg it.
		// based on bitset
		class MeshCounterB
		{
		private:
#define _MAX_MCB_SIZE 1024
			std::bitset<_MAX_MCB_SIZE> cnt;
		public:
			static int MaxVal() {return _MAX_MCB_SIZE;}
			bool Empty() const {return cnt.none();}
			void Clear() {cnt.reset();}
			bool IsSet(int i) const {return cnt.test(i);}
			void Set(int i) {cnt.set(i);}
			void UnSet(int i) {cnt.reset(i);}
			size_t Count() const { return cnt.count();}

			// Return a vector with all the id of the meshes
			void Pack(std::vector<int> &v) const
			{
				v.clear();
				for(int i=0;i<_MAX_MCB_SIZE;++i)
					if(cnt[i]==1) v.push_back(i);
			}

			void Dump() const
			{
				for(int i=0;i<64;++i) 
				{
					if((i%32)==0) printf(" " );
					if(cnt[i]) printf("1"); else printf("0");
				}
			}

			bool operator < (const MeshCounterB &c) const 
			{
				int ii=0;
				if(cnt==c.cnt) return false;
				while(ii<_MAX_MCB_SIZE)
				{
					if(cnt[ii]!=c.cnt[ii]) return cnt[ii]<c.cnt[ii];
					++ii;
				}
				return false;
			}
		};

		/***********************************************************/
		// Uncomment one of the following to switch between the two possible implementations of MeshCounter
		//typedef  MeshCounterV MeshCounter;
		typedef  MeshCounterB MeshCounter;
		/***********************************************************/

		class OGUseInfo
		{
		public:
			OGUseInfo() {id=-1; area=0;}
			OGUseInfo(const int _id, const int _area) :id(_id),area(_area){}
			int id;
			int area;
			bool operator < (const OGUseInfo &o) const { return area<o.area;}
		};

		class OGMeshInfo
		{
		public:
			enum {maxcnt =3};
			OGMeshInfo() {Init(); used=false;}
			void Init() {
				coverage=0;area=0;
			}

			std::vector<int> unicityDistribution; // Distribution of the occupancy ratios:
			// unicityDistribution[i] says how many voxel (among the ones coverd by this mesh) are covered by <i> othermeshes.
			// if unicityDistribution[1] > 0 means that this is the unique to cover some portion of the space.
			int coverage;  // and as 'covered by other mesh except itself (eg: if I have two mesh 1000 to 30% overlap with the covrg and' 300)
			int area;      // number of voxels touched by this mesh
			bool operator < (OGMeshInfo &o) const { return area<o.area;}
			static int MaxStat() { return 64;}
			bool used;
		};
		/* Classe con informazioni su un arco plausibile
		*/
		class OGArcInfo 
		{
		public:

			enum sort {AREA,NORM_AREA,DEGREE};
			int s,t; // source and target (come indici nel gruppo corrente)
			//ArcPt a;
			int area;  //
			float norm_area;
			OGArcInfo(int _s,int _t,int _area,float _n){s=_s;t=_t;area=_area;norm_area=_n;}
			OGArcInfo(int _s,int _t,int _a){s=_s;t=_t;area=_a;}

			bool operator <  (const OGArcInfo &p) const {return norm_area <  p.norm_area;}
		};

		void Clear();
		bool Init(int _mn, Box3f bb, int size);

		void Add(const char *MeshName, Matrix44f &Tr, int id);
		void AddMeshes(std::vector<std::string> &names, std::vector<Matrix44f> &trv,int size);
		template <class MESH>
		void AddMesh(MESH &M, const Matrix44f &Tr, int ind);

		void RemoveMesh(int id);

		void ChooseArcs(std::vector<std::pair<int,int> > &AV, std::vector<int> &BNV,std::vector<int> &adjcnt, float normarea= 0.3);
		void Compute();
		void ComputeUsefulMesh(FILE *elfp=0);
		//void Dump(FILE *fp);
		void Dump();
		void Dump(float&);
		void ComputeTotalArea();
		GridStaticObj<MeshCounter, float> G;
		std::vector<int> VA; // virtual arcs
		int mn;
		int TotalArea;
		int MaxCount;   // massimo numero di mesh che passano per una cella;
		std::vector<OGArcInfo>  SVA; // SortedVirtual Arcs;
		std::vector<OGMeshInfo> VM;  // vector of the information collected on the mesh.
		std::vector<OGUseInfo>  Use; // vector with the indices of the most 'useful mesh
	};

	// Implementation of the templated AddMesh
	template <class MESH>
	void OccupancyGrid::AddMesh(MESH &M, const Matrix44f &Tr, int ind)
	{
		Matrix44f Trf;
		Trf.Import(Tr);
		typename MESH::VertexIterator vi;
		for(vi=M.vert.begin();vi!=M.vert.end();++vi)
		{
			if(!(*vi).IsD())
				G.Grid( Trf * Point3f::Construct((*vi).P()) ).Set(ind);
		}
		
		VM[ind].Init();
		VM[ind].used=true;
	}
}


// Note that the box is automatically infected by GetBBox ();
bool OccupancyGrid::Init(int _mn, Box3f bb, int size)
{
	mn=_mn; // the number of meshes (including all the unused ones; eg it is the range of the possible id)
	if(mn>MeshCounter::MaxVal()) return false;
	MeshCounter MC;
	MC.Clear();
	G.Create(bb,size,MC);
	VM.clear();
	VM.resize(mn);
	return true;
}

void OccupancyGrid::Add(const char *MeshName, Matrix44f &Tr, int id)
{
	AlignPair::A2Mesh M;
	tri::io::Importer<AlignPair::A2Mesh>::Open(M,MeshName);
	tri::Clean<AlignPair::A2Mesh>::RemoveUnreferencedVertex(M);
	AddMesh(M,Tr,id);
}

void OccupancyGrid::AddMeshes(std::vector<string> &names, std::vector<Matrix44f> &trv, int size )
{
	unsigned int i;

	Box3f bb,totalbb;

	bb.SetNull();
	totalbb.SetNull();

	printf("OG::AddMesh:Scanning BBoxex\n");
	for(i=0;i<names.size();++i)
	{
		//ply::ScanBBox(names[i].c_str(),bb);
		//totalbb.Add( trv[i], bb);
	}
	Init(names.size(),totalbb,size);

	for(i=0;i<names.size();++i)
	{
		printf("OG::AddMesh:Adding Mesh %i '%s'\n",i,names[i].c_str());
		Add(names[i].c_str(),trv[i],i);
	}
}


void OccupancyGrid::Compute()
{
	// Analysis of the grid
	// You must find the most plausible set of edges
	// an arc has "sense" in a cell if both mesh appear in that span
	// Considering all the possible arcs and counts how many cells sense an arc
	VA.clear();
	VA.resize(mn*mn,0);

	// scan the grid and update possible arc count
	for(int i=0;i<G.siz[0];++i)
		for(int j=0;j<G.siz[1];++j)
			for(int k=0;k<G.siz[2];++k)
			{
				vector<int > vv;
				G.Grid(i,j,k).Pack(vv);
				size_t meshInCell = vv.size();
				for( size_t ii=0; ii< vv.size(); ++ii)
				{
					int meshId = vv[ii];
					++VM[meshId].area; // compute mesh area
					if(meshInCell>VM[meshId].unicityDistribution.size())
						VM[meshId].unicityDistribution.resize(meshInCell);
					++(VM[meshId].unicityDistribution[meshInCell-1]);
				}

				for(size_t ii=0;ii<vv.size();++ii)
					for(size_t jj=ii+1;jj<vv.size();++jj)
						++VA[vv[ii]+vv[jj]*mn]; // count intersections of all mesh pairs
			}

			// Find all the arcs
			SVA.clear();
			for(int i=0;i<mn-1;++i)
				if(VM[i].used)
					for(int j=i+1;j<mn;++j)
						if(VM[j].used && VA[i+j*mn]>0)
							SVA.push_back( OGArcInfo(i,j, VA[i+j*mn], VA[i+j*mn]/float( min(VM[i].area,VM[j].area)) ));

			// Compute Mesh Coverage
			for(size_t i=0;i<SVA.size();++i)
			{
				VM[SVA[i].s].coverage += SVA[i].area;
				VM[SVA[i].t].coverage += SVA[i].area;
			}

			sort(SVA.begin(),SVA.end());
			reverse(SVA.begin(),SVA.end());
}


void OccupancyGrid::ComputeTotalArea()
{
	int ccnt=0;
	MaxCount=0;
	int sz=G.size();
	for(int i=0;i<sz;++i)
		if(!G.grid[i].Empty()) 
		{
			ccnt++;
			if(G.grid[i].Count()>MaxCount) MaxCount=G.grid[i].Count();
		}

		TotalArea=ccnt;
}
/*
Ordering Range Map according to what are useful.
A Range Map and 'useful if it covers parts not yet seen

For each cell of the og c 'and' a bit that says which range map we pass
range map for each know the area (expressed in cells of the og)
It is considering a voxel seen if there are at least <K> range map covering it.
Initially multiplies * 1.2, ..K area all rm

It begins with the rm with greater area and decreases by one the area of all the other rm touches into voxels having less than two times.

*/
void OccupancyGrid::ComputeUsefulMesh(FILE *elfp)
{
	vector<int> UpdArea(mn);
	vector<int> UpdCovg(mn);

	Use.clear();
	int i,j,m,mcnt=0;
	for(m=0;m<mn;++m) 
	{
		if(VM[m].used && VM[m].area>0) 
		{
			mcnt++;
			UpdCovg[m]=VM[m].coverage;
			UpdArea[m]=VM[m].area;
		}
	}

	int sz=G.size();
	if(elfp) 
	{
		fprintf(elfp,"\n\nComputing Usefulness of Meshes of %i(on %i) meshes\n Og with %i / %i fill ratio %i max mesh per cell\n\n",mcnt,mn,TotalArea,sz,MaxCount);
		fprintf(elfp,"\n");

	}
	int CumArea=0;
	for(m=0;m<mn-1;++m)
	{

		int best = max_element(UpdArea.begin(),UpdArea.end())-UpdArea.begin();
		CumArea+=UpdArea[best];
		if(UpdCovg[best]<0) break;
		if(VM[best].area==0) continue; // se era una mesh fuori del working group si salta tutto.

		if(elfp) fprintf(elfp,"%3i %3i %7i (%7i) %7i %5.2f %7i(%7i)\n",
			m, best, UpdArea[best],VM[best].area, TotalArea-CumArea, 100.0-100*float(CumArea)/TotalArea, UpdCovg[best],VM[best].coverage);

		Use.push_back(OGUseInfo(best,UpdArea[best]));
		UpdArea[best]=-1;
		UpdCovg[best]=-1;

		for(i=0;i<sz;++i)
		{
			MeshCounter &mc=G.grid[i];
			if(mc.IsSet(best))	
			{
				mc.UnSet(best);
				for(j=0;j<mn;++j)
					if(mc.IsSet(j)) 
					{
						--UpdArea[j];
						UpdCovg[j]-=mc.Count();
					}
					mc.Clear();
			}
		}
	}
}

//void OccupancyGrid::Dump(FILE *fp)
//{
//	printf(fp,"Occupancy Grid\n");
//	printf(fp,"grid of ~%i kcells: %d x %d x %d\n",G.size(),G.siz[0],G.siz[1],G.siz[2]);
//	printf(fp,"grid voxel size of %f %f %f\n",G.voxel[0],G.voxel[1],G.voxel[2]);
//
//	fprintf(fp,"Computed %lu arcs for %i meshes\n",SVA.size(),mn);
//	for(size_t i=0;i<VM.size();++i)
//	{
//		if(VM[i].used)
//		{
//			fprintf(fp,"mesh %3lu area %6i covg %7i (%5.2f%%) Uniq:",i,VM[i].area,VM[i].coverage,float(VM[i].coverage)/float(VM[i].area));
//			for(size_t j=0;j<std::min(size_t(8),VM[i].unicityDistribution.size());++j)
//				fprintf(fp," %3i ", VM[i].unicityDistribution[j]);
//			fprintf(fp,"\n");
//		}
//		else
//			fprintf(fp,"mesh %3lu ---- UNUSED\n",i);
//	}
//	fprintf(fp,"Computed %lu Arcs :\n",SVA.size());
//	for(size_t i=0;i<SVA.size() && SVA[i].norm_area > .1; ++i)
//		fprintf(fp,"%4i -> %4i Area:%5i NormArea:%5.3f\n",SVA[i].s,SVA[i].t,SVA[i].area,SVA[i].norm_area);
//
//	fprintf(fp,"End OG Dump\n");
//}

void OccupancyGrid::Dump()
{
	printf("Occupancy Grid\n");
	printf("grid of ~%i kcells: %d x %d x %d\n",G.size(),G.siz[0],G.siz[1],G.siz[2]);
	printf("grid voxel size of %f %f %f\n",G.voxel[0],G.voxel[1],G.voxel[2]);

	printf("Computed %lu arcs for %i meshes\n",SVA.size(),mn);
	for(size_t i=0;i<VM.size();++i)
	{
		if(VM[i].used)
		{
#undef min;
			printf("mesh %3lu area %6i covg %7i (%5.2f%%) Uniq:",i,VM[i].area,VM[i].coverage,float(VM[i].coverage)/float(VM[i].area));
			for(size_t j=0;j<std::min(size_t(8),VM[i].unicityDistribution.size());++j)
				printf(" %3i ", VM[i].unicityDistribution[j]);
			printf("\n");
		}
		else
			printf("mesh %3lu ---- UNUSED\n",i);
	}
	printf("Computed %lu Arcs :\n",SVA.size());
	for(size_t i=0;i<SVA.size() && SVA[i].norm_area > .1; ++i)
		printf("%4i -> %4i Area:%5i NormArea:%5.3f\n",SVA[i].s,SVA[i].t,SVA[i].area,SVA[i].norm_area);

	printf("End OG Dump\n");
}

void OccupancyGrid::Dump(float& Area)
{
	printf("Occupancy Grid\n");
	printf("grid of ~%i kcells: %d x %d x %d\n",G.size(),G.siz[0],G.siz[1],G.siz[2]);
	printf("grid voxel size of %f %f %f\n",G.voxel[0],G.voxel[1],G.voxel[2]);

	printf("Computed %lu arcs for %i meshes\n",SVA.size(),mn);
	for(size_t i=0;i<VM.size();++i)
	{
		if(VM[i].used)
		{
#undef min;
			printf("mesh %3lu area %6i covg %7i (%5.2f%%) Uniq:",i,VM[i].area,VM[i].coverage,float(VM[i].coverage)/float(VM[i].area));
			for(size_t j=0;j<std::min(size_t(8),VM[i].unicityDistribution.size());++j)
				printf(" %3i ", VM[i].unicityDistribution[j]);
			printf("\n");
		}
		else
			printf("mesh %3lu ---- UNUSED\n",i);
	}
	printf("Computed %lu Arcs :\n",SVA.size());
	for(size_t i=0;i<SVA.size() && SVA[i].norm_area > .1; ++i)
		printf("%4i -> %4i Area:%5i NormArea:%5.3f\n",SVA[i].s,SVA[i].t,SVA[i].area,SVA[i].norm_area);
	
	Area = SVA[0].norm_area;
	
	printf("End OG Dump\n");
	
}


// sceglie gli archi da fare che abbiano una sovrapposizione di almeno <normarea>
// e restituisce la lista di nodi isolati;
void OccupancyGrid::ChooseArcs(vector<pair<int,int> > &AV, vector<int> &BNV, vector<int> &adjcnt, float normarea)
{
	AV.clear();
	BNV.clear();
	size_t i=0;
	adjcnt.clear();
	adjcnt.resize(mn,0);

	while(SVA[i].norm_area>normarea && i<SVA.size())
	{
		AV.push_back(make_pair( SVA[i].s, SVA[i].t) );

		++adjcnt[SVA[i].s];
		++adjcnt[SVA[i].t];
		++i;
	}

	// Second loop to add some more constraints we add also all the arc with area > normarea/3
	// and that connects meshes poorly connected (e.g. with zero or one adjacent)
	normarea/=3.0;
	while(SVA[i].norm_area>normarea && i<SVA.size())
	{
		if(adjcnt[SVA[i].s]<=1 || adjcnt[SVA[i].t]<=1 )
		{
			AV.push_back(make_pair( SVA[i].s, SVA[i].t) );

			++adjcnt[SVA[i].s];
			++adjcnt[SVA[i].t];
		}
		++i;
	}

	for(i=0;i<mn;++i) if(VM[i].used && adjcnt[i]==0) BNV.push_back(i);
}

void OccupancyGrid::RemoveMesh(int id)
{
	MeshCounter *GridEnd=G.grid+G.size();
	MeshCounter *ig;
	for(ig=G.grid;ig!=GridEnd;++ig)
		ig->UnSet(id);
}
