#ifndef __VCG_ALIGNPAIR
#define __VCG_ALIGNPAIR
#include "StdAfx.h"
#include <ctime>
#include <vcg/math/histogram.h>
#include <vcg/math/matrix44.h>
#include <vcg/space/index/grid_static_ptr.h>
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/component_ep.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/component_ep.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/update/position.h>
#include <vcg/complex/algorithms/update/component_ep.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/closest.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>

#include <vcg/math/random_generator.h>
#include <vcg/math/gen_normal.h>
#include <vcg/space/point_matching.h>


namespace vcg
{
	/*************************************************************************
	AlignPair
	Class to handle an alignment between two mesh sun
	**************************************************************************/

	class AlignPair
	{
	public: 

		enum ErrorCode {SUCCESS, NO_COMMON_BBOX, TOO_FEW_POINTS, 
			LSQ_DIVERGE, TOO_MUCH_SHEAR, TOO_MUCH_SCALE, FORBIDDEN, INVALID, UNKNOWN_MODE };
		/*********************** Classi Accessorie ****************************/
		class A2Vertex;

		class A2Face ;

		class A2UsedTypes: public vcg::UsedTypes < vcg::Use<A2Vertex>::AsVertexType,
			vcg::Use<A2Face  >::AsFaceType >{};

		class A2Vertex   : public vcg::Vertex<A2UsedTypes,vcg::vertex::Coord3f,vcg::vertex::Normal3f, vcg::vertex::Color4b, vcg::vertex::BitFlags> {};
		class A2Face     : public vcg::Face< A2UsedTypes,vcg::face::VertexRef, vcg::face::Normal3f,vcg::face::Mark,vcg::face::BitFlags> {};
		class A2Mesh     : public vcg::tri::TriMesh< std::vector<A2Vertex>, std::vector<A2Face> > 
		{ 
		public:
			bool Import(const char *filename) { Matrix44<float> Tr; Tr.SetIdentity(); return Import(filename,Tr);} 
			bool Import(const char *filename, Matrix44<float> &Tr);
			bool InitVert(const Matrix44<float> &Tr);
			bool Init(const Matrix44<float> &Tr);
		};

		typedef A2Mesh::FaceContainer FaceContainer;	  
		typedef A2Mesh::FaceType      FaceType;	  
		typedef GridStaticPtr<FaceType, float > A2Grid;
		typedef GridStaticPtr<A2Mesh::VertexType, float > A2GridVert;

		class Stat
		{
		public:

			class IterInfo
			{
			public:
				IterInfo() {memset ( (void *) this, 0, sizeof(IterInfo)); }

				float MinDistAbs;    
				int DistanceDiscarded;  //rejected by distance
				int ColorDiscarded;
				int AngleDiscarded;		//rejected by angle compalitity
				int BorderDiscarded;    //rejected due to the boundary edge
				int SampleTested;		// how many points I have tested with MinDist
				int SampleUsed;			// how many points I chose for computematrix
				float pcl50;
				float pclhi;
				float AVG;
				float RMS;
				float StdDev;
				int Time;  // and when finished this iteration

			};
			std::vector<IterInfo> I; 

			float FirstPc150() const {return I.front().pcl50;}
			float LastPcl50() const {return I.back().pcl50;}
			int LastSampleUsed() const {return I.back().SampleUsed;}

			int MovVertNum;
			int FixVertNum;
			int FixFaceNum;
			int TotTime() { return I.back().Time-StartTime; }
			int IterTime(unsigned int i) const 
			{   const int clock_per_ms = std::max<int>(CLOCKS_PER_SEC / 1000,1);
			assert(i<I.size()); 
			if(i==0) return  (I[i].Time-StartTime )/clock_per_ms;
			else return (I[i].Time - I[i-1].Time)/clock_per_ms ;
			}
			int StartTime;
			void clear();
			void Dump();	
			bool Stable(int last);

		};


		class Param
		{
		public:
			enum MatchModeEnum  {MMSimilarity, MMRigid};
			enum SampleModeEnum {SMRandom, SMNormalEqualized};

			Param()
			{
				SampleNum    = 500;
				MaxPointNum     = 100000;
				MinPointNum  =  100;

				MaxIterNum   =   50;
				TrgDistAbs  = 0.05f;	// if framing a square 10cm x 10cm means 0.05 mm

				MinDistAbs   = 50;
				MaxAngleRad  = math::ToRad(30.0);
				MaxShear     = 0.5;
				MaxScale     = 0.5;    // It means that the scales must be between 1 and 1-MaxScale + MaxScale
				PassHiFilter = 0.75;
				ReduceFactorPerc = 0.80;
				MinMinDistPerc = 0.01;
				EndStepNum   = 5;
				MatchMode    = MMRigid;
				//SampleMode   = SMRandom;//SMNormalEqualized;
				SampleMode   = SMNormalEqualized;
				UGExpansionFactor=10;
				MinFixVertNum=20000;
				MinFixVertNumPerc=.25;
				UseVertexOnly = false;
				Threshold_Color = 500;
			}

			Param(int Sample_Num, int Max_Iter, float Min_Distance, float Thr_color = 173)
			{
				SampleNum    = Sample_Num;
				MaxPointNum  = 100000;
				MinPointNum  =  100;

				MaxIterNum   =   Max_Iter;
				TrgDistAbs	 = 0.05f;	// if framing a square 10cm x 10cm means 0.05 mm

				MinDistAbs   =  Min_Distance;
				MaxAngleRad  = math::ToRad(45.0);
				MaxShear     = 0.5;
				MaxScale     = 0.5;    // It means that the scales must be between 1 and 1-MaxScale + MaxScale
				PassHiFilter = 0.75;
				ReduceFactorPerc = 0.80;
				MinMinDistPerc = 0.01;
				EndStepNum   = 5;
				MatchMode    = MMRigid;
				SampleMode   = SMNormalEqualized;
				UGExpansionFactor=10;
				MinFixVertNum=20000;
				MinFixVertNumPerc=.25;
				UseVertexOnly = false;
				Threshold_Color = Thr_color;
			}

			int SampleNum;			 // number of samples to be taken on the mesh fix, using the SampleMode chosen including then choose the most <MaxPointNum> and at least <MinPointNum> to use for alignment.
			int MaxPointNum;		 // number of pairs of points to use when calculating the matrix of allienamento and then put them aside for the global economy; usually not used
			int MinPointNum;		 // minimum number of pairs of points is considered permissible please Valid alignment
			float MinDistAbs;	     // minimum initial because two points are taken into account. NOT more and expressed as a percentage of bbox mesh in ug. At each step you can be reduced to speed using ReduceFactor
			float MaxAngleRad;	     // maximum angle in radians between two normal because the two points are taken into account.
			int MaxIterNum;			 // maximum number of iterations to make align
			float TrgDistAbs;		 // target distance within which must be at least half of the samples selected; usually it does not come into play because 'has a low default
			int EndStepNum;			 // number of iterations to consider when deciding if icp converged.
			float PassHiFilter;     // Filtering used to decide which points to choose between that found close enough. Expressed in percentiles. Usually you discard the ones over 75 and those under 5
			float ReduceFactorPerc; // At each step we discard the points farther than a given threshold. The threshold is reduced iterativeley;
			float MinMinDistPerc;	 // Ratio between initial starting distance (MinDistAbs) and what can reach by the application of the ReduceFactor. 
			int UGExpansionFactor;   // Size of UG to fix the mesh as the ratio of the number of mesh faces

			// If using some structure multiresolution
			int   MinFixVertNum;	  // The alignments are made by putting in ug as meshes fix a simplified;
			float MinFixVertNumPerc;  // using the max between MinFixVertNum and OrigMeshSize * MinFixVertNumPerc
			bool UseVertexOnly;       // if true to the Alignment pipeline ignores faces and works over point clouds.

			float MaxShear;
			float MaxScale;
			float Threshold_Color;
			MatchModeEnum MatchMode;
			SampleModeEnum SampleMode;
			void Dump(FILE *fp,float BoxDiag);

		};

		// Class to store the result of an alignment between two mesh
		// points are considered in the frame of the two initial mesh.
		//
		// if the meshes have a transformation of the input database,
		// this only appears during A2Mesh :: Import and then 'forgotten forever.
		// These points are then in the reference systems constructed during the Import
		// the matrix Tr that	Tr*Pmov[i]== Pfix

		class Result
		{
		public: 
			int MovName;
			int FixName;

			Matrix44<float> Tr;
			std::vector<Point3f> Pfix;		// corresponding vertices of fix (red)
			std::vector<Point3f> Nfix; 		// normal corresponding fix on (red)
			std::vector<Point3f> Pmov;		// Vertices chosen on mov (green) before processing input (Original Point Target)
			std::vector<Point3f> Nmov; 		// Normal chosen on mov (green)
			Histogramf H;
			Stat as;
			Param ap;
			ErrorCode status;
			bool IsValid() {return status==SUCCESS;}
			float err;
			float area; // the overlapping area, a percentage as computed in Occupancy Grid.

			bool operator <  (const Result & rr) const {return (err< rr.err);}
			bool operator <= (const Result & rr) const {return (err<=rr.err);}
			bool operator >  (const Result & rr) const {return (err> rr.err);}
			bool operator >= (const Result & rr) const {return (err>=rr.err);}
			bool operator == (const Result & rr) const {return (err==rr.err);}
			bool operator != (const Result & rr) const {return (err!=rr.err);}

			std::pair<float,float> ComputeAvgErr() const 
			{
				float sum_before=0;
				float sum_after=0;
				for(unsigned int ii=0;ii<Pfix.size();++ii)
				{
					sum_before+=Distance(Pfix[ii],   Pmov[ii]);
					sum_after+=Distance(Pfix[ii], Tr*Pmov[ii]);
				}
				return std::make_pair(sum_before/float(Pfix.size()),sum_after/float(Pfix.size()) ) ;
			}

		};

		/******************* Classes end Accessorie ************************/

		static const char *ErrorMsg( ErrorCode code);
		void Clear(){status=SUCCESS;}
		AlignPair() {Clear();}

		/******* Data Members *********/

		std::vector<A2Vertex> *mov; //align vertex (subsampled)
		A2Mesh *fix; // to mesh

		ErrorCode status;
		AlignPair::Param ap;

		/**** End Data Members *********/

		template < class MESH >
		void ConvertMesh(MESH &M1, A2Mesh &M2) // convert mesh to target mesh format to be used as target mesh
		{
			tri::Append<A2Mesh,MESH>::MeshCopy(M2,M1); 
		}

		template < class VERTEX >
		void ConvertVertex(const std::vector<VERTEX> &vert1, std::vector<A2Vertex> &vert2, Box3f *Clip=0) //convert vertex to source vertex formate to be used as source mesh
		{
			vert2.clear();
			typename std::vector<VERTEX>::const_iterator vi;
			A2Vertex tv;
			Box3<typename VERTEX::ScalarType> bb;
			if(Clip){
				bb.Import(*Clip);
				for(vi=vert1.begin();vi<vert1.end();++vi)
					if(!(*vi).IsD() && bb.IsIn((*vi).cP())){
						tv.P().Import((*vi).cP());
						tv.N().Import((*vi).cN());
						tv.C().Import((*vi).cC());
						vert2.push_back(tv);
					}
			}
			else
				for(vi=vert1.begin();vi<vert1.end();++vi)
					if(!(*vi).IsD()){
						tv.P().Import((*vi).cP());
						tv.N().Import((*vi).cN());
						tv.C().Import((*vi).cC());
						vert2.push_back(tv);
					}		
		}

		//Sample the source vertex before we use it, there are 3/2? different strategies that we can use
		bool SampleMovVert(std::vector<A2Vertex> &vert, int SampleNum, AlignPair::Param::SampleModeEnum SampleMode);
		bool SampleMovVertRandom(std::vector<A2Vertex> &vert, int SampleNum);
		bool SampleMovVertNormalEqualized(std::vector<A2Vertex> &vert, int SampleNum);
		/*
		Once found <SampleNum> pairs of corresponding points if they choose
		at most <PointNum> to compute the transformation that makes them coincide.
		The choice is made based on these two parameters and PassLo PassHi
		*/
		bool ChoosePoints( 	std::vector<Point3f> &Ps,		// corresponding vertices of fix (red)
			std::vector<Point3f> &Ns, 						// normal corresponding fix on (red)
			std::vector<Point3f> &Pt,						// vertici scelti su mov (verdi) 
			std::vector<Point3f> &OPt,						// vertici scelti su mov (verdi) 
			float PassHi,
			Histogramf &H)
			;

		bool Align(const Matrix44<float> &in, A2Grid &UG, A2GridVert &UGV, Result &res)
		{
			res.ap=ap;

			bool ret=Align(	UG,UGV,
				in,
				res.Tr,
				res.Pfix, res.Nfix,
				res.Pmov, res.Nmov,
				res.H, 
				res.as);

			res.err=res.as.LastPcl50();
			res.status=status;
			return ret;	
		}

		float Abs2Perc(float val, Box3f bb) const {return val/bb.Diag();}
		float Perc2Abs(float val, Box3f bb) const {return val*bb.Diag();}

		/************************************************************************************
		Vera version of Align low level.
		It assumes that the mesh fix is already 'been put in ug u with the necessary transformations.		in 
		************************************************************************************/

		bool Align(A2Grid &u,
			A2GridVert &uv,
			const Matrix44<float> &in,					// Initial transformation leading points mov on fix
			Matrix44<float> &out,							// transformation calculated
			std::vector<Point3f> &Pfix,				// corresponding vertices of src (red)
			std::vector<Point3f> &Nfix, 			// src of corresponding normal (red)
			std::vector<Point3f> &OPmov,			// Vertexice chosen on trg (green) before processing input (Original Point Target)
			std::vector<Point3f> &ONmov, 			// normal chosen on trg (green)
			Histogramf &H,
			Stat &as);

		bool InitMov(
			std::vector< Point3f > &movvert,
			std::vector< Point3f > &movnorm,
			std::vector< Color4b > &movcol,
			Box3f &movbox,
			const Matrix44<float> &in	);				

		static bool InitFixVert(A2Mesh *fm,
			AlignPair::Param &pp,
			A2GridVert &u,
			int PreferredGridSize=0);

		static bool InitFix(A2Mesh *fm,
			AlignPair::Param &pp,
			A2Grid &u,
			int PreferredGridSize=0);

	}; // end class

} // end namespace vcg


using namespace vcg;
using namespace std;

////////////////
//AlignPair::A2Mesh: Load Mesh and preprocessing
////////////////

//as we import ply mesh into the program
bool AlignPair::A2Mesh::Import(const char *filename, Matrix44<float> &Tr)
{
	int err = tri::io::Importer<A2Mesh>::Open(*this,filename);
	if(err) {
		printf("Error in reading %s: '%s'\n",filename,tri::io::Importer<A2Mesh>::ErrorMsg(err));
		exit(-1);
	}
	printf("read mesh `%s'\n", filename);
	return Init(Tr);
}

//if input matrix is not an identity matrix, than update vertex, normal and bounding box accordingly
bool AlignPair::A2Mesh::InitVert(const Matrix44<float> &Tr)
{
	Matrix44<float> Idn; Idn.SetIdentity();
	if(Tr!=Idn) tri::UpdatePosition<A2Mesh>::Matrix(*this,Tr);
	tri::UpdateNormal<A2Mesh>::NormalizePerVertex(*this);
	tri::UpdateBounding<A2Mesh>::Box(*this);
	return true;
}

//do preprocessing on the input mes:
//1) remove unreference vertex
//2) upadate position accoreding the the input matrix
//3) Equivalent to PerVertexNormalizedPerFace() and NormalizePerFace().
//4) Computes per-face border flags without requiring any kind of topology It has a O(fn log fn) complexity.
//5) Calculates the bounding box of the given mesh m.
bool AlignPair::A2Mesh::Init(const Matrix44<float> &Tr)
{
	Matrix44<float> Idn; Idn.SetIdentity();
	tri::Clean<A2Mesh>::RemoveUnreferencedVertex(*this);
	if(Tr!=Idn) tri::UpdatePosition<A2Mesh>::Matrix(*this,Tr);
	tri::UpdateNormal<A2Mesh>::PerVertexNormalizedPerFaceNormalized(*this);
	tri::UpdateFlags<A2Mesh>::FaceBorderFromNone(*this);
	tri::UpdateBounding<A2Mesh>::Box(*this);

	return true;
}

////////////////
//AlignPair::Stat:: Gather statistic info
////////////////
void AlignPair::Stat::clear()
{
	I.clear();
	StartTime=0;
	MovVertNum=0;
	FixVertNum=0;
	FixFaceNum=0;
}

//Returns true if the last <lastiter> iterations and not 'diminished
// the error
bool AlignPair::Stat::Stable(int lastiter)
{
	if(I.empty()) return false;
	int parag = I.size()-lastiter;

	if(parag<0) parag=0;
	if( I.back().pcl50 < I[parag].pcl50 ) return false; // if we are not reduced and stable

	return true;

}

void AlignPair::Stat::Dump()
{
	if(I.size()==0) {
		printf("Empty AlignPair::Stat\n");
		return;
	}
	printf("Initial Error %8.5f, Final Err %8.5f In %i iterations Total Time %ims\n",FirstPc150(),LastPcl50(),(int)I.size(),TotTime());
	/*printf(" Med  Avg  RMS   StdDev   Time Tested Used  Dist Bord Angl \n");
	for(unsigned int qi=0;qi<I.size();++qi)
	printf("%5.2f (%6.3f:%6.3f) (%6.3f %6.3f %6.3f) %4ims %5i %5i %4i+%4i+%4i\n",
	I[qi].MinDistAbs,
	I[qi].pcl50, I[qi].pclhi,
	I[qi].AVG, I[qi].RMS, I[qi].StdDev ,
	IterTime(qi),
	I[qi].SampleTested,I[qi].SampleUsed,I[qi].DistanceDiscarded,I[qi].BorderDiscarded,I[qi].AngleDiscarded);*/

}

/*
This function is used to choose remove outliers after each ICP iteration.
All the points with a distance over the given Percentile are discarded.
It uses two parameters
MaxPointNum an (unused) hard limit on the number of points that are choosen
MinPointNum the minimum number of points that have to be chosen to be usable

*/
bool AlignPair::ChoosePoints( 	vector<Point3f> &Ps,		// corresponding vertices of src (red)
							 vector<Point3f> &Ns, 		// src of corresponding normal (red)
							 vector<Point3f> &Pt,		// vertices chosen on trg (green)
							 vector<Point3f> &OPt,		// vertices chosen on trg (green)
							 float PassHi,
							 Histogramf &H)
{
	const int N = ap.MaxPointNum; //100000
	float newmaxd = H.Percentile(PassHi);

	int sz = Ps.size();
	int fnd=0;
	int lastgood=sz-1;
	math::SubtractiveRingRNG myrnd; //random generator
	while(fnd<N && fnd<lastgood)
	{
		int index = fnd+myrnd.generate(lastgood-fnd);
		float dd=Distance(Ps[index],Pt[index]);
		if(dd<=newmaxd) //if this correspondence could be take, swap to the front
		{
			swap(Ps[index],Ps[fnd]);
			swap(Ns[index],Ns[fnd]);
			swap(Pt[index],Pt[fnd]);
			swap(OPt[index],OPt[fnd]);
			++fnd;
		}
		else //if this correspondence could not be take, swap the last value to the index pos and short the list to be test
		{
			swap(Ps[index],Ps[lastgood]);
			swap(Ns[index],Ns[lastgood]);
			swap(Pt[index],Pt[lastgood]);
			swap(OPt[index],OPt[lastgood]);
			lastgood--;
		}
	}
	Ps.resize(fnd);
	Ns.resize(fnd);
	Pt.resize(fnd);
	OPt.resize(fnd);
	//printf("Choices %i f the couples between the initial %i , iscarded those with dist > %f\n",fnd,sz,newmaxd);

	if( (int)Ps.size()<ap.MinPointNum )		{ //if finally we got less than 30 pairs failure
		printf("Too few points!\n");
		Ps.clear();
		Ns.clear();
		Pt.clear();
		OPt.clear();
		return false;
	}
	return true;
}

/*
Function call from the Align at each cycle
Fills carriers <MovVert> and <MovNorm> with the coordinates and normal vector of vertices taken from mov
the mesh to move transformed according to the matrix <In>
Calculate the new bounding box of these summits transformed.
*/
bool AlignPair::InitMov(
	vector< Point3f > &MovVert,
	vector< Point3f > &MovNorm,
	vector< Color4b > &Movcol,
	Box3f &trgbox,
	const Matrix44<float> &in	)				// trasformazione Iniziale (che porta i punti di trg su src)
{
	Point3f pp,nn;
	Color4b cc;
	MovVert.clear();
	MovNorm.clear();
	Movcol.clear();
	trgbox.SetNull();

	A2Mesh::VertexIterator vi;
	for(vi=mov->begin(); vi!=mov->end(); vi++) {
		pp=in*(*vi).P();
		nn=in*Point3f((*vi).P()+(*vi).N())-pp;
		nn.Normalize();
		cc = (*vi).C();
		MovVert.push_back(pp);
		MovNorm.push_back(nn);
		Movcol.push_back(cc);
		trgbox.Add(pp);
	}
	return true;
}

bool AlignPair::InitFixVert(AlignPair::A2Mesh *fm,
							AlignPair::Param &pp,
							A2GridVert &u, //typedef GridStaticPtr<A2Mesh::VertexType, float > A2GridVert;
							int PreferredGridSize)
{
	Box3f bb2=fm->bbox;
	float MinDist= pp.MinDistAbs*1.1;
	//the bbox of the grid should be enflated of the mindist used in the ICP search
	bb2.Offset(Point3f(MinDist,MinDist,MinDist));

	u.SetBBox(bb2);
	//I enter the src in the grid
	if(PreferredGridSize==0) PreferredGridSize=fm->vert.size()*pp.UGExpansionFactor;
	u.Set(fm->vert.begin(), fm->vert.end());//, PreferredGridSize);
	printf("UG %i %i %i\n",u.siz[0],u.siz[1],u.siz[2]);
	return true;
}


bool AlignPair::InitFix(AlignPair::A2Mesh *fm,
						AlignPair::Param &pp,
						A2Grid &u, //typedef GridStaticPtr<FaceType, float > A2Grid;
						int PreferredGridSize)
{
	tri::InitFaceIMark(*fm); //Initialize the imark-system of the faces.

	Box3f bb2=fm->bbox;
	float MinDist= pp.MinDistAbs*1.1; //MinDistAbs = 10
	//swelling of the distance the user BBox the second mesh
	bb2.Offset(Point3f(MinDist,MinDist,MinDist));
	u.SetBBox(bb2);
	//I enter the src in the grid
	if(PreferredGridSize==0) PreferredGridSize=fm->face.size()*pp.UGExpansionFactor;
	u.Set(fm->face.begin(), fm->face.end(), PreferredGridSize);
	printf("UG %i %i %i\n",u.siz[0],u.siz[1],u.siz[2]);
	return true;
}


/*
The Main ICP alignment Function:
It assumes that:
we have two meshes:
- Fix the mesh that does not move and stays in the spatial indexing structure.
- Mov the mesh we 'move' e.g. the one for which we search the transforamtion.

requires normalize normals for vertexes AND faces
Allinea due mesh;
Assume che:
la uniform grid sia gia' inizializzata con la mesh fix
*/

bool AlignPair::Align(
	A2Grid &u,
	A2GridVert &uv,
	const	Matrix44<float> &in,		// trasformazione Iniziale (leading points mov on fix)
	Matrix44<float> &out,				// calculated transformation
	vector<Point3f> &Pfix,		// corresponding vertices of src (red)
	vector<Point3f> &Nfix, 		// src of corresponding normal (red)
	vector<Point3f> &OPmov,		// vertexices chosen on trg (green) before processing input (Original Point Target)
	vector<Point3f> &ONmov, 	// normal chosen on trg (green)
	Histogramf &H,
	AlignPair::Stat &as)
{
	vector<char> beyondCntVec;    // carrier to mark movvert that definitely should not be used every time a summit is over max distance dist is incremented its counter; the movvert that were discarded more 'than MaxCntDist times do not look at the most;
	const int maxBeyondCnt=3;
	vector< Point3f > movvert;
	vector< Point3f > movnorm;
	vector< Color4b > movcol;
	vector<Point3f> Pmov; // vertices chosen after the initial Transfer
	status=SUCCESS;
	int tt0=clock();
	out=in;
	int i;
	float CosAngleThr=cos(ap.MaxAngleRad);
	float StartMinDist=ap.MinDistAbs;
	int tt1=clock();
	int ttsearch=0;
	int ttleast=0;
	int nc=0;
	as.clear();
	as.StartTime=clock();

	beyondCntVec.resize(mov->size(),0);

	/**************** BEGIN ICP LOOP ****************/
	do
	{
		Stat::IterInfo ii;
		Box3f movbox;
		InitMov(movvert,movnorm,movcol, movbox, out);
		H.SetRange(0,StartMinDist,512,2.5);
		Pfix.clear(); 
		Nfix.clear();
		Pmov.clear();
		OPmov.clear();
		ONmov.clear();
		int tts0=clock();
		ii.MinDistAbs=StartMinDist;
		int LocSampleNum=min(ap.SampleNum,int(movvert.size()));
		//int LocSampleNum=min(10000,int(movvert.size()));
		Box3f fixbox;
		if(u.Empty()) fixbox = uv.bbox;
		else fixbox = u.bbox;
		//int count_out = 0;

		for(i=0;i<LocSampleNum;++i) // for each sample point in mov
		{
			if( beyondCntVec[i] < maxBeyondCnt )
			{
				if(! fixbox.IsIn(movvert[i]) )
					beyondCntVec[i]=maxBeyondCnt+1;
			}
			else 
			{
				//count_out++;
				continue;
			}

			float error=StartMinDist;
			Point3f closestPoint, closestNormal;
			float maxd= StartMinDist;
			ii.SampleTested++;
			if(u.Empty()) // using the point cloud grid
			{
				A2Mesh::VertexPointer vp = tri::GetClosestVertex(*fix,uv,movvert[i], maxd, error);		//here we calculate out the error

				//find the cloest vertex of point movvert[i]
				if(error>=StartMinDist) {  //reject by distance
					ii.DistanceDiscarded++; ++beyondCntVec[i]; continue;
				}
				if(movnorm[i].dot(vp->N()) < CosAngleThr) { //reject by normal angle
					ii.AngleDiscarded++; continue;
				}

				closestPoint=vp->P();
				closestNormal=vp->N();
			}
			else // using the standard faces and grid
			{
				A2Mesh::FacePointer f=vcg::tri::GetClosestFaceBase<vcg::AlignPair::A2Mesh, vcg::AlignPair::A2Grid >(*fix, u, movvert[i], maxd, error, closestPoint); //find the cloest vertex of point movvert[i]
				if(error>=StartMinDist) { //reject by distance
					ii.DistanceDiscarded++; ++beyondCntVec[i]; continue;
				}
				float angle = movnorm[i].dot(f->N());
				if( angle < CosAngleThr) { //reject by normal angle
					ii.AngleDiscarded++; continue;
				}

				Point3f ip;
				InterpolationParameters<A2Face,float>(*f,f->N(),closestPoint, ip);
				const float IP_EPS = 0.00001;
				// If ip[i] == 0 it means that we are on the edge opposite to i
				if(	(fabs(ip[0])<=IP_EPS && f->IsB(1)) ||  (fabs(ip[1])<=IP_EPS && f->IsB(2)) || (fabs(ip[2])<=IP_EPS && f->IsB(0))   ){ //reject by boarder edge
					ii.BorderDiscarded++;  continue;
				}

				if(ap.Threshold_Color < 441)
				{
					float col_d = (float)(movcol[i] - f->V(0)->C()).Norm();
					if( col_d > ap.Threshold_Color){ //every channel have 100 var
						ii.ColorDiscarded++; continue;
					}
				}
				closestNormal = f->N();
			}
			// The sample was accepted. Store it.
			Pmov.push_back(movvert[i]);
			OPmov.push_back((*mov)[i].P());
			ONmov.push_back((*mov)[i].N());
			Nfix.push_back( closestNormal );
			Pfix.push_back( closestPoint );
			H.Add(float(error));	//add up the error
			ii.SampleUsed++;		//the count of uesable sample++

		} // End for each pmov

		//printf("%i ",count_out);

		int tts1=clock();

		if(!ChoosePoints(Pfix,Nfix,Pmov,OPmov,ap.PassHiFilter,H))
		{
			if(int(Pfix.size())<ap.MinPointNum)
			{
				status = TOO_FEW_POINTS;
				ii.Time=clock();
				as.I.push_back(ii);
				return false;
			}
		}
		Matrix44<float> newout;
		switch(ap.MatchMode) {
			//case AlignPair::Param::MMSimilarity : ComputeRotoTranslationScalingMatchMatrix(newout,Pfix,OPmov); break;
		case AlignPair::Param::MMRigid   : ComputeRigidMatchMatrix(Pfix,OPmov,newout);   break;
		default :
			status = UNKNOWN_MODE;
			ii.Time=clock();
			as.I.push_back(ii);
			return false;
		}
		// then use the same pass as soon as this initial transformation found.
		// In the next cycle it is part of this matrix as initial.
		out=newout;

		assert(Pfix.size()==Pmov.size());
		int tts2=clock();
		ttsearch+=tts1-tts0;
		ttleast +=tts2-tts1;
		ii.pcl50=H.Percentile(.5);
		ii.pclhi=H.Percentile(ap.PassHiFilter);
		ii.AVG=H.Avg();
		ii.RMS=H.RMS();
		ii.StdDev=H.StandardDeviation();
		ii.Time=clock();
		as.I.push_back(ii);
		nc++;
		// The distance of the next points to be considered is lowered according to the <ReduceFactor> parameter.
		// We use 5 times the <ReduceFactor> percentile of the found points.
		if(ap.ReduceFactorPerc<1) StartMinDist=max(ap.MinDistAbs*ap.MinMinDistPerc, min(StartMinDist,5.0*H.Percentile(ap.ReduceFactorPerc)));
	}
	while (
		nc<=ap.MaxIterNum &&
		H.Percentile(.5) > ap.TrgDistAbs &&
		(nc<ap.EndStepNum+1 || ! as.Stable(ap.EndStepNum) )
		);
	/**************** END ICP LOOP ****************/
	int tt2=clock();
	Matrix44<float> ResCopy=out;
	Point3f scv,shv,rtv,trv;
	Decompose(ResCopy,scv,shv,rtv,trv);
	if((ap.MatchMode==vcg::AlignPair::Param::MMRigid) && (math::Abs(1-scv[0])>ap.MaxScale || math::Abs(1-scv[1])>ap.MaxScale || math::Abs(1-scv[2])>ap.MaxScale) ) {
		status = TOO_MUCH_SCALE;
		return false;
	}
	if(shv[0]>ap.MaxShear || shv[1]>ap.MaxShear || shv[2]>ap.MaxShear ) {
		status = TOO_MUCH_SHEAR;
		return false;
	}
	printf("Grid %i %i %i - fn %i\n",u.siz[0],u.siz[1],u.siz[2],fix->fn);
	printf("Time: Init %8.3f Loop %8.3f Search %8.3f least sqrt %8.3f\n",
		float(tt1-tt0)/CLOCKS_PER_SEC, float(tt2-tt1)/CLOCKS_PER_SEC,
		float(ttsearch)/CLOCKS_PER_SEC,float(ttleast)/CLOCKS_PER_SEC );

	return true;
}

const char *AlignPair::ErrorMsg( ErrorCode code)
{
	switch(code){
	case SUCCESS:         return "Success         ";
	case NO_COMMON_BBOX : return "No Common BBox  ";
	case TOO_FEW_POINTS : return "Too few points  ";
	case LSQ_DIVERGE    : return "LSQ not converge";
	case TOO_MUCH_SHEAR : return "Too much shear  ";
	case TOO_MUCH_SCALE : return "Too much scale  ";
	case UNKNOWN_MODE   : return "Unknown mode    ";
	default :  assert(0); return "Catastrophic Error";
	}
	return 0;
}

/**********************************************************/
// Functions for the choice of the vertices on the mesh to move
//choice function
bool AlignPair::SampleMovVert(vector<A2Vertex> &vert, int SampleNum, AlignPair::Param::SampleModeEnum SampleMode)
{
	switch(SampleMode)
	{
	case AlignPair::Param::SMRandom :			return SampleMovVertRandom(vert,SampleNum);
	case AlignPair::Param::SMNormalEqualized :	return SampleMovVertNormalEqualized(vert,SampleNum);
	default: assert(0);
	}
	return false;
}

// Function to retrieve a static random number generator object.
static math::SubtractiveRingRNG &LocRnd(){
	static math::SubtractiveRingRNG myrnd(time(NULL));
	return myrnd;
}

// Gets a random number in the interval [0..n].
static int LocRnd(int n){
	return LocRnd().generate(n);
}

// Choosing a simple case
bool AlignPair::SampleMovVertRandom(vector<A2Vertex> &vert, int SampleNum)
{
	if(int(vert.size())<=SampleNum) return true;
	int i;
	for(i=0;i<SampleNum;++i)
	{
		int pos=LocRnd(vert.size());
		assert(pos>=0 && pos < int(vert.size()));
		swap(vert[i],vert[pos]);
	}
	vert.resize(SampleNum);
	return true;
}

/*
Randomly chosen in such a way that the distribution of the normals of the
points chosen is the most possible uniform. In this way even small
inclined parts are definitely sampled

Precompute is a small (42) set of normal and do bucketing of
all the vertices of the mesh on them.e.
Then choose <SampleNum> points every time before choosing a bucket
and then a point inside the bucket
*/

bool AlignPair::SampleMovVertNormalEqualized(vector<A2Vertex> &vert, int SampleNum)
{
	static vector<Point3f> NV;
	if(NV.size()==0)
	{
		GenNormal<float>::Uniform(30,NV); //uniformaly generate 30 directions for normal to classify the input normal int bkt
		printf("Generated %i normals\n",int(NV.size()));
	}
	// Bucket vector where, for each normal put indexes
	// vertex corresponding thereto
	vector<vector <int> > BKT(NV.size());
	for(size_t i=0;i<vert.size();++i) // go through the how vertex to classify the vertex normal
	{
		int ind=GenNormal<float>::BestMatchingNormal(vert[i].N(),NV);
		BKT[ind].push_back(i);
	}
	// vector of counters to see how many points I have already 'taken for each bucket
	vector <int> BKTpos(BKT.size(),0);

	if(SampleNum >= int(vert.size())) SampleNum= vert.size()-1;

	for(int i=0;i<SampleNum;)
	{
		int ind=LocRnd(BKT.size()); // I turn to a Bucket
		int &CURpos = BKTpos[ind];
		vector<int> &CUR = BKT[ind];

		if(CURpos<int(CUR.size()))
		{
			swap(CUR[CURpos], CUR[ CURpos + LocRnd(BKT[ind].size()-CURpos)]);
			swap(vert[i],vert[CUR[CURpos]]);
			++BKTpos[ind];
			++i;
		}
	}
	vert.resize(SampleNum);

	return true;
}
#endif
