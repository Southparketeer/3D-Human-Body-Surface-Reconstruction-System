#pragma once
#include <list>
#include <queue>
#include <wrap/callback.h>
#include "ICP/AlignPair.h"

using namespace std;
namespace vcg
{
	class AlignGlobal
	{
	public:
		class Node;

		// Virtual alignment between two mesh (excerpt from a alignresult)
		// Important Note: the transformation and points stored here are net of the radical base of the two mesh into play.
		// So if someone moves a mesh the pos of the points are still valid but it does not apply to the Transfer.
		class VirtAlign
		{
		public:
			Node *Fix, *Mov; // alignment between i and j
			std::vector<Point3f> FixP; // points on Fix
			std::vector<Point3f> MovP; // points on Mov
			std::vector<Point3f> FixN; // Normal on Fix
			std::vector<Point3f> MovN; // Normal on Mov
			Matrix44f M2F; //the matrix to be applied to the points of Mov to obtain those on Fix
			Matrix44f F2M; //the matrix to be applied to the points of Fix to obtain those on Mov

			/*
			In the simplified case that the mesh had as Transfer of the basic identity must be true:
			N2A(N).Apply(   P(N)) ~= AdjP(N)
			A2N(N).Apply(AdjP(N)) ~=    P(N)
			In general, a node N VirtAlign any of that is true:
			N2A(N).Apply(       N->M.Apply(   P(N)) ) ~= AdjN(N)->M.Apply( AdjP(N) );
			A2M(N).Apply( AdjN(N)->M.Apply(AdjP(N)) ) ~=       N->M.Apply(    P(N) );
			on which il ~= It means equal net alignment error.
			To get virtualmate related to a node n:
			*/

			Node *Adj(Node *n);

			inline Matrix44f       &N2A(Node *n) {if(n==Fix) return F2M; else return M2F;}
			inline Matrix44f       &A2N(Node *n) {if(n==Fix) return M2F; else return F2M;}

			inline std::vector<Point3f>    &P(Node *n) {if(n==Fix) return FixP; else return MovP;}
			inline std::vector<Point3f>    &N(Node *n) {if(n==Fix) return FixN; else return MovN;}

			inline std::vector<Point3f> &AdjP(Node *n) {if(n==Fix) return MovP; else return FixP;}
			inline std::vector<Point3f> &AdjN(Node *n) {if(n==Fix) return MovN; else return FixN;}
			bool Check();
		};


		class Node
		{
		public:
			Node(){id=-1;Active=false;Discarded=false;Queued=false;}

			int id;  // id of the mesh which corresponds to the node
			int sid; // Subgraph id;
			Matrix44f M; // The matrix that puts the mesh in its basic position
			std::list<VirtAlign *> Adj;

			bool Active;  //true if a node in Active // false if and dormant;
			bool Queued;
			bool Discarded;
			// Align a node with all its neighbors
			double AlignWithActiveAdj(bool Rigid);
			double MatrixNorm(Matrix44f &NewM) const;
			double MatrixBoxNorm(Matrix44f &NewM,Box3f &bb) const;
			int PushBackActiveAdj(std::queue<Node *>	&Q);
			int DormantAdjNum();
			int ActiveAdjNum();
		};

		// auxiliary class to store the connected components of the graph
		class SubGraphInfo
		{
		public:
			int sid;
			int size;
			Node *root;
		};

		Node *ChooseDormantWithMostDormantLink ();
		Node *ChooseDormantWithMostActiveLink  ();
		void MakeAllDormant();
		void Clear();
		bool GlobalAlign(const std::map<int,std::string> &Names, 	const double epsilon, int maxiter, bool Rigid, FILE *elfp=0, CallBack * cb=DummyCallBack );

		bool CheckGraph();

		// Data members:
		std::list<Node> N;
		std::list<VirtAlign *> A;
		std::list<SubGraphInfo> CC; // Descriptors of connected components, filled by ComputeConnectedComponents


		int DormantNum();
		int ActiveNum();
		void Dump(FILE *fp);

		int ComputeConnectedComponents();
		void BuildGraph(std::vector<AlignPair::Result *> &Res, std::vector<Matrix44f> &Tr, std::vector<int> &Id);
		bool GetMatrixVector(std::vector<Matrix44f> &Tr, std::vector<int> &Id);
	};

	inline void LOG( FILE *fp, const char * f, ... )
	{
		if(fp==0) return;

		va_list marker;
		va_start( marker, f );
		vfprintf(fp,f,marker);
		va_end( marker );
		fflush(fp);
	}

	int AlignGlobal::ComputeConnectedComponents()
	{
		//printf("Building Connected Components on a graph with %lu nodes and %lu arcs\n",N.size(),A.size());

		CC.clear();
		list<AlignGlobal::Node>::iterator li;

		stack<AlignGlobal::Node *> ToReach; // nodes still to visit
		stack<AlignGlobal::Node *> st;      // nodes that you are visiting

		for(li=N.begin();li!=N.end();++li)
		{
			(*li).sid=-1;
			ToReach.push(&*li);
		}

		int cnt=0;

		while(!ToReach.empty())
		{
			SubGraphInfo sg;
			st.push(&*ToReach.top());
			ToReach.pop();
			assert(st.top()->sid==-1);
			sg.root=st.top();
			sg.sid=cnt;
			sg.size=0;
			st.top()->sid=cnt;
			while(!st.empty())
			{
				AlignGlobal::Node *cur=st.top();
				st.pop();
				++sg.size;
				assert(cur->sid==cnt);

				list<VirtAlign *>::iterator li;
				for(li=cur->Adj.begin();li!=cur->Adj.end();++li)
					if((*li)->Adj(cur)->sid==-1) {
						(*li)->Adj(cur)->sid=cnt;
						st.push((*li)->Adj(cur));
					} else assert((*li)->Adj(cur)->sid==cnt);
			}
			cnt++;
			CC.push_back(sg);
			while(!ToReach.empty() && ToReach.top()->sid!=-1) ToReach.pop();
		}

		return cnt;
	}

	////////////////////////////////////////////////////////////////////////////////
	// the two matrices M2F and F2M equate points supposed to be applied to them
	// the transf own node to which they belong (Mov->M e Fix->M)
	// Check that the transformation matrices actually lead the points chosen to coincide.
	//
	bool AlignGlobal::VirtAlign::Check()
	{
		int i;
		if(FixP.size()!=MovP.size()) return false;
		Point3f mp,fp;
		double md=0,fd=0;
		double md2=0,fd2=0;
		Matrix44f &MovTr=Mov->M;
		Matrix44f &FixTr=Fix->M;
		for(i=0;i<FixP.size();++i)
		{
			mp=MovTr*MovP[i];
			fp=FixTr*FixP[i];

			md +=       Distance(fp,M2F*mp);
			md2+=SquaredDistance(fp,M2F*mp);

			fd +=       Distance(mp,F2M*fp);
			fd2+=SquaredDistance(mp,F2M*fp);
		}
		int nn=MovP.size();

		//printf("Arc %3i -> %3i : %i pt\n",Fix->id,Mov->id,nn);
		//printf("SquaredSum Distance %7.3f %7.3f Avg %7.3f %7.3f\n",fd2, md2, fd2/nn, md2/nn);
		//printf("       Sum Distance %7.3f %7.3f Avg %7.3f %7.3f\n",fd , md ,  fd/nn, md/nn);
		return true;
	}


	AlignGlobal::Node *AlignGlobal::VirtAlign::Adj(AlignGlobal::Node *n)
	{
		assert(n==Fix || n==Mov);
		if(n==Fix) return Mov;
		else return Fix;
	}
	void AlignGlobal::MakeAllDormant()
	{
		list<AlignGlobal::Node>::iterator li;
		for(li=N.begin();li!=N.end();++li)
			(*li).Active=false;
	}
	void AlignGlobal::Dump(FILE *elfp)
	{
		fprintf(elfp,"Alignment Graph of %lu nodes and %lu arcs\n",N.size(),A.size());
	}

	//Check that each node is reachable from the root
	bool AlignGlobal::CheckGraph()
	{
		vector<bool> Visited(N.size(),false);
		stack<AlignGlobal::Node *> st;
		st.push(&(*N.begin()));
		while(!st.empty())
		{
			AlignGlobal::Node *cur=st.top();
			st.pop();
			list<VirtAlign *>::iterator li;
			for(li=cur->Adj.begin();li!=cur->Adj.end();++li)
				if(!Visited[(*li)->Adj(cur)->id]) {
					Visited[(*li)->Adj(cur)->id]=true;
					st.push((*li)->Adj(cur));
				}
		}
		int cnt=count(Visited.begin(),Visited.end(),true);
		printf("Nodes that can be reached from root %i on %i \n",cnt,N.size());
		return cnt==N.size();
	}

	int AlignGlobal::DormantNum()
	{
		int cnt=0;
		list<AlignGlobal::Node>::iterator li;
		for(li=N.begin();li!=N.end();++li)
			if(!(*li).Active) ++cnt;

		return cnt;
	}

	int AlignGlobal::ActiveNum()
	{
		int cnt=0;
		list<AlignGlobal::Node>::iterator li;
		for(li=N.begin();li!=N.end();++li)
			if((*li).Active) ++cnt;

		return cnt;
	}

	AlignGlobal::Node *AlignGlobal::ChooseDormantWithMostDormantLink ()
	{
		AlignGlobal::Node *BestNode=0;
		int MaxAdjNum=0;
		list<AlignGlobal::Node>::iterator li;
		for(li=N.begin();li!=N.end();++li)
			if(!(*li).Active)
			{
				int AdjNum = (*li).DormantAdjNum();
				if(AdjNum>MaxAdjNum)
				{
					MaxAdjNum=AdjNum;
					BestNode=&(*li);
				}
			}
			assert(BestNode);
			assert(!BestNode->Queued);
			assert(!BestNode->Active);
			return BestNode;
	}

	AlignGlobal::Node *AlignGlobal::ChooseDormantWithMostActiveLink ()
	{
		AlignGlobal::Node *BestNode=0;
		int MaxAdjNum=0;
		list<AlignGlobal::Node>::iterator li;
		for(li=N.begin();li!=N.end();++li)
			if(!(*li).Active)
			{
				int AdjNum = (*li).ActiveAdjNum();
				if(AdjNum>MaxAdjNum)
				{
					MaxAdjNum=AdjNum;
					BestNode=&(*li);
				}
			}
			if(!BestNode)
			{   // We finished arranging this connected component.
				//printf("Warning! Unable to find a Node with at least an active link!!\n");
				return 0;
			}
			assert(BestNode);
			assert(!BestNode->Queued);
			assert(!BestNode->Active);
			return BestNode;
	}

	int AlignGlobal::Node::PushBackActiveAdj(queue<AlignGlobal::Node *>	&Q)
	{
		int cnt=0;
		assert(Active);
		AlignGlobal::Node *pt;
		list<VirtAlign *>::iterator li;
		for(li=Adj.begin();li!=Adj.end();++li)
		{
			pt=(*li)->Adj(this);
			if(pt->Active && !pt->Queued)
			{
				++cnt;
				pt->Queued=true;
				Q.push(pt);
			}
		}
		return cnt;
	}

	int AlignGlobal::Node::ActiveAdjNum ()
	{
		int cnt=0;
		list<VirtAlign *>::iterator li;
		for(li=Adj.begin();li!=Adj.end();++li)
			if((*li)->Adj(this)->Active) ++cnt;
		return cnt;
	}

	int AlignGlobal::Node::DormantAdjNum ()
	{
		int cnt=0;
		list<VirtAlign *>::iterator li;
		for(li=Adj.begin();li!=Adj.end();++li)
			if(!(*li)->Adj(this)->Active) ++cnt;
		return cnt;
	}


	/******************************
	Date a mesh with a basic position, changes its pos base in such a way that
	It is aligned with all the points of the adjacent active;
	In practice the same node and mesh mov, while everyone around between Y ear a kind of virtual fix
	At the end and 'it changed the matrix M of the node and all arrays of arrays virtual
	******************************/
	double AlignGlobal::Node::AlignWithActiveAdj(bool Rigid)
	{
		list<VirtAlign *>::iterator li;

		//printf("--- AlignWithActiveAdj --- \nMoving node %i with respect to nodes:",id);
		//for(li=Adj.begin();li!=Adj.end();++li) if((*li)->Adj(this)->Active) printf(" %i,",(*li)->Adj(this)->id);
		//printf("\n");

		// Step 1; We build two lists of points to align
		vector<Point3f> FixP,MovP, FixN,MovN;
		Box3f FixBox,MovBox;FixBox.SetNull();MovBox.SetNull();
		for(li=Adj.begin();li!=Adj.end();++li)
			if((*li)->Adj(this)->Active) // skim all nodes adjacent active
			{
				vector<Point3f> &AP=(*li)->AdjP(this);   // Points on the adjacent node current;
				vector<Point3f> &AN=(*li)->AdjN(this);   // Normal node adjacent current;

				Point3f pf,nf;
				Point3f pm;
				for(int i=0;i<AP.size();++i)
				{
					pf=(*li)->Adj(this)->M*AP[i]; // fixed points are those on the adjacent sup put in their current pos
					FixP.push_back(pf);
					FixBox.Add(pf);
					nf=(*li)->Adj(this)->M*Point3f(AP[i]+AN[i])-pf;
					nf.Normalize();
					FixN.push_back(nf);

					pm=(*li)->A2N(this)*pf;
					MovP.push_back(pm); // the points that move are those on the adj processed so as to fall on the node corr.
					MovBox.Add(pm);
				}
			}
			Matrix44f out;
			if(Rigid) ComputeRigidMatchMatrix(FixP,MovP,out);
			//else ComputeRotoTranslationScalingMatchMatrix(out,FixP,MovP);

			Matrix44f outIn=vcg::Inverse(out);
			double maxdiff = MatrixBoxNorm(out,FixBox);

			// The matrix out calculated and that which applied to the points MOVP leads them on FixP, then the points of the mesh current
			// The new position of the base of the mesh becomes then
			// M * out
			// In fact, if I consider a point on the original mesh apply ourselves the means to make new matricie
			// p * M * out

			M=out*M;

			// as the last step necessary to apply the matrix found in all the alignments in the game.
			for(li=Adj.begin();li!=Adj.end();++li)// skim all nodes adjacent active
			{
				(*li)->N2A(this)=(*li)->N2A(this)*outIn;
				(*li)->A2N(this)=(*li)->A2N(this)*out  ;
			}
			return maxdiff;
	}


	// Nuova Norma per matrici:
	// restituisce la max spostamento che si puo' ottenere se si
	// applica tale trasf a un insieme di punti contenuto nel box bb

	double AlignGlobal::Node::MatrixBoxNorm(Matrix44f &NewM,Box3f &bb) const
	{
		double maxdiff=0;
		Point3f pt;

		pt=Point3f(bb.min[0],bb.min[1],bb.min[2]); maxdiff=max(maxdiff,Distance(pt,NewM*pt));
		pt=Point3f(bb.max[0],bb.min[1],bb.min[2]); maxdiff=max(maxdiff,Distance(pt,NewM*pt));
		pt=Point3f(bb.min[0],bb.max[1],bb.min[2]); maxdiff=max(maxdiff,Distance(pt,NewM*pt));
		pt=Point3f(bb.max[0],bb.max[1],bb.min[2]); maxdiff=max(maxdiff,Distance(pt,NewM*pt));
		pt=Point3f(bb.min[0],bb.min[1],bb.max[2]); maxdiff=max(maxdiff,Distance(pt,NewM*pt));
		pt=Point3f(bb.max[0],bb.min[1],bb.max[2]); maxdiff=max(maxdiff,Distance(pt,NewM*pt));
		pt=Point3f(bb.min[0],bb.max[1],bb.max[2]); maxdiff=max(maxdiff,Distance(pt,NewM*pt));
		pt=Point3f(bb.max[0],bb.max[1],bb.max[2]); maxdiff=max(maxdiff,Distance(pt,NewM*pt));
		return maxdiff;
	}

	double AlignGlobal::Node::MatrixNorm(Matrix44f &NewM) const
	{
		double maxdiff=0;
		Matrix44f diff;
		diff.SetIdentity();
		diff=diff-NewM;
		for(int i=0;i<4;++i)
			for(int j=0;j<4;++j)
				maxdiff+=(diff[i][j]*diff[i][j]);
		return maxdiff;
	}

	void AlignGlobal::Clear()
	{

		list<VirtAlign *>::iterator li;
		for(li=A.begin();li != A.end();++li)
			delete(*li);

		N.clear();
		A.clear();

	}


	/******************************
	For each connected component,
	it starts from the node with the most adjacent
	Get active
	do you choose between
	******************************/

	bool AlignGlobal::GlobalAlign(const std::map<int,string> &Names, 	const double epsilon, int maxiter, bool Rigid, FILE *elfp, CallBack* cb )
	{
		double change;
		int step, localmaxiter;
		cb("Global Alignment...");
		LOG(elfp,"----------------\n----------------\nGlobalAlignment (target eps %7.3f)\n",epsilon);

		queue<AlignGlobal::Node *>	Q;
		MakeAllDormant();
		AlignGlobal::Node *curr=ChooseDormantWithMostDormantLink();
		curr->Active=true;
		int cursid=curr->sid;
		LOG(elfp,"Root node %i '%s' with %i dormant link\n", curr->id, Names.find(curr->id)->second.c_str(),curr->DormantAdjNum());

		while(DormantNum()>0)
		{
			LOG(elfp,"---------\nGlobalAlignment loop DormantNum = %i\n",DormantNum());

			curr=ChooseDormantWithMostActiveLink ();
			if(!curr) {
				// the connected component and over and you move to the next looking for a dormant with all dormant.
				LOG(elfp,"\nCompleted Connected Component %i\n",cursid);
				LOG(elfp,"\nDormant Num: %i\n",DormantNum());

				curr=ChooseDormantWithMostDormantLink ();
				if(curr==0) {
					LOG(elfp,"\nFailed ChooseDormantWithMostDormantLink, choosen id:%i\n" ,0);
					break; // there are no more connected components consist of more than a single mesh.
				}
				else LOG(elfp,"\nCompleted ChooseDormantWithMostDormantLink, choosen id:%i\n" ,curr->id);
				curr->Active=true;
				cursid=curr->sid;
				curr=ChooseDormantWithMostActiveLink ();
				if(curr==0) LOG(elfp,"\nFailed    ChooseDormantWithMostActiveLink, choosen id:%i\n" ,0);
				else  LOG(elfp,"\nCompleted ChooseDormantWithMostActiveLink, choosen id:%i\n" ,curr->id);
			}

			LOG(elfp,"\nAdded node %i '%s' with %i/%i Active link\n",curr->id,Names.find(curr->id)->second.c_str(),curr->ActiveAdjNum(),curr->Adj.size());
			curr->Active=true;
			curr->Queued=true;
			localmaxiter=ActiveNum()*10;  // It is supposed, by eye, to rearrange that a set of n mesh serve to 10n more steps;
			Q.push(curr);
			step=0;
			// inner loop alignment
			while(!Q.empty())
			{
				curr=Q.front();
				Q.pop();
				curr->Queued=false;
				change=curr->AlignWithActiveAdj(Rigid);
				step++;
				LOG(elfp,"     Step %5i Queue size %5i Moved %4i  err %10.4f\n",step,Q.size(),curr->id,change);
				if(change>epsilon)
				{
					curr->PushBackActiveAdj(Q);
					LOG(elfp,"         Large Change pushing back active nodes adj to %i to Q (new size %i)\n",curr->id,Q.size());
					if(change>epsilon*1000)  printf("Large Change Warning\n\n");
				}
				if(step>localmaxiter) return false;
				if(step>maxiter) return false;
			}
		}
		if(!curr)
		{
			LOG(elfp,"Alignment failed for %i meshes:\n",DormantNum());
			list<AlignGlobal::Node>::iterator li;

			for(li=N.begin();li!=N.end();++li)
				if(!(*li).Active){
					(*li).Discarded=true;
					LOG(elfp,"%5i\n",(*li).id);
				}
		}
		LOG(elfp,"Completed Alignment in %i steps with error %f\n",step,epsilon);
		return true;
	}

	// fills an array of matrices with matrices result global alignment.
	// Carrier says Id like mesh take.
	bool AlignGlobal::GetMatrixVector(std::vector<Matrix44f> &Tr, std::vector<int> &Id)
	{
		std::list<Node>::iterator li;
		Tr.clear();
		map<int,AlignGlobal::Node *> Id2N;
		for(li=N.begin();li!=N.end();++li)
			Id2N[(*li).id]=&*li;

		Tr.resize(Id.size());
		for(int i=0;i<Id.size();++i)
		{
			if( Id2N[Id[i]] ==0 ) return false;
			Tr[i]=Id2N[Id[i]]->M;
		}
		return false;
	}


	//Build the Alignment Graphs starting from the vector of Results and from the vector of the matrix with the current starting positions.
	void AlignGlobal::BuildGraph(std::vector<AlignPair::Result *> &Res, vector<Matrix44f> &Tr, vector<int> &Id)
	{
		Clear();
		// it is assumed that the matrix Tr[i] is relative to a node with id Id[i];
		int i,mn=Tr.size();

		AlignGlobal::Node rgn;
		rgn.Active=false;
		rgn.Queued=false;
		rgn.Discarded=false;
		map<int,AlignGlobal::Node *> Id2N;
		map<int,int> Id2I;
		for(i=0;i<mn;++i)
		{
			rgn.id=Id[i];
			rgn.M=Tr[i];
			N.push_back(rgn);
			Id2N[rgn.id]=&(N.back());
			Id2I[rgn.id]=i;
		}

		//printf("building %i graph arcs\n",Res.size());
		VirtAlign *tv;

		// Main loop where you build various arches
		// It is assumed that the result be made in the reference system of matrices fix.

		vector<AlignPair::Result *>::iterator rii;
		for(rii=Res.begin();rii!=Res.end();++rii)
		{
			AlignPair::Result *ri= *rii;
			tv = new VirtAlign();
			tv->Fix=Id2N[(*ri).FixName];
			tv->Mov=Id2N[(*ri).MovName];
			tv->Fix->Adj.push_back(tv);
			tv->Mov->Adj.push_back(tv);
			tv->FixP=(*ri).Pfix;
			tv->MovP=(*ri).Pmov;
			tv->FixN=(*ri).Nfix;
			tv->MovN=(*ri).Nmov;

			/*
			are:
			Pf and Pm points on the mesh fix and mov in sist Ref original
			Pft and Pmt the points on the mesh fix and mov after the current transformer;
			Mf and Mm the transformer carrying the mesh fixes and mov in their current location;
			If and Im  Transfer the reverse of the above
			Vale:
			Pft = Mf * Pf and Pmt = Mm * Pm
			Pf = If * Pft and Pm = Im * Pmt

			Res * Pm = Pf;
			Res * Im * Pmt = If * Pft
			Mf * Res * Im * Pmt = Mf * If * Pft
			(Mf * Res * Im) * Pmt = Pft
			*/
			Matrix44f Mm=Tr[Id2I[(*ri).MovName]];
			Matrix44f Mf=Tr[Id2I[(*ri).FixName]];
			Matrix44f Im=Inverse(Mm);
			Matrix44f If=Inverse(Mf);

			Matrix44f NewTr = Mf * (*ri).Tr * Im; // --- orig

			tv->M2F=NewTr;
			tv->F2M=Inverse(NewTr);

			assert(tv->Check());
			A.push_back(tv);
		}

		ComputeConnectedComponents();
	}

} // end namespace

