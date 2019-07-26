#ifndef MESHMODEL_H
#define MESHMODEL_H
#include <GL/glew.h>

#include <stdio.h>
#include <time.h>
#include <map>

#include <vcg/complex/complex.h>

#include <vcg/simplex/face/topology.h>

#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/color.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/position.h>
#include <vcg/complex/algorithms/update/quality.h>
#include <vcg/complex/algorithms/update/selection.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/create/mc_trivial_walker.h>

#include <wrap/gl/trimesh.h>
#include <wrap/callback.h>
#include <wrap/io_trimesh/io_mask.h>
#include <wrap/io_trimesh/additionalinfo.h>

#include <list>
#include <String>
#include <wrap/gl/math.h>
#include <vcg/complex/append.h>
#include "Base.h"
#include "Helper.h"
//end namespace vcg
/*
MeshModel Class
The base class for representing a single mesh.
It contains a single vcg mesh object with some additional information for keeping track of its origin and of what info it has.
*/
class MeshModel
{
public: 
	/*
	This enum specify the various simplex components
	It is used in various parts of the framework:
	- to know what elements are currently active and therefore can be saved on a file
	- to know what elements are required by a filter and therefore should be made ready before starting the filter (e.g. if a
	- to know what elements are changed by a filter and therefore should be saved/restored in case of dynamic filters with a preview
	*/
	enum MeshElement{
		MM_NONE             = 0x00000000,
		MM_VERTCOORD        = 0x00000001,
		MM_VERTNORMAL       = 0x00000002,
		MM_VERTFLAG         = 0x00000004,
		MM_VERTCOLOR        = 0x00000008,
		MM_VERTQUALITY      = 0x00000010,
		MM_VERTMARK	        = 0x00000020,
		MM_VERTFACETOPO     = 0x00000040,
		MM_VERTCURV	        = 0x00000080,
		MM_VERTCURVDIR      = 0x00000100,
		MM_VERTRADIUS       = 0x00000200,
		MM_VERTTEXCOORD     = 0x00000400,
		MM_VERTNUMBER       = 0x00000800,

		MM_FACEVERT         = 0x00001000,
		MM_FACENORMAL       = 0x00002000,
		MM_FACEFLAG	        = 0x00004000,
		MM_FACECOLOR        = 0x00008000,
		MM_FACEQUALITY      = 0x00010000,
		MM_FACEMARK	        = 0x00020000,
		MM_FACEFACETOPO     = 0x00040000,
		MM_FACENUMBER       = 0x00080000,
		MM_FACECURVDIR      = 0x00100000,

		MM_WEDGTEXCOORD     = 0x00200000,
		MM_WEDGNORMAL       = 0x00400000,
		MM_WEDGCOLOR        = 0x00800000,

		// 	Selection
		MM_VERTFLAGSELECT   = 0x01000000,
		MM_FACEFLAGSELECT   = 0x02000000,

		// Per Mesh Stuff....
		MM_CAMERA			= 0x08000000,
		MM_TRANSFMATRIX     = 0x10000000,
		MM_COLOR            = 0x20000000,
		MM_POLYGONAL        = 0x40000000,
		MM_UNKNOWN          = 0x80000000,

		MM_ALL				= 0xffffffff
	};

	CMeshO cm;


public:
	/*
	Bitmask denoting what fields are currently used in the mesh
	it is composed by MeshElement enums.
	it should be changed by only mean the following functions:

	updateDataMask(neededStuff)
	clearDataMask(no_needed_stuff)
	hasDataMask(stuff)
	Note that if an element is active means that is also allocated
	Some unactive elements (vertex color) are usually already allocated
	other elements (FFAdj or curvature data) not necessarily.
	*/
private:
	int currentDataMask;
	std::string fullPathFileName;
	std::string _label;
	int _id;
	bool modified;

public:
	void Clear();
	void UpdateBoxAndNormals(); // This is the STANDARD method that you should call after changing coords.
	inline int id() const {return _id;}
	std::string label() const { return _label;}

	void setLabel(std::string newName) {_label=newName;}

	bool visible; // used in rendering; Needed for toggling on and off the meshes
	bool isVisible() { return visible; }
	MeshModel(int id, std::string name);
	MeshModel(int id, CMeshO & m, string name);
	// This function is roughly equivalent to the updateDataMask,
	// but it takes in input a mask coming from a filetype instead of a filter requirement (like topology etc)
	void Enable(int openingFileMask);

	bool hasDataMask(const int maskToBeTested) const;
	void updateDataMask(MeshModel *m);
	void updateDataMask(int neededDataMask);
	void clearDataMask(int unneededDataMask);
	int dataMask() const;


	bool& meshModified();
	static int io2mm(int single_iobit);
};// end class MeshModel


/*
A class designed to save partial aspects of the state of a mesh, such as vertex colors, current selections, vertex positions
and then be able to restore them later.
This is a fundamental part for the dynamic filters framework.
Note: not all the MeshElements are supported!!
*/
class MeshModelState
{
private:
	int changeMask; // a bit mask indicating what have been changed. Composed of MeshModel::MeshElement (e.g. stuff like MeshModel::MM_VERTCOLOR)
	MeshModel *m; // the mesh which the changes refers to.
	std::vector<float> vertQuality;
	std::vector<vcg::Color4b> vertColor;
	std::vector<vcg::Color4b> faceColor;
	std::vector<Point3m> vertCoord;
	std::vector<Point3m> vertNormal;
	std::vector<Point3m> faceNormal;
	std::vector<bool> faceSelection;
	std::vector<bool> vertSelection;
	Matrix44m Tr;
	Shotm shot;
public:
	// This function save the <mask> portion of a mesh into the private members of the MeshModelState class;
	void create(int _mask, MeshModel* _m);
	bool apply(MeshModel *_m);
	bool isValid(MeshModel *m);
};



using namespace vcg;


void error_print(int err, std::string filename)
{
	//if(err) { 
	//	printf("Error in reading %s: '%s'\n",filename, tri::io::Importer<CMeshO>::ErrorMsg(err));
	//	exit(-1);
	//}
}

MeshModel::MeshModel(int id, std::string name)
{
	_id=id;
	_label = name;
	Log::LoadMesh(name.c_str(), cm);
	int dup = tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
	int unref = tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
	tri::UpdateNormal<CMeshO>::NormalizePerVertex(cm);
	printf("Removed %i duplicate and %i unreferenced vertices from mesh %i\n",dup,unref, id);
	printf( "Mesh has %i vert and %i faces\n", cm.VN(), cm.FN());

	Clear();
}

MeshModel::MeshModel(int id, CMeshO & m, string name = "0")
{
	_id=id;
	_label = name;
	int err; int i = 0;

	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(cm, m);
	int dup = tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
	int unref = tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
	tri::UpdateNormal<CMeshO>::NormalizePerVertex(cm);
	tri::UpdateBounding<CMeshO>::Box(cm);
	printf("Removed %i duplicate and %i unreferenced vertices from mesh %i\n",dup,unref,i);
	printf( "Mesh has %i vert and %i faces\n", cm.VN(), cm.FN());

	Clear();
}

void MeshModel::Clear()
{
	meshModified() = false;
	// These data are always active on the mesh
	currentDataMask = MM_NONE;
	currentDataMask |= MM_VERTCOORD | MM_VERTNORMAL | MM_VERTFLAG ;
	currentDataMask |= MM_FACEVERT  | MM_FACENORMAL | MM_FACEFLAG ;

	visible=true;
	cm.Tr.SetIdentity();
	cm.sfn=0;
	cm.svn=0;
}

void MeshModel::UpdateBoxAndNormals()
{
	tri::UpdateBounding<CMeshO>::Box(cm);
	if(cm.fn>0) {
		tri::UpdateNormal<CMeshO>::PerFaceNormalized(cm);
		tri::UpdateNormal<CMeshO>::PerVertexAngleWeighted(cm);
	}
}

int MeshModel::io2mm(int single_iobit)
{
	switch(single_iobit)
	{
	case tri::io::Mask::IOM_NONE					: return  MM_NONE;
	case tri::io::Mask::IOM_VERTCOORD				: return  MM_VERTCOORD;
	case tri::io::Mask::IOM_VERTCOLOR				: return  MM_VERTCOLOR;
	case tri::io::Mask::IOM_VERTFLAGS				: return  MM_VERTFLAG;
	case tri::io::Mask::IOM_VERTQUALITY				: return  MM_VERTQUALITY;
	case tri::io::Mask::IOM_VERTNORMAL				: return  MM_VERTNORMAL;
	case tri::io::Mask::IOM_VERTTEXCOORD			: return  MM_VERTTEXCOORD;
	case tri::io::Mask::IOM_VERTRADIUS				: return  MM_VERTRADIUS;

	case tri::io::Mask::IOM_FACEINDEX   			: return  MM_FACEVERT;
	case tri::io::Mask::IOM_FACEFLAGS   			: return  MM_FACEFLAG;
	case tri::io::Mask::IOM_FACECOLOR   			: return  MM_FACECOLOR;
	case tri::io::Mask::IOM_FACEQUALITY 			: return  MM_FACEQUALITY;
	case tri::io::Mask::IOM_FACENORMAL  			: return  MM_FACENORMAL ;

	case tri::io::Mask::IOM_WEDGTEXCOORD 			: return  MM_WEDGTEXCOORD;
	case tri::io::Mask::IOM_WEDGCOLOR				: return  MM_WEDGCOLOR;
	case tri::io::Mask::IOM_WEDGNORMAL   			: return  MM_WEDGNORMAL;

	case tri::io::Mask::IOM_BITPOLYGONAL   			: return  MM_POLYGONAL;

	default:
		assert(0);
		return MM_NONE;  // FIXME: Returning this is not the best solution (!)
		break;
	} ;
}


void MeshModelState::create(int _mask, MeshModel* _m)
{
	m=_m;
	changeMask=_mask;
	if(changeMask & MeshModel::MM_VERTCOLOR)
	{
		vertColor.resize(m->cm.vert.size());
		std::vector<Color4b>::iterator ci;
		CMeshO::VertexIterator vi;
		for(vi = m->cm.vert.begin(), ci = vertColor.begin(); vi != m->cm.vert.end(); ++vi, ++ci)
			if(!(*vi).IsD()) (*ci)=(*vi).C();
	}

	if(changeMask & MeshModel::MM_VERTQUALITY)
	{
		vertQuality.resize(m->cm.vert.size());
		std::vector<float>::iterator qi;
		CMeshO::VertexIterator vi;
		for(vi = m->cm.vert.begin(), qi = vertQuality.begin(); vi != m->cm.vert.end(); ++vi, ++qi)
			if(!(*vi).IsD()) (*qi)=(*vi).Q();
	}

	if(changeMask & MeshModel::MM_VERTCOORD)
	{
		vertCoord.resize(m->cm.vert.size());
		std::vector<Point3m>::iterator ci;
		CMeshO::VertexIterator vi;
		for(vi = m->cm.vert.begin(), ci = vertCoord.begin(); vi != m->cm.vert.end(); ++vi, ++ci)
			if(!(*vi).IsD()) (*ci)=(*vi).P();
	}

	if(changeMask & MeshModel::MM_VERTNORMAL)
	{
		vertNormal.resize(m->cm.vert.size());
		std::vector<Point3m>::iterator ci;
		CMeshO::VertexIterator vi;
		for(vi = m->cm.vert.begin(), ci = vertNormal.begin(); vi != m->cm.vert.end(); ++vi, ++ci)
			if(!(*vi).IsD()) (*ci)=(*vi).N();
	}

	if(changeMask & MeshModel::MM_FACENORMAL)
	{
		faceNormal.resize(m->cm.face.size());
		std::vector<Point3m>::iterator ci;
		CMeshO::FaceIterator fi;
		for(fi = m->cm.face.begin(), ci = faceNormal.begin(); fi != m->cm.face.end(); ++fi, ++ci)
			if(!(*fi).IsD()) (*ci) = (*fi).N();
	}

	if(changeMask & MeshModel::MM_FACECOLOR)
	{
		m->updateDataMask(MeshModel::MM_FACECOLOR);
		faceColor.resize(m->cm.face.size());
		std::vector<Color4b>::iterator ci;
		CMeshO::FaceIterator fi;
		for(fi = m->cm.face.begin(), ci = faceColor.begin(); fi != m->cm.face.end(); ++fi, ++ci)
			if(!(*fi).IsD()) (*ci) = (*fi).C();
	}

	if(changeMask & MeshModel::MM_FACEFLAGSELECT)
	{
		faceSelection.resize(m->cm.face.size());
		std::vector<bool>::iterator ci;
		CMeshO::FaceIterator fi;
		for(fi = m->cm.face.begin(), ci = faceSelection.begin(); fi != m->cm.face.end(); ++fi, ++ci)
			if(!(*fi).IsD()) (*ci) = (*fi).IsS();
	}

	if(changeMask & MeshModel::MM_VERTFLAGSELECT)
	{
		vertSelection.resize(m->cm.vert.size());
		std::vector<bool>::iterator ci;
		CMeshO::VertexIterator vi;
		for(vi = m->cm.vert.begin(), ci = vertSelection.begin(); vi != m->cm.vert.end(); ++vi, ++ci)
			if(!(*vi).IsD()) (*ci) = (*vi).IsS();
	}

	if(changeMask & MeshModel::MM_TRANSFMATRIX)
		Tr = m->cm.Tr;
	if(changeMask & MeshModel::MM_CAMERA)
		this->shot = m->cm.shot;
}

bool MeshModelState::apply(MeshModel *_m)
{
	if(_m != m)
		return false;
	if(changeMask & MeshModel::MM_VERTCOLOR)
	{
		if(vertColor.size() != m->cm.vert.size()) return false;
		std::vector<Color4b>::iterator ci;
		CMeshO::VertexIterator vi;
		for(vi = m->cm.vert.begin(), ci = vertColor.begin(); vi != m->cm.vert.end(); ++vi, ++ci)
			if(!(*vi).IsD()) (*vi).C()=(*ci);
	}
	if(changeMask & MeshModel::MM_FACECOLOR)
	{
		if(faceColor.size() != m->cm.face.size()) return false;
		std::vector<Color4b>::iterator ci;
		CMeshO::FaceIterator fi;
		for(fi = m->cm.face.begin(), ci = faceColor.begin(); fi != m->cm.face.end(); ++fi, ++ci)
			if(!(*fi).IsD()) (*fi).C()=(*ci);
	}
	if(changeMask & MeshModel::MM_VERTQUALITY)
	{
		if(vertQuality.size() != m->cm.vert.size()) return false;
		std::vector<float>::iterator qi;
		CMeshO::VertexIterator vi;
		for(vi = m->cm.vert.begin(), qi = vertQuality.begin(); vi != m->cm.vert.end(); ++vi, ++qi)
			if(!(*vi).IsD()) (*vi).Q()=(*qi);
	}

	if(changeMask & MeshModel::MM_VERTCOORD)
	{
		if(vertCoord.size() != m->cm.vert.size()) return false;
		std::vector<Point3m>::iterator ci;
		CMeshO::VertexIterator vi;
		for(vi = m->cm.vert.begin(), ci = vertCoord.begin(); vi != m->cm.vert.end(); ++vi, ++ci)
			if(!(*vi).IsD()) (*vi).P()=(*ci);
	}

	if(changeMask & MeshModel::MM_VERTNORMAL)
	{
		if(vertNormal.size() != m->cm.vert.size()) return false;
		std::vector<Point3m>::iterator ci;
		CMeshO::VertexIterator vi;
		for(vi = m->cm.vert.begin(), ci=vertNormal.begin(); vi != m->cm.vert.end(); ++vi, ++ci)
			if(!(*vi).IsD()) (*vi).N()=(*ci);
	}

	if(changeMask & MeshModel::MM_FACENORMAL)
	{
		if(faceNormal.size() != m->cm.face.size()) return false;
		std::vector<Point3m>::iterator ci;
		CMeshO::FaceIterator fi;
		for(fi = m->cm.face.begin(), ci=faceNormal.begin(); fi != m->cm.face.end(); ++fi, ++ci)
			if(!(*fi).IsD()) (*fi).N()=(*ci);
	}

	if(changeMask & MeshModel::MM_FACEFLAGSELECT)
	{
		if(faceSelection.size() != m->cm.face.size()) return false;
		std::vector<bool>::iterator ci;
		CMeshO::FaceIterator fi;
		for(fi = m->cm.face.begin(), ci = faceSelection.begin(); fi != m->cm.face.end(); ++fi, ++ci)
		{
			if((*ci))
				(*fi).SetS();
			else
				(*fi).ClearS();
		}
	}

	if(changeMask & MeshModel::MM_VERTFLAGSELECT)
	{
		if(vertSelection.size() != m->cm.vert.size()) return false;
		std::vector<bool>::iterator ci;
		CMeshO::VertexIterator vi;
		for(vi = m->cm.vert.begin(), ci = vertSelection.begin(); vi != m->cm.vert.end(); ++vi, ++ci)
		{
			if((*ci))
				(*vi).SetS();
			else
				(*vi).ClearS();
		}
	}

	if(changeMask & MeshModel::MM_TRANSFMATRIX)
		m->cm.Tr=Tr;
	if(changeMask & MeshModel::MM_CAMERA)
		m->cm.shot = this->shot;

	return true;
}

/**** DATAMASK STUFF ****/
bool MeshModel::hasDataMask(const int maskToBeTested) const
{
	return ((currentDataMask & maskToBeTested)!= 0);
}

void MeshModel::updateDataMask(MeshModel *m)
{
	updateDataMask(m->currentDataMask);
}

void MeshModel::updateDataMask(int neededDataMask)
{
	if((neededDataMask & MM_FACEFACETOPO)!=0)
	{
		cm.face.EnableFFAdjacency();
		tri::UpdateTopology<CMeshO>::FaceFace(cm);
	}
	if((neededDataMask & MM_VERTFACETOPO)!=0)
	{
		cm.vert.EnableVFAdjacency();
		cm.face.EnableVFAdjacency();
		tri::UpdateTopology<CMeshO>::VertexFace(cm);
	}

	if((neededDataMask & MM_WEDGTEXCOORD)!=0)  cm.face.EnableWedgeTexCoord();
	if((neededDataMask & MM_FACECOLOR)!=0)     cm.face.EnableColor();
	if((neededDataMask & MM_FACEQUALITY)!=0)   cm.face.EnableQuality();
	if((neededDataMask & MM_FACECURVDIR)!=0)   cm.face.EnableCurvatureDir();
	if((neededDataMask & MM_FACEMARK)!=0)	     cm.face.EnableMark();
	if((neededDataMask & MM_VERTMARK)!=0)      cm.vert.EnableMark();
	if((neededDataMask & MM_VERTCURV)!=0)      cm.vert.EnableCurvature();
	if((neededDataMask & MM_VERTCURVDIR)!=0)   cm.vert.EnableCurvatureDir();
	if((neededDataMask & MM_VERTRADIUS)!=0)    cm.vert.EnableRadius();
	if((neededDataMask & MM_VERTTEXCOORD)!=0)  cm.vert.EnableTexCoord();

	currentDataMask |= neededDataMask;
}

void MeshModel::clearDataMask(int unneededDataMask)
{
	if( ( (unneededDataMask & MM_VERTFACETOPO)!=0)	&& hasDataMask(MM_VERTFACETOPO)) {cm.face.DisableVFAdjacency();
	cm.vert.DisableVFAdjacency(); }
	if( ( (unneededDataMask & MM_FACEFACETOPO)!=0)	&& hasDataMask(MM_FACEFACETOPO))	cm.face.DisableFFAdjacency();

	if( ( (unneededDataMask & MM_WEDGTEXCOORD)!=0)	&& hasDataMask(MM_WEDGTEXCOORD)) 	cm.face.DisableWedgeTexCoord();
	if( ( (unneededDataMask & MM_FACECOLOR)!=0)			&& hasDataMask(MM_FACECOLOR))			cm.face.DisableColor();
	if( ( (unneededDataMask & MM_FACEQUALITY)!=0)		&& hasDataMask(MM_FACEQUALITY))		cm.face.DisableQuality();
	if( ( (unneededDataMask & MM_FACEMARK)!=0)			&& hasDataMask(MM_FACEMARK))			cm.face.DisableMark();
	if( ( (unneededDataMask & MM_VERTMARK)!=0)			&& hasDataMask(MM_VERTMARK))			cm.vert.DisableMark();
	if( ( (unneededDataMask & MM_VERTCURV)!=0)			&& hasDataMask(MM_VERTCURV))			cm.vert.DisableCurvature();
	if( ( (unneededDataMask & MM_VERTCURVDIR)!=0)		&& hasDataMask(MM_VERTCURVDIR))		cm.vert.DisableCurvatureDir();
	if( ( (unneededDataMask & MM_VERTRADIUS)!=0)		&& hasDataMask(MM_VERTRADIUS))		cm.vert.DisableRadius();
	if( ( (unneededDataMask & MM_VERTTEXCOORD)!=0)	&& hasDataMask(MM_VERTTEXCOORD))	cm.vert.DisableTexCoord();

	currentDataMask = currentDataMask & (~unneededDataMask);
}

void MeshModel::Enable(int openingFileMask)
{
	if( openingFileMask & tri::io::Mask::IOM_VERTTEXCOORD )
		updateDataMask(MM_VERTTEXCOORD);
	if( openingFileMask & tri::io::Mask::IOM_WEDGTEXCOORD )
		updateDataMask(MM_WEDGTEXCOORD);
	if( openingFileMask & tri::io::Mask::IOM_VERTCOLOR    )
		updateDataMask(MM_VERTCOLOR);
	if( openingFileMask & tri::io::Mask::IOM_FACECOLOR    )
		updateDataMask(MM_FACECOLOR);
	if( openingFileMask & tri::io::Mask::IOM_VERTRADIUS   ) updateDataMask(MM_VERTRADIUS);
	if( openingFileMask & tri::io::Mask::IOM_CAMERA       ) updateDataMask(MM_CAMERA);
	if( openingFileMask & tri::io::Mask::IOM_VERTQUALITY  ) updateDataMask(MM_VERTQUALITY);
	if( openingFileMask & tri::io::Mask::IOM_FACEQUALITY  ) updateDataMask(MM_FACEQUALITY);
	if( openingFileMask & tri::io::Mask::IOM_BITPOLYGONAL ) updateDataMask(MM_POLYGONAL);
}

bool& MeshModel::meshModified()
{
	return this->modified;
}

int MeshModel::dataMask() const
{
	return currentDataMask;
}

#endif