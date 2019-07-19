#pragma once
#include	<stdio.h>
#include	<Eigen/Dense>
#include	"ppl.h"

using namespace Eigen;
//Common typedef
typedef Matrix<float, 3, 3, RowMajor> Mat3;
typedef Matrix<float, 4, 4, RowMajor> Mat4;
typedef Vector3f Vet3;
typedef Vector4f Vet4;
typedef unsigned int UINT;
typedef unsigned char BYTE;
//Common structure
struct BaryCoord{
	int idx;			//mesh idx
	float u;			//BaryCoordinate u;
	float v;			//BaryCoordinate v; // u + v + w = 1
};
struct GlobalCorr{
	BaryCoord  Corr_T;	//correspondence Coordinate in target surface
	int VexId_S;		//Vertex id on source surface
	int MeshId_S;		//Source mesh Id
	int MeshId_T;		//Target mesh Id
};
struct Coo{
	int row;
	int col;
	double val;
};

//Common macro definition
#ifndef SAFE_DELETE
#define SAFE_DELETE(p) { if (p) { delete (p); (p)=NULL; } }
#endif
#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(p) { if (p) { delete[] (p); (p)=NULL; } }
#endif
#ifndef PI
#define PI 3.14159265
#endif
//Custom macro definition
#define NUM_FACE 8

#define FRAME_NUM	30
#define DEPTH_RAW_WIDTH  512
#define DEPTH_RAW_HEIGHT 424
#define COLOR_RAW_WIDTH  1920
#define COLOR_RAW_HEIGHT 1080
#define DEBUG_INFO 1