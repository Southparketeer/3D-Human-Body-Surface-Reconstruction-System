// Marching cubes ASSIMP exporter based on Marching Cubes Example Program
// by Cory Bloyd with additional source from Paul Bourke (public domain)
// http://paulbourke.net/geometry/polygonise/
//
// Marching Cubes Example Program
// by Cory Bloyd (corysama@yahoo.com)
//
// A simple, portable and complete implementation of the Marching Cubes
// and Marching Tetrahedrons algorithms in a single source file.
// There are many ways that this code could be made faster, but the
// intent is for the code to be easy to understand.
//
// For a description of the algorithm go to
// http://astronomy.swin.edu.au/pbourke/modelling/polygonise/
//
// This code is public domain.
//

#include "stdio.h"
#include "math.h"

#include "Sdf.h"
#include "MarchingCubes.h"
#include "MarchingCubesTables.h"

namespace loo
{

	//fGetOffset finds the approximate point of intersection of the surface
	// between two points with the values fValue1 and fValue2
	inline float fGetOffset(float fValue1, float fValue2, float fValueDesired)
	{
		const double fDelta = fValue2 - fValue1;
		if(fDelta == 0.0) {
			return 0.5;
		}
		return (fValueDesired - fValue1)/fDelta;
	}

	//vGetColor generates a color from a given position and normal of a point
	inline void vGetColor(float3 &rfColor, const float3 &rfPosition, const float3 &rfNormal)
	{
		rfColor.x = (rfNormal.x > 0.0 ? rfNormal.x : 0.0) + (rfNormal.y < 0.0 ? -0.5*rfNormal.y : 0.0) + (rfNormal.z < 0.0 ? -0.5*rfNormal.z : 0.0);
		rfColor.y = (rfNormal.y > 0.0 ? rfNormal.y : 0.0) + (rfNormal.z < 0.0 ? -0.5*rfNormal.z : 0.0) + (rfNormal.x < 0.0 ? -0.5*rfNormal.x : 0.0);
		rfColor.z = (rfNormal.z > 0.0 ? rfNormal.z : 0.0) + (rfNormal.x < 0.0 ? -0.5*rfNormal.x : 0.0) + (rfNormal.y < 0.0 ? -0.5*rfNormal.y : 0.0);
	}


	template<typename T> bool isfinite(T arg)
	{
		return arg == arg && 
			arg != std::numeric_limits<T>::infinity() &&
			arg != -std::numeric_limits<T>::infinity();
	}


	//vMarchCube performs the Marching Cubes algorithm on a single cube
	template<typename T, typename TColor>
	bool vMarchCube(
		const BoundedVolume<T,loo::TargetHost> vol,
		const BoundedVolume<TColor,loo::TargetHost> volColor,
		int x, int y, int z,
		std::vector<float3>& verts,
		//std::vector<float3>& norms,
		std::vector<int3>& faces,
		float fTargetValue = 0.0f,
		float truncate_dist = 100.f
		) {
			const float3 p = vol.VoxelPositionInUnits(x,y,z);
			const float3 fScale = vol.VoxelSizeUnits();

			//Make a local copy of the values at the cube's corners
			float afCubeValue[8];
			
			for(int iVertex = 0; iVertex < 8; iVertex++) {
				afCubeValue[iVertex] = vol.Get(x+a2fVertexOffset[iVertex][0],y+a2fVertexOffset[iVertex][1],z+a2fVertexOffset[iVertex][2]);
				//if(!isfinite<float>(afCubeValue[iVertex])){ return;}
				if(afCubeValue[iVertex] >= (truncate_dist * 0.85f) || afCubeValue[iVertex] <= -(truncate_dist * 0.85f)) return false;
			}

			//Find which vertices are inside of the surface and which are outside
			int iFlagIndex = 0;
			for(int iVertexTest = 0; iVertexTest < 8; iVertexTest++) {
				if(afCubeValue[iVertexTest] <= fTargetValue)
					iFlagIndex |= 1<<iVertexTest;
			}

			////------------
			////output each volex 's iFlagIndex -->uchar mask
			////------------

			//Find which edges are intersected by the surface
			int iEdgeFlags = aiCubeEdgeFlags[iFlagIndex];

			//If the cube is entirely inside or outside of the surface, then there will be no intersections
			if(iEdgeFlags == 0) {
				return false;
			}

			//Find the point of intersection of the surface with each edge
			//Then find the normal to the surface at those points
			float3 asEdgeVertex[12];
			float3 asEdgeNorm[12];

			bool result = false;
			for(int iEdge = 0; iEdge < 12; iEdge++)
			{
				//if there is an intersection on this edge
				if(iEdgeFlags & (1<<iEdge))
				{
					//float afCubeValue[8];
					float fOffset = fGetOffset(afCubeValue[ a2iEdgeConnection[iEdge][0] ], afCubeValue[ a2iEdgeConnection[iEdge][1] ], fTargetValue);

					asEdgeVertex[iEdge] = make_float3(
						p.x + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][0]  +  fOffset * a2fEdgeDirection[iEdge][0]) * fScale.x,
						p.y + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][1]  +  fOffset * a2fEdgeDirection[iEdge][1]) * fScale.y,
						p.z + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][2]  +  fOffset * a2fEdgeDirection[iEdge][2]) * fScale.z
						);

					const float3 deriv = vol.GetUnitsBackwardDiffDxDyDz( asEdgeVertex[iEdge] );
					asEdgeNorm[iEdge] = deriv / length(deriv);
					if( !isfinite<float>(asEdgeNorm[iEdge].x) || !isfinite<float>(asEdgeNorm[iEdge].y) || !isfinite<float>(asEdgeNorm[iEdge].z) ) {
						asEdgeNorm[iEdge] = make_float3(0,0,0);
					}
				}
			}

			//Draw the triangles that were found.  There can be up to five per cube

			for(int iTriangle = 0; iTriangle < 5; iTriangle++)
			{
				if(a2iTriangleConnectionTable[iFlagIndex][3*iTriangle] < 0)
					break;

				bool no_face = false;
				for(int iCorner = 0; iCorner < 3; iCorner++)
				{
					int iVertex = a2iTriangleConnectionTable[iFlagIndex][3*iTriangle+iCorner];
					if(dot(asEdgeNorm[iVertex], make_float3(0,0,-1))<= 0)
					{
						no_face = true;
						break;
					}
				}

				if(!no_face)
				{
					int face[3];
					for(int iCorner = 0; iCorner < 3; iCorner++)
					{

						int iVertex = a2iTriangleConnectionTable[iFlagIndex][3*iTriangle+iCorner];

						face[iCorner] = verts.size();
						verts.push_back(make_float3(asEdgeVertex[iVertex].x, asEdgeVertex[iVertex].y, asEdgeVertex[iVertex].z) );
					}

					int3 facec = make_int3(face[0], face[1], face[2]);
					faces.push_back(facec);
					result = true;
				}
			}
			return result;
	}

	template<typename T, typename TColor>
	void SaveMesh(std::string filename, const BoundedVolume<T,TargetHost> vol, const BoundedVolume<TColor,TargetHost> volColor, std::vector<float3>&  verts , std::vector<int3>&  faces, float truncate_dist)
	{
		float iso = 0;
		int count  =0 ;
		for(GLint iX = 0; iX < vol.Voxels().x-1; iX++) {
			for(GLint iY = 0; iY < vol.Voxels().y-1; iY++) {
				for(GLint iZ = 0; iZ < vol.Voxels().z-1; iZ++) {
					
					bool a = vMarchCube(vol, volColor, iX,iY,iZ, verts, faces, iso, truncate_dist);
					if(a) count ++;
					
				}
			}
		}
		std::cout<<count<<std::endl;
		//aiMesh* mesh = MeshFromLists(verts,norms,faces,colors);
		//SaveMesh(filename, mesh);
	}

	// Instantiate templates
	template  __declspec(dllexport) void SaveMesh<SDF_t,float>(std::string, const BoundedVolume<SDF_t,TargetHost,DontManage> vol, const BoundedVolume<float,TargetHost> volColor , std::vector<float3>&  verts , std::vector<int3>&  faces, float truncate_dist);


	void vMarchCube2(
		const BoundedVolume<SDF_t,loo::TargetHost>& vol,
		int x, int y, int z,
		uchar2 mask,
		std::vector<float3>& verts,
		std::vector<int3>& faces
		){
			//std::cout<<x<<" "<<y <<" "<<z<<std::endl;
			const float fTargetValue = 0.0f;
			const float3 p = vol.VoxelPositionInUnits(x,y,z);
  			const float3 fScale = vol.VoxelSizeUnits();

			//Make a local copy of the values at the cube's corners
			float afCubeValue[8];
			for(int iVertex = 0; iVertex < 8; iVertex++) {
				afCubeValue[iVertex] = vol.Get(x+a2fVertexOffset[iVertex][0], y+a2fVertexOffset[iVertex][1], z+a2fVertexOffset[iVertex][2]);
			}

			//Find which vertices are inside of the surface and which are outside
			int iFlagIndex = (int)mask.x;
			//Find which edges are intersected by the surface
			int iEdgeFlags = aiCubeEdgeFlags[iFlagIndex];
			//Find the point of intersection of the surface with each edge
			//Then find the normal to the surface at those points
			float3 asEdgeVertex[12];
			float3 asEdgeNorm[12];

			for(int iEdge = 0; iEdge < 12; iEdge++)
			{
				//if there is an intersection on this edge
				if(iEdgeFlags & (1<<iEdge))
				{
					float fOffset = fGetOffset(afCubeValue[ a2iEdgeConnection[iEdge][0] ], afCubeValue[ a2iEdgeConnection[iEdge][1] ], fTargetValue);
					asEdgeVertex[iEdge] = make_float3(
						p.x + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][0]  +  fOffset * a2fEdgeDirection[iEdge][0]) * fScale.x,
						p.y + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][1]  +  fOffset * a2fEdgeDirection[iEdge][1]) * fScale.y,
						p.z + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][2]  +  fOffset * a2fEdgeDirection[iEdge][2]) * fScale.z
						);
				}
			}
			//Draw the triangles that were found.  There can be up to five per cube
			int iFlageTriangle = (int) mask.y;
			for(int iTriangle = 0; iTriangle < 5; iTriangle++)
			{
				if(iFlageTriangle & (1<<iTriangle))
				{
					int face[3];
					for(int iCorner = 0; iCorner < 3; iCorner++)
					{
						int iVertex = a2iTriangleConnectionTable[iFlagIndex][3*iTriangle+iCorner];
						face[iCorner] = verts.size();
						verts.push_back(make_float3(asEdgeVertex[iVertex].x, asEdgeVertex[iVertex].y, asEdgeVertex[iVertex].z) );
					}
					int3 facec = make_int3(face[0], face[1], face[2]);
					faces.push_back(facec);
				}
			}
	}


	__declspec(dllexport) void SaveMesh2(const BoundedVolume<SDF_t,TargetHost>& vol, std::vector<float3>&  verts , std::vector<int3>&  faces,const std::vector<int3>& coor, const std::vector<uchar2>& mask)
	{
		for(int i = 0 ; i < mask.size(); i++)
		{
			//std::cout<<coor.at(i).x<<" "<<coor.at(i).y<<" "<<coor.at(i).z<<std::endl;
			vMarchCube2(vol, coor[i].x, coor[i].y, coor[i].z, mask[i], verts, faces);
		}
	}

	//__declspec(dllexport) void SaveMesh2(const BoundedVolume<SDF_t,TargetHost>& vol, std::vector<float3>&  verts , std::vector<int3>&  faces,const std::vector<int3>& coor, const std::vector<uchar2>& mask);

}
