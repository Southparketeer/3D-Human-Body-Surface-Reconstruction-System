#pragma once

#include "BoundedVolume.h"
namespace loo {

	//////////////////////////////////////////
	// Save SDF
	//////////////////////////////////////////

	template<typename T, typename TColor>
	__declspec(dllexport)
		void SaveMesh(std::string filename, const BoundedVolume<T,TargetHost> vol, const BoundedVolume<TColor,TargetHost> volColor , std::vector<float3>&  verts , std::vector<int3>&  faces , float truncate_dist);

	template<typename T, typename Manage>
	void SaveMesh(std::string filename, BoundedVolume<T,TargetDevice,Manage>& vol , std::vector<float3>&  verts , std::vector<int3>&  faces , float truncate_dist)
	{
		loo::BoundedVolume<T,loo::TargetHost,loo::Manage> hvol(vol.w, vol.h, vol.d, vol.bbox.Min(), vol.bbox.Max());
		loo::BoundedVolume<float,loo::TargetHost,loo::Manage> hvolcolor(1,1,1, vol.bbox.Min(), vol.bbox.Max() );
		hvol.CopyFrom(vol);
		SaveMesh<T,float>(filename, hvol, hvolcolor,  verts, faces, truncate_dist);
	}

	template<typename T, typename TColor, typename Manage>
	void SaveMesh(std::string filename, BoundedVolume<T,TargetDevice,Manage>& vol, BoundedVolume<TColor,TargetDevice,Manage>& volColor , std::vector<float3>&  verts , std::vector<int3>&  faces , float truncate_dist)
	{
		loo::BoundedVolume<T,loo::TargetHost,loo::Manage> hvol(vol.w, vol.h, vol.d, vol.bbox.Min(), vol.bbox.Max());
		loo::BoundedVolume<TColor,loo::TargetHost,loo::Manage> hvolcolor(volColor.w, volColor.h, volColor.d, volColor.bbox.Min(), volColor.bbox.Max());
		hvol.CopyFrom(vol);
		hvolcolor.CopyFrom(volColor);

		SaveMesh<T,TColor>(filename, hvol, hvolcolor, verts, faces, truncate_dist);
	}

	template<typename T, typename Manage>
	void SaveMesh(std::string filename, BoundedVolume<T,TargetHost,Manage>& hvol , std::vector<float3>&  verts , std::vector<int3>&  faces , float truncate_dist)
	{
		loo::BoundedVolume<float,loo::TargetHost,loo::Manage> hvolcolor(1,1,1, hvol.bbox.Min(), hvol.bbox.Max() );
		SaveMesh<T,float>(filename, hvol, hvolcolor,  verts, faces, truncate_dist);
	}
	__declspec(dllexport) 
		void SaveMesh2(const BoundedVolume<SDF_t,TargetHost>& vol, std::vector<float3>&  verts , std::vector<int3>&  faces,const std::vector<int3>& coor, const std::vector<uchar2>& mask);

}
