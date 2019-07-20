#pragma once
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/point_sampling.h>
#include <vcg/complex/algorithms/create/resampler.h>
#include <vcg/complex/algorithms/clustering.h>
#include <vcg/simplex/face/distance.h>
#include <vcg/complex/algorithms/geodesic.h>
#include <vcg/space/index/grid_static_ptr.h>
#include <vcg/complex/algorithms/voronoi_processing.h>
#include "Base.h"

class MeshResampling
{
public: 
	static void UniformResampling(CMeshO& mesh_source, int num)
	{
	}
};