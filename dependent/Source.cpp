//#include "StdAfx.h"
//#include "tsdfcuda.h"
//#include "Helper.h"
//#include "Base.h"
//#include "Timer.h"
//#include "cvUndistort.h"
//#include "region_grow.h"
//#include "math.h"
//#include "IMAGE_DISPLAY/GLDisplay.h"
//#include "IMAGE_DISPLAY/png_io.h"
//#include "ICP/icp.h"
//#include "SKELETON/JointsEstimate.h"
//#include "depthclean.h"
//#include "2D Morph/diffuseWeight.h"
//#include "2D Morph/pixelTriangle.h"
#if 0
Mat3 setRotMatrix(float angle)
{
	float rotate_angle = angle * (3.14159 / 180.);
	Mat3 matRes;
	matRes.setIdentity();
	if(angle == 0)
		return matRes;
	matRes(0,0) =  cos(rotate_angle);
	matRes(0,1) = -sin(rotate_angle);
	matRes(1,0) =  sin(rotate_angle);
	matRes(1,1) =  cos(rotate_angle);
	return matRes;
}

int main()
{
	int h = DEPTH_RAW_WIDTH;
	int w = DEPTH_RAW_HEIGHT;
	diffuseWeight dw;
	dw.init("DiffusionWeight_Test.png", DEPTH_RAW_HEIGHT, DEPTH_RAW_WIDTH);
	dw.weightDiffuse(200);
	diffuseWeight dw_target;
	dw_target.init("Target_0.png", DEPTH_RAW_HEIGHT, DEPTH_RAW_WIDTH);
	CMeshO TrianglizeMesh;
	pixelTriangle::ConvertPixel2Triangle(dw.image, TrianglizeMesh, DEPTH_RAW_HEIGHT, DEPTH_RAW_WIDTH, 1);
	CMeshO TrianglizeMesh_target;
	pixelTriangle::ConvertPixel2Triangle(dw_target.image, TrianglizeMesh_target, DEPTH_RAW_HEIGHT, DEPTH_RAW_WIDTH, 1);
	Log::LogMesh_CMeshO(&TrianglizeMesh_target, "Trianglize_Test_Target.ply");

	//--------------------------Test-----------------------------//

	vector<float> buff_img(w * h);
	vector<Mat3, aligned_allocator<Mat3>> mat_Segmenbt(6);
	mat_Segmenbt[wcombo::torso] = setRotMatrix(0);
	mat_Segmenbt[wcombo::head] = setRotMatrix(0);
	mat_Segmenbt[wcombo::armLeft] = setRotMatrix(-25);
	mat_Segmenbt[wcombo::armRight] = setRotMatrix(27);
	mat_Segmenbt[wcombo::legLeft] = setRotMatrix(-7);
	mat_Segmenbt[wcombo::legRight] = setRotMatrix(4);

	for(int i = 0; i < TrianglizeMesh.vert.size(); i++)
	{
		Point3f currv = TrianglizeMesh.vert[i].P();
		int imgidx = (int) currv.X() + (int) currv.Y() * DEPTH_RAW_HEIGHT; 
		wcombo curr_w = dw.weight[imgidx];
		Vector3f avge_Pos(0,0,0);
		for(int k = 0 ; k < 6; k++)
		{
			if(curr_w.w[k] == 0)
				continue;
			Vector3f curr_Pos(currv.X(), currv.Y(), 1);
			Point3f Joint_Pos(dw.JointControl[k].X(), dw.JointControl[k].Y(), 1);
			Mat3 T, T_inv;
			T.setIdentity();
			T.block<2,1>(0,2) = Vector2f(-Joint_Pos.X(), -Joint_Pos.Y());
			T_inv.setIdentity();
			T_inv.block<2,1>(0,2) = Vector2f(Joint_Pos.X(), Joint_Pos.Y());
			Vector3f morp_Pos = (T_inv * mat_Segmenbt[k] * T) * curr_Pos;
			avge_Pos += morp_Pos * curr_w.w[k];
		}
		TrianglizeMesh.vert[i].P() = Point3f(avge_Pos.x(), avge_Pos.y(), 0);
	}

	vcg::tri::UpdateBounding<CMeshO>::Box(TrianglizeMesh_target);
	vcg::tri::UpdateBounding<CMeshO>::Box(TrianglizeMesh);
	Point3f trans;
	trans.X() = 0;//TrianglizeMesh_target.bbox.Center().X() - TrianglizeMesh.bbox.Center().X();
	trans.Y() = TrianglizeMesh_target.bbox.min.Y() - TrianglizeMesh.bbox.min.Y();
	trans.Z() = 0;
	vcg::tri::UpdatePosition<CMeshO>::Translate(TrianglizeMesh, trans);
	Log::LogMesh_CMeshO(&TrianglizeMesh, "Trianglize_Test.ply");
	return 0;
}
#endif