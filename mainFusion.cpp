#include "StdAfx.h"
#include "tsdfcuda.h"
#include "Helper.h"
#include "Base.h"
#include "Timer.h"
#include "cvUndistort.h"
#include "region_grow.h"
#include "math.h"
#include "IMAGE_DISPLAY/GLDisplay.h"
#include "IMAGE_DISPLAY/png_io.h"
#include "ICP/icp.h"
#include "SKELETON/JointsEstimate.h"
#include "depthclean.h"
#include "Nanoknn.h"
#include "SKELETON/JointsEstimate2.h"
using namespace std;
string ROOT("Data//DATA TEST6//");
string DEPTHK0 = ROOT + "DEPTH//K0//";
string DEPTHK1 = ROOT + "DEPTH//K1//";
string DEPTHK0_C = ROOT + "DEPTH_CLEAN//K0//";
string DEPTHK1_C = ROOT + "DEPTH_CLEAN//K1//";
string ICPFILE = ROOT + "ICP//i";
string FUSFILE = ROOT + "FUSE//f";
string DBGFILE = ROOT + "DEBUG//";
string GRUFILE = ROOT + "GROUND//Pose1_";
string FILE_GROUND_PLY = ROOT + "Ground.ply";
//PARAMETER IN
Mat4 Extrinstic_ir_ir;
loo::ImageIntrinsics K_K0;
loo::ImageIntrinsics CK_K0;
loo::distortParam D_K0;
loo::distortParam CD_K0;
loo::ImageIntrinsics CK_K1;
loo::ImageIntrinsics K_K1;
loo::distortParam D_K1;
loo::distortParam CD_K1;
Mat4 Mat_extr_ir_rgb_K0;
Mat4 Mat_extr_ir_rgb_K1;

Mat4 ICP_refine;
float4 ground_plane_param;

//-------------------------------Global Variables-----------------------------------//
int w = DEPTH_RAW_WIDTH;
int h = DEPTH_RAW_HEIGHT;
Timer ttime;

ICP icprefine;
GLdisplay * displayGL;

Mat4 World_Camera_K0;
Mat4 World_Camera_K1;
vector<vector<float>> depth_buff_K0(FRAME_NUM);
vector<vector<float>> depth_buff_K1(FRAME_NUM);
vector<BYTE> color_buff_K0;
vector<BYTE> color_buff_K1;

loo::Mat<float,3,4> T_tsdf_cw_K0;
loo::Mat<float,3,4> T_tsdf_cw_K1;
loo::Mat<float,3,4> T_raycast_cw;

//contorl
float max_w = 1000;
const float trunc_dist_factor = 2.0;
int global_index = 0;
float ray_cast_z = -500;
bool if_normal = false;
//-------------------------------CUDA memory objects-----------------------------------//
//raw depth
loo::Image<float, loo::TargetHost, loo::Manage> h_rawdepth_K0(w,h);
loo::Image<float, loo::TargetHost, loo::Manage> h_rawdepth_K1(w,h);
loo::Image<float, loo::TargetDevice, loo::Manage> d_tsdf_depth(w,h);
loo::Image<float, loo::TargetDevice, loo::Manage> d_tsdf_depth_undistort(w,h);
//TSDF depth processing
loo::Image<float, loo::TargetDevice, loo::Manage> d_tsdf_depth_bi(w,h);
loo::Image<float, loo::TargetDevice, loo::Manage> d_tsdf_depth_mi(w,h);
loo::Image<float4, loo::TargetDevice, loo::Manage> d_tsdf_normal(w,h);
loo::Image<float4, loo::TargetDevice, loo::Manage> d_tsdf_vertex(w,h);
//ray cast
loo::Image<float, loo::TargetDevice, loo::Manage> d_raycast_render(w, h);
loo::Image<float, loo::TargetDevice, loo::Manage> d_raycast_depth(w, h);
loo::Image<float4, loo::TargetDevice, loo::Manage> d_raycast_norm(w, h);
loo::Image<float,  loo::TargetHost, loo::Manage> h_raycast_depth(w, h);
loo::Image<float4, loo::TargetHost, loo::Manage> h_raycast_normal(w,h);
loo::Image<float,  loo::TargetHost, loo::Manage> h_raycast_render(w,h);
loo::Image<float4, loo::TargetHost, loo::Manage> h_raycast_vertex(w,h);

//--------------------------------------------------------------------------------------//
void runTSDF(loo::BoundedVolume<loo::SDF_t, loo::TargetDevice, loo::Manage> &d_vol,
			 const loo::Image<float, loo::TargetHost, loo::Manage> &h_rawdepth,  
			 const loo::Mat<float,3,4>& T_tsdf_cw_K, 
			 const loo::distortParam& DP_K, 
			 const loo::ImageIntrinsics& K_K, 
			 unsigned char if_clip)
{
	float depth_clip_near = 500;
	float depth_clip_far = 2000;
	float bigs = 1.5, bigr = 0.01;
	int biwin = 1;
	int midwin = 1;
	float mincostheta = 0.1;

	const float trunc_dist = trunc_dist_factor*length(d_vol.VoxelSizeUnits());

	d_tsdf_depth.CopyFrom(h_rawdepth);
	loo::MediumFilter<float, float>(d_tsdf_depth_mi, d_tsdf_depth, midwin, depth_clip_near, depth_clip_far);
	loo::BilateralFilter<float, float>(d_tsdf_depth_bi, d_tsdf_depth_mi, bigs, bigr, biwin);
	loo::MediumFilter<float, float>(d_tsdf_depth_mi, d_tsdf_depth_bi, midwin, depth_clip_near, depth_clip_far);
	loo::DepthToVbo<float>(d_tsdf_vertex, d_tsdf_depth_mi, K_K);
	loo::NormalsFromVbo(d_tsdf_normal, d_tsdf_vertex);
	loo::SdfFuse(d_vol, d_tsdf_depth_mi, d_tsdf_normal, T_tsdf_cw_K, K_K, trunc_dist, max_w, mincostheta, if_clip);
}

inline int countones(unsigned char c)
{
	int count = 0;
	for(int i = 0 ; i < 8 ; i ++)
	{
		if(c & 1 << i ) count++;
	}
	return count;
}

void MeshClean(CMeshO & cm_in, CMeshO & cm_out, int dense = 5, float percent = 0.01f)
{
	int dup = tri::Clean<CMeshO>::RemoveDuplicateVertex(cm_in);
	int cls = tri::Clean<CMeshO>::MergeCloseVertex(cm_in, dense);
	int unref = vcg::tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm_in);
	printf("Remove %d duplicate Vertex\n2. Merge %d Vertex\n3. Remove %d unreference vertex\n", dup, cls, unref);

	tri::Append<CMeshO, CMeshO>::MeshCopy(cm_out, cm_in);
	const int minCC= (int)(cm_out.vert.size() * percent);
	tri::UpdateTopology<CMeshO>::FaceFace(cm_out);
	std::pair<int,int> delInfo=tri::Clean<CMeshO>::RemoveSmallConnectedComponentsSize(cm_out,minCC);
	int unref2 = vcg::tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm_out);
	tri::UpdateNormal<CMeshO>::PerVertex(cm_out);
	tri::UpdateNormal<CMeshO>::NormalizePerVertex(cm_out);
	printf("remove %d component out of %d component\n", delInfo.first, delInfo.second);
}

void PoseGroundUp(CMeshO & mesh, const float4& GroundParam, Mat4* Tr = NULL)
{
	vcg::tri::UpdateBounding<CMeshO>::Box(mesh);

	float a = GroundParam.x;
	float b = GroundParam.y;
	float c = GroundParam.z;
	float d = GroundParam.w;

	Vet3 xA(a, b, c);
	xA = (Extrinstic_ir_ir.block<3,3>(0,0) * ICP_refine.block<3,3>(0,0)).inverse() * xA;

	xA.normalize();
	Vet3 xB(0, -1, 0);
	xA.normalize();
	Vet3 x = xA.cross(xB);
	float s = x.norm();
	float cc = xA.dot(xB);

	Mat3 mA;
	mA.setZero();
	mA(0,1) = -x.z();
	mA(0,2) =  x.y();
	mA(1,0) =  x.z();
	mA(1,2) = -x.x();
	mA(2,0) = -x.y();
	mA(2,1) =  x.x();
	Mat4 R;
	R.setIdentity();
	R.block<3,3>(0,0) += mA + mA * mA * (1 - cc) / s * s;


	vcg::Matrix44f gr;
	gr.FromEigenMatrix(R);
	vcg::tri::UpdatePosition<CMeshO>::Matrix(mesh, gr);
	vcg::tri::UpdateBounding<CMeshO>::Box(mesh);

	Mat4 R2;
	R2.setIdentity();
	R2(0,0) = cos(PI);
	R2(0,1) = -sin(PI);
	R2(1,0) = sin(PI);
	R2(1,1) = cos(PI);
	R2(1,3) = mesh.bbox.max.Y();
	R2(2,3) = -mesh.bbox.Center().Z();
	gr.FromEigenMatrix(R2);
	vcg::tri::UpdatePosition<CMeshO>::Matrix(mesh, gr);
	if(Tr != NULL)
	{
		*Tr = R2 * R;
	}
}

void MeshOutput( loo::BoundedVolume<loo::SDF_t, loo::TargetDevice, loo::Manage> &d_vol,	CMeshO& mesh_des, int dense = 5, float percent = 0.05)
{
	const float trunc_dist = trunc_dist_factor * length(d_vol.VoxelSizeUnits());
	float3 ray_K0 = make_float3(0,0,-1);
	Eigen::Vector3f r0(ray_K0.x, ray_K0.y, ray_K0.z);
	Eigen::Vector3f r1 = World_Camera_K1.block<3,3>(0,0) * r0;
	float3 ray_K1 = make_float3(r1.x(), r1.y(), r1.z());

	loo::BoundedVolume<uchar2, loo::TargetDevice, loo::Manage> d_vol_mask(d_vol.w, d_vol.h, d_vol.d, d_vol.bbox);
	loo::MarchingCubeMaskGPU(d_vol, d_vol_mask, trunc_dist, ray_K0, ray_K1);
	loo::BoundedVolume<uchar2, loo::TargetHost, loo::Manage> h_vol_mask(d_vol.w, d_vol.h, d_vol.d, d_vol.bbox);
	h_vol_mask.CopyFrom(d_vol_mask);

	vector<uchar2> mask;
	vector<int3> index;
	mask.reserve(200000);
	index.reserve(200000);

	int count_triangles = 0;
	for( int z = 0 ; z < d_vol.d -1 ; z++ )
	{
		for( int y = 0 ; y < d_vol.h -1 ; y++ )
		{
			for(int x = 0 ; x < d_vol.w -1 ; x++ )
			{
				uchar2 m = h_vol_mask(x,y,z);
				if(m.x != 0 && m.y != 0)
				{
					mask.push_back(m);
					index.push_back(make_int3(x,y,z));
					count_triangles += countones(m.y);
				}
			}
		}
	}

	vector<float3> verts;
	vector<int3> faces;
	verts.clear();
	faces.clear();
	verts.reserve(3 * count_triangles);
	faces.reserve(count_triangles);

	loo::BoundedVolume<loo::SDF_t, loo::TargetHost, loo::Manage> h_vol(d_vol.w, d_vol.h, d_vol.d, d_vol.bbox);
	h_vol.CopyFrom(d_vol);
	loo::SaveMesh2(h_vol, verts, faces, index, mask);


	vcg::CMeshO cm;
	cm.vert.resize(verts.size());
	cm.vn = cm.vert.size();
	cm.face.resize(faces.size());
	cm.fn = cm.face.size();

	Concurrency::parallel_for(0, (int) cm.vert.size(), [&](int k)
	{
		cm.vert.at(k).P() = vcg::Point3f(verts[k].x, verts[k].y, verts[k].z);
	});

	CVertexO * V_start = &cm.vert[0];

	for(int k = 0 ; k < cm.face.size(); k++)
	{
		cm.face[k].setvptr(0, V_start + faces[k].x);
		cm.face[k].setvptr(2, V_start + faces[k].y);
		cm.face[k].setvptr(1, V_start + faces[k].z);
	}

	CMeshO cm_clean;
	if(dense != 0)
	{
		MeshClean(cm, cm_clean, dense, percent);
	}
	else
	{
		tri::Append<CMeshO, CMeshO>::MeshCopy(cm_clean, cm);
	}
	vcg::tri::UpdateNormal<CMeshO>::PerVertex(cm_clean);
	vcg::tri::UpdateNormal<CMeshO>::NormalizePerVertex(cm_clean);
	tri::Append<CMeshO, CMeshO>::MeshCopy(mesh_des, cm_clean);
}

//h_rawdepthBK -> h_raycast_depth
//h_normalBK -> h_raycast_normal
//h_vertexBK -> h_raycast_vertex
//d_rawdepthBK -> d_tsdf_depth
//d_rawdepthBK_bi -> d_tsdf_depth_bi
//d_normalBK -> d_tsdf_normal
//d_vertexBK -> d_tsdf_vertex
void undistort( float* src, float* des, const loo::ImageIntrinsics& K, const loo::distortParam & D, int output = 0, string* name = NULL, bool if_up = true, int idx_k = 0)
{
	cvUndistort::cv_undistort(src, des, K, D, DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT);
	// if not the up Kinect, do background clean
	if(if_up == false)
	{
		memcpy(h_raycast_depth.ptr, des, w * h * sizeof(float));
		d_tsdf_depth.CopyFrom(h_raycast_depth);
		loo::BilateralFilter<float, float>(d_tsdf_depth_bi, d_tsdf_depth, 1.5, 0.1, 0);
		loo::DepthToVbo<float>(d_tsdf_vertex, d_tsdf_depth_bi, K_K1);
		loo::NormalsFromVbo(d_tsdf_normal, d_tsdf_vertex);
		h_raycast_normal.CopyFrom(d_tsdf_normal);
		h_raycast_vertex.CopyFrom(d_tsdf_vertex);
		h_rawdepth_K1.CopyFrom(d_tsdf_depth);

		Vet3 n_planev(ground_plane_param.x, ground_plane_param.y, ground_plane_param.z);
		n_planev.normalize();
		float a = ground_plane_param.x;
		float b = ground_plane_param.y;
		float c = ground_plane_param.z;
		float d = ground_plane_param.w;

		int clip_y_up = 512;
		int clip_y_low = 0;
		Concurrency::parallel_for(0, (h * w), [&](int j){
			if(h_raycast_depth.ptr[j] > 2000 || h_raycast_depth[j] < 500) 
			{
				des[j] = 0;
				return;
			}
			int x = j % w;
			int y = j / w;



			if(x < 0)
			{
				des[j] = 0;
				return;
			}

			Vet3 vv(h_raycast_vertex.ptr[j].x, h_raycast_vertex.ptr[j].y, h_raycast_vertex.ptr[j].z);
			float dis = a * vv.x() + b * vv.y() + c * vv.z() + d;
			dis = dis / sqrtf( a * a + b * b + c * c);

			if(dis < 500 && h_raycast_vertex.ptr[j].z > 2000)
			{
				des[j] = 0;
				return;
			}


			int s = 13;
			int thr ;
			thr = (idx_k == 2 || idx_k == 6) ? 2 - s : 0 - s;
			thr = (idx_k == 0) ? -7 - s : 0 - s;
			if(dis > thr) 
			{
				if( x > w / 3 && x < w && (idx_k == 2 || idx_k == 6))
				{
					clip_y_up = min(clip_y_up, y);
					clip_y_low = max(clip_y_low, y);
				}
				return;
			}
			if(dis <thr) {des[j] = 0;	 return;	}

		});

		if(idx_k == 2 || idx_k == 6)
		{
			Concurrency::parallel_for(0, (h * w), [&](int j){
				int y = j / w;
				if(y < clip_y_up - 30 || y > clip_y_low + 30)
					des[j] = 0;
			});
		}
	}

	if(idx_k == 2 && if_up == true)
		return;
	regiongrow::RegionGrowW(des, 500, 2000, 1000, DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT);


	if(output == 0)
		return;
	if(output == 1)
	{
		if(name)	PNG_IO::depth_array_to_PNG(des, (*name + "0_undistort.png").c_str(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT);
		else		PNG_IO::depth_array_to_PNG(des, "0_undistort.png", DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT);
	}
	else if(output == 2)
	{
		if(name)	PNG_IO::depth_array_to_PNG(des, (*name + "1_undistort.png").c_str(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT);
		else		PNG_IO::depth_array_to_PNG(des, "1_undistort.png", DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT);
	}

}

std::vector<cv::Point3f> Generate3DPoints(const CMeshO& mesh, const Mat4 & extr_ir_rgb, const Mat4 & extr_ir_world = Mat4::Identity())
{
	Mat4 extr_ir_rgb_inv = extr_ir_rgb.inverse();

	std::vector<cv::Point3f> points;
	points.reserve(10000);
	for(int i = 0 ; i < mesh.vert.size(); i++)
	{
		Vet3 a(mesh.vert[i].P().X(), mesh.vert[i].P().Y(), mesh.vert[i].P().Z());
		a = extr_ir_world.block<3,3>(0,0) * a + extr_ir_world.block<3,1>(0,3);
		a = a / 1000;
		a = extr_ir_rgb_inv.block<3,3>(0,0) * a + extr_ir_rgb_inv.block<3,1>(0,3);
		points.push_back(cv::Point3f(a.x(), a.y(), a.z()));
	}
	return points;
}

void ProjectPoints( const vector<cv::Point3f>& objPoints, const loo::ImageIntrinsics &intr,const loo::distortParam & distCoff, const Mat4& extr, vector<cv::Point2f> &imgPoints)
{
	cv::Mat intrinsticMat(3, 3, cv::DataType<double>::type); // Intrisic matrix
	intrinsticMat.at<double>(0, 0) = intr.fu;
	intrinsticMat.at<double>(1, 0) = 0;
	intrinsticMat.at<double>(2, 0) = 0;
	intrinsticMat.at<double>(0, 1) = 0;
	intrinsticMat.at<double>(1, 1) = intr.fv;
	intrinsticMat.at<double>(2, 1) = 0;
	intrinsticMat.at<double>(0, 2) = intr.u0;
	intrinsticMat.at<double>(1, 2) = intr.v0;
	intrinsticMat.at<double>(2, 2) = 1;

	cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
	rVec.at<double>(0) = 0;
	rVec.at<double>(1) = 0;
	rVec.at<double>(2) = 0;

	cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
	tVec.at<double>(0) = 0;
	tVec.at<double>(1) = 0;
	tVec.at<double>(2) = 0;

	cv::Mat distCoeffs(8, 1, cv::DataType<double>::type);   // Distortion vector
	distCoeffs.at<double>(0) = distCoff.k1;
	distCoeffs.at<double>(1) = distCoff.k2;
	distCoeffs.at<double>(2) = distCoff.p1;
	distCoeffs.at<double>(3) = distCoff.p2;
	distCoeffs.at<double>(4) = distCoff.k3;
	cv::projectPoints(objPoints, rVec, tVec, intrinsticMat, distCoeffs, imgPoints);
}

void textureMapping_SignleMesh(CMeshO & Mesh_K, string File_Mesh_Output_png, vector<BYTE>& color_buff_K, const Mat4& Extr_ir_rgb, const Mat4& Extr_ir_world, const loo::ImageIntrinsics& CK_K, const loo::distortParam& CD_K)
{
	std::vector<cv::Point3f> objectPoints_K;
	std::vector<cv::Point2f> imagePoints_K;
	objectPoints_K = Generate3DPoints(Mesh_K, Extr_ir_rgb, Extr_ir_world);
	imagePoints_K.resize(objectPoints_K.size());
	ProjectPoints(objectPoints_K, CK_K, CD_K, Extr_ir_rgb, imagePoints_K);
	Concurrency::parallel_for(0u, (UINT) Mesh_K.vert.size(), [&](UINT i){
		CVertexO* curr_v = &Mesh_K.vert.at(i);
		int x = floor(imagePoints_K[i].x + 0.5);
		int y = floor(imagePoints_K[i].y + 0.5);
		int base = y * COLOR_RAW_WIDTH +  x;
		if(base < COLOR_RAW_WIDTH * COLOR_RAW_HEIGHT && base >= 0)
		{
			BYTE R = color_buff_K[base * 3 + 0];
			BYTE G = color_buff_K[base * 3 + 1];
			BYTE B = color_buff_K[base * 3 + 2];
			Mesh_K.vert.at(i).C() = (Color4b(R, G, B, 255));
		}
	});

	if(File_Mesh_Output_png.length() > 0)
	{
		Concurrency::parallel_for(0u, (UINT) imagePoints_K.size(), [&](UINT i){
			int x = floor(imagePoints_K[i].x + 0.5);
			int y = floor(imagePoints_K[i].y + 0.5);
			int base = y * COLOR_RAW_WIDTH +  x;
			if(base < COLOR_RAW_WIDTH * COLOR_RAW_HEIGHT && base >= 0)
			{
				color_buff_K[base * 3 + 0] = 245;
				color_buff_K[base * 3 + 1] = 0;
				color_buff_K[base * 3 + 2] = 0;
			}
		});
		PNG_IO::BYTE_array_to_PNG(color_buff_K.data(), string(File_Mesh_Output_png).c_str(), COLOR_RAW_WIDTH, COLOR_RAW_HEIGHT);
	}
}

void textureMapping( CMeshO& Mesh_K, string File_Mesh_Output_png, const Mat4& World_Camera_K1_refine, int k = 0)
{
	std::vector<cv::Point3f> objectPoints_K0;
	std::vector<cv::Point2f> imagePoints_K0;
	std::vector<cv::Point3f> objectPoints_K1;
	std::vector<cv::Point2f> imagePoints_K1;
	objectPoints_K0 = Generate3DPoints(Mesh_K, Mat_extr_ir_rgb_K0);
	imagePoints_K0.resize(objectPoints_K0.size());
	ProjectPoints(objectPoints_K0, CK_K0, CD_K0, Mat_extr_ir_rgb_K0, imagePoints_K0);
	objectPoints_K1 = Generate3DPoints(Mesh_K, Mat_extr_ir_rgb_K1, World_Camera_K1_refine);
	imagePoints_K1.resize(objectPoints_K1.size());
	ProjectPoints(objectPoints_K1, CK_K1, CD_K1, Mat_extr_ir_rgb_K1, imagePoints_K1);

	float boundary_dw;
	float boundary_up;

	vcg::tri::UpdateBounding<CMeshO>::Box(Mesh_K);
	vcg::Box3f box = Mesh_K.bbox;
	boundary_up = box.max.X() - 700;
	boundary_dw = (k == -1) ? -10000 : box.min.X() + 700;
	int color_threshold = (k == 2 || k == 6) ? 210 : 210;

	for(int i = 0 ; i < Mesh_K.vert.size(); i++)
	{
		CVertexO* curr_v = &Mesh_K.vert.at(i);

		float w0, w1;

		if(curr_v->P().X() > boundary_dw && curr_v->P().X() < boundary_up)
		{
			w0 = (curr_v->P().X() - (boundary_dw)) / ( boundary_up - boundary_dw );
			w1 = 1 - w0;
		}
		else if(curr_v->P().X() <= boundary_dw)
		{
			w0 = 0;
			w1 = 1;
		}
		else if(curr_v->P().X() >= boundary_up)
		{
			w0 = 1;
			w1 = 0;
		}
		if(w1 == 0)
		{
			int x = (int)floor(imagePoints_K0[i].x + 0.5) % 1920;
			int y = (int)floor(imagePoints_K0[i].y + 0.5) % 1080;
			int base = y * COLOR_RAW_WIDTH +  x;
			if(base < COLOR_RAW_WIDTH * COLOR_RAW_HEIGHT && base >= 0)
			{
				BYTE R = color_buff_K0[base * 3 + 0];
				BYTE G = color_buff_K0[base * 3 + 1];
				BYTE B = color_buff_K0[base * 3 + 2];
				Mesh_K.vert.at(i).C() = (Color4b(R, G, B, 255));
			}
			continue;
		}
		if(w0 == 0)
		{
			int x = (int)(floor(imagePoints_K1[i].x + 0.5)) % 1920;
			int y = (int)(floor(imagePoints_K1[i].y + 0.5)) % 1080;
			int base = y * COLOR_RAW_WIDTH +  x;
			if(base < COLOR_RAW_WIDTH * COLOR_RAW_HEIGHT && base >= 0)
			{
				BYTE R = color_buff_K1[base * 3 + 0];
				BYTE G = color_buff_K1[base * 3 + 1];
				BYTE B = color_buff_K1[base * 3 + 2];
				Mesh_K.vert.at(i).C() = (Color4b(R, G, B, 255));
			}
			continue;
		}
		else
		{
			Point3f Color(0,0,0);
			int x0 = (int)(floor(imagePoints_K0[i].x + 0.5)) % 1920;
			int y0 = (int)(floor(imagePoints_K0[i].y + 0.5)) % 1080;
			int base0 = y0 * COLOR_RAW_WIDTH +  x0;
			int x1 = floor(imagePoints_K1[i].x + 0.5);
			int y1 = floor(imagePoints_K1[i].y + 0.5);
			int base1 = y1 * COLOR_RAW_WIDTH +  x1;

			if(base1 < 0 || base1 > COLOR_RAW_HEIGHT * COLOR_RAW_WIDTH) continue;
			if(base0 < 0 || base0 > COLOR_RAW_HEIGHT * COLOR_RAW_WIDTH) continue;

			int color_difference = abs(color_buff_K0[base0 * 3 + 0] - color_buff_K1[base1 * 3 + 0]) + abs(color_buff_K0[base0 * 3 + 1] - color_buff_K1[base1 * 3 + 1]) + abs(color_buff_K0[base0 * 3 + 2] - color_buff_K1[base1 * 3 + 2]);


			if(color_difference > color_threshold && base0 >= 0)
			{
				Color.X() = 255;
				Color.Y() = 0;
				Color.Z() = 255;
			}
			else
			{
				if(base0 < COLOR_RAW_WIDTH * COLOR_RAW_HEIGHT && base0 >= 0)
				{
					Color.X() += ((float) color_buff_K0[base0 * 3 + 0]) * w0;
					Color.Y() += ((float) color_buff_K0[base0 * 3 + 1]) * w0;
					Color.Z() += ((float) color_buff_K0[base0 * 3 + 2]) * w0;
				}

				if(base1 < COLOR_RAW_WIDTH * COLOR_RAW_HEIGHT && base1 >= 0)
				{
					Color.X() += ((float) color_buff_K1[base1 * 3 + 0]) * w1;
					Color.Y() += ((float) color_buff_K1[base1 * 3 + 1]) * w1;
					Color.Z() += ((float) color_buff_K1[base1 * 3 + 2]) * w1;
				}
			}
			Mesh_K.vert.at(i).C() = (Color4b((BYTE)Color.X(), (BYTE)Color.Y(), (BYTE)Color.Z(), 255));
		}
		curr_v = NULL;
	}

	if(k == -1)
	{
		return;
	}
	for(int i = 0 ; i < imagePoints_K0.size(); i++)
	{
		int x = (int)(floor(imagePoints_K0[i].x + 0.5)) % 1920;
		int y = (int)(floor(imagePoints_K0[i].y + 0.5)) % 1080;
		if(objectPoints_K0[i].x < -650) continue;
		int base = y * COLOR_RAW_WIDTH +  x;
		if(base < COLOR_RAW_WIDTH * COLOR_RAW_HEIGHT && base >= 0)
		{
			color_buff_K0[base * 3 + 0] = 245;
			color_buff_K0[base * 3 + 1] = 0;
			color_buff_K0[base * 3 + 2] = 0;
		}
	}

	for(int i = 0 ; i < imagePoints_K1.size(); i++)
	{
		int x = (int)(floor(imagePoints_K1[i].x + 0.5)) % 1920;
		int y = (int)(floor(imagePoints_K1[i].y + 0.5)) % 1080;
		if(objectPoints_K1[i].x > 200) continue;
		int base = y * COLOR_RAW_WIDTH +  x;
		if(base < COLOR_RAW_WIDTH * COLOR_RAW_HEIGHT && base >= 0)
		{
			color_buff_K1[base * 3 + 0] = 0;
			color_buff_K1[base * 3 + 1] = 0;
			color_buff_K1[base * 3 + 2] = 245;
		}
	}

	PNG_IO::BYTE_array_to_PNG(color_buff_K0.data(), string(File_Mesh_Output_png + "_K0.png").c_str(), COLOR_RAW_WIDTH, COLOR_RAW_HEIGHT);
	PNG_IO::BYTE_array_to_PNG(color_buff_K1.data(), string(File_Mesh_Output_png + "_K1.png").c_str(), COLOR_RAW_WIDTH, COLOR_RAW_HEIGHT);
}


int upperbound(int c)
{
	if(c < 256)
		return 256;
	else if( c < 384)
		return 384;
	else if( c < 512)
		return 512;
	else if(c < 768)
		return 768;
	else
		return 1024;
}

void estmateGroundParam()
{
	vector<Point3f> groundpoints;
	Log::LoadPC_Point3f(FILE_GROUND_PLY.c_str(), &groundpoints);
	Point4f PCA_X;
	Geometry_Helper::PointClouldPCA(groundpoints, PCA_X);

	ground_plane_param.x = PCA_X.X();
	ground_plane_param.y = PCA_X.Y();
	ground_plane_param.z = PCA_X.Z();
	ground_plane_param.w = PCA_X.W();

	Point3f N(ground_plane_param.x, ground_plane_param.y, ground_plane_param.z);
	if(Point3f(1,0,0).dot(N) < 0 )
	{
		ground_plane_param.x = - PCA_X.X();
		ground_plane_param.y = - PCA_X.Y();
		ground_plane_param.w = - PCA_X.W();
	}

	std::cout<<ground_plane_param.x<<"\t"
		<<ground_plane_param.y<<"\t"
		<<ground_plane_param.z<<"\t"
		<<ground_plane_param.w<<endl;
}

void initParameters()
{
	/////////////////////
	//PARAMETER GET BY CALIBRATION
	/////////////////////

	K_K0 = loo::ImageIntrinsics(363.764865, 362.678369, 253.729154, 205.863793 );
	D_K0 = loo::distortParam(0.091550, -0.243064, 0.002042, -0.001558, 0.044704 );
	CK_K0 = loo::ImageIntrinsics(1056.648650, 1054.186716, 944.829956, 533.429269 );
	CD_K0 = loo::distortParam(0.042811, -0.030363, -0.000417, 0.000657, -0.013007 );
	Mat_extr_ir_rgb_K0<< 0.999958, 0.005512, 0.007327, 0.053539, -0.005473, 0.999971, -0.005275, -0.004906, -0.007355, 0.005234, 0.999959, 0.003090, 0.000000, 0.000000, 0.000000, 1.000000 ;



	K_K1 = loo::ImageIntrinsics(366.304323, 365.511333, 255.672696, 205.769797 );
	D_K1 = loo::distortParam(0.092628, -0.262820, -0.000102, -0.000100, 0.070204 );
	CK_K1 = loo::ImageIntrinsics(1066.200437, 1063.336068, 976.353967, 525.870975 );
	CD_K1 = loo::distortParam (0.039916, -0.029472, -0.000722, 0.000289, -0.010327 );
	Mat_extr_ir_rgb_K1<< 0.999984, 0.003817, -0.004244, 0.052571,  -0.003798, 0.999983, 0.004376, -0.003229, 0.004261, -0.004360, 0.999981, -0.008469, 0.000000, 0.000000, 0.000000, 1.000000;
	//June 31 2016
	//Extrinstic_ir_ir<<0.999797, 0.013630, 0.014816, 631.609,	-0.013410, 0.999801, -0.014797, -1.914,		-0.015014, 0.014596, 0.999781, -5.209,		0.000000, 0.000000, 0.000000, 1.000000;
	//Sept 8 2016
	Extrinstic_ir_ir<<0.999806, 0.013688, 0.014151, 633.551,	-0.013482, 0.999803, -0.014571, -3.696,		-0.014347, 0.014378, 0.999794, -3.254,		0.000000, 0.000000, 0.000000, 1.000000;
	//office before group test
	//Extrinstic_ir_ir<<0.999854, 0.011045, -0.013000, 638.635, -0.011146, 0.999908, -0.007720, -3.478, 0.012914, 0.007864, 0.999886, 11.630, 0.000000, 0.000000, 0.000000, 1.000000;
	Mat4 Refine;
	Refine.setIdentity();
	//Refine<< 0.99995, -0.000385249, 0.00997878, -13.3116,	0.000551133, 0.999862, -0.0166264, 13.3108,	-0.00997099, 0.016631, 0.999812, -0.515738, 0, 0, 0, 1;
	Extrinstic_ir_ir = Extrinstic_ir_ir * Refine;


	ICP_refine.setIdentity();

	//////////////////////////////
	//Inialize depth image buffer
	//////////////////////////////
	for(int k = 0 ; k < FRAME_NUM ; k++)
	{
		depth_buff_K0.at(k).resize(DEPTH_RAW_WIDTH * DEPTH_RAW_HEIGHT);
		depth_buff_K1.at(k).resize(DEPTH_RAW_WIDTH * DEPTH_RAW_HEIGHT);
	}
	color_buff_K0.resize(COLOR_RAW_WIDTH * COLOR_RAW_HEIGHT * 3);
	color_buff_K1.resize(COLOR_RAW_WIDTH * COLOR_RAW_HEIGHT * 3);
	//////////////////////////////
	//Set Camera Pose
	//////////////////////////////
	World_Camera_K0.setIdentity();
	World_Camera_K1.setIdentity();
	World_Camera_K1.block<3,3>(0,0) = Extrinstic_ir_ir.block<3,3>(0,0) * World_Camera_K0.block<3,3>(0,0);
	World_Camera_K1.block<3,1>(0,3) = Extrinstic_ir_ir.block<3,1>(0,3) + World_Camera_K0.block<3,1>(0,3); 
	T_tsdf_cw_K0.SetIdentity();
	T_tsdf_cw_K1.SetIdentity();

	std::cout<<"World_Camera_K0"<<endl<<World_Camera_K0<<endl;
	std::cout<<"World_Camera_K1"<<endl<<World_Camera_K1<<endl;
}

void generateMesh_AdaptVolume(	CMeshO& Mesh, 
							  const vector<vector<float>>* buff_K0, 
							  const vector<vector<float>>* buff_K1, 
							  const int& vox_per_meter, 
							  const float3& volume_center, 
							  const loo::BoundingBox& bbox,
							  const loo::Mat<float,3,4>& Loo_tsdf_cw_K0,
							  const loo::Mat<float,3,4>& Loo_tsdf_cw_K1
							  )
{
	int3 volres_adp = make_int3(
		(int)ceil((bbox.Max().x - bbox.Min().x) * 0.001 * vox_per_meter / 32.0) * 32,
		(int)ceil((bbox.Max().y - bbox.Min().y) * 0.001 * vox_per_meter / 32.0) * 32,
		(int)ceil((bbox.Max().z - bbox.Min().z) * 0.001 * vox_per_meter / 32.0) * 32
		);

	loo::BoundingBox bbox_adp;
	bbox_adp.Max() = make_float3(
		(float) volres_adp.x * 1000. / (2.0 * vox_per_meter) + (float)volume_center.x,
		(float) volres_adp.y * 1000. / (2.0 * vox_per_meter) + (float)volume_center.y,
		(float) volres_adp.z * 1000. / (2.0 * vox_per_meter) + (float)volume_center.z
		);
	bbox_adp.Min() = make_float3(
		-(float) volres_adp.x * 1000. / (2.0 * vox_per_meter) + (float)volume_center.x,
		-(float) volres_adp.y * 1000. / (2.0 * vox_per_meter) + (float)volume_center.y,
		-(float) volres_adp.z * 1000. / (2.0 * vox_per_meter) + (float)volume_center.z
		);

	assert(volres_adp.x >= 0 && volres_adp.x <= 1024);
	assert(volres_adp.y >= 0 && volres_adp.y <= 1024);
	assert(volres_adp.z >= 0 && volres_adp.z <= 1024);
	loo::BoundedVolume<loo::SDF_t, loo::TargetDevice, loo::Manage> vol_adp(volres_adp.x, volres_adp.y, volres_adp.z, bbox_adp);

	const float trunc_dist = trunc_dist_factor * length(vol_adp.VoxelSizeUnits());

	loo::SdfReset(vol_adp, trunc_dist);

	CMeshO meshK;

	if(buff_K0 && !buff_K1)
	{
		for(int i = 0 ; i < buff_K0->size() ; i++)
		{
			memcpy(h_rawdepth_K0.ptr, depth_buff_K0.at(i).data(), DEPTH_RAW_HEIGHT * DEPTH_RAW_WIDTH * sizeof(float));
			runTSDF(vol_adp, h_rawdepth_K0, Loo_tsdf_cw_K0, D_K0, K_K0, 0);
		}
		MeshOutput(vol_adp, meshK, 2, 0.01); //MeshOutput(vol_adp, meshK, 5, 0.01);
	}

	if(!buff_K0 && buff_K1)
	{
		for(int i = 0 ; i < buff_K1->size() ; i++)
		{   
			memcpy(h_rawdepth_K1.ptr, depth_buff_K1.at(i).data(), DEPTH_RAW_HEIGHT * DEPTH_RAW_WIDTH * sizeof(float));
			runTSDF(vol_adp, h_rawdepth_K1, Loo_tsdf_cw_K1, D_K1, K_K1, 0);
		}
		MeshOutput(vol_adp, meshK, 2,  0.01); //MeshOutput(vol_adp, meshK, 5,  0.01);
	}

	if(buff_K0 && buff_K1)
	{
		assert(buff_K0->size() == buff_K1->size());
		for(int i = 0 ; i < buff_K0->size() ; i++)
		{
			memcpy(h_rawdepth_K0.ptr, depth_buff_K0.at(i).data(), DEPTH_RAW_HEIGHT * DEPTH_RAW_WIDTH * sizeof(float));
			runTSDF(vol_adp, h_rawdepth_K0, Loo_tsdf_cw_K0, D_K0, K_K0, 1);
			memcpy(h_rawdepth_K1.ptr, depth_buff_K1.at(i).data(), DEPTH_RAW_HEIGHT * DEPTH_RAW_WIDTH * sizeof(float));
			runTSDF(vol_adp, h_rawdepth_K1, Loo_tsdf_cw_K1, D_K1, K_K1, 2);
		}
		MeshOutput(vol_adp, meshK, 2, 0.05);
	}

	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(Mesh, meshK);
}

loo::BoundingBox estimateBounding(const vector<float>& depth_buff, const loo::ImageIntrinsics& K, const Mat4& IR_extr)
{
	loo::BoundingBox test_box;
	test_box.Max() = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	test_box.Min() = make_float3( FLT_MAX,  FLT_MAX,  FLT_MAX);
	for(int j = 0 ; j < DEPTH_RAW_HEIGHT ; j++)
	{
		for(int i = 0 ; i < DEPTH_RAW_WIDTH ; i++)
		{
			float x = i;
			float y = j;
			float z = depth_buff[j * DEPTH_RAW_WIDTH + i];

			if(z > 1900 || z < 500) continue;

			x -= K_K0.u0;
			y -= K_K0.v0;
			x /= K_K0.fu;
			y /= K_K0.fv;
			x = x * z;
			y = y * z;

			Vet3 p_c(x, y, z);
			Vet3 p_w = IR_extr.block<3,3>(0,0) * p_c + IR_extr.block<3,1>(0,3);
			test_box.Max().x = p_w.x() > test_box.Max().x ? p_w.x() : test_box.Max().x;
			test_box.Max().y = p_w.y() > test_box.Max().y ? p_w.y() : test_box.Max().y;
			test_box.Max().z = p_w.z() > test_box.Max().z ? p_w.z() : test_box.Max().z;
			test_box.Min().x = p_w.x() < test_box.Min().x ? p_w.x() : test_box.Min().x;
			test_box.Min().y = p_w.y() < test_box.Min().y ? p_w.y() : test_box.Min().y;
			test_box.Min().z = p_w.z() < test_box.Min().z ? p_w.z() : test_box.Min().z;
		}
	}

	//test_box.Max() += make_float3(100, 100, 0);
	//test_box.Min() -= make_float3(100, 100, 0);
	return test_box;	
}
inline Point3f PointBackProjection(loo::ImageIntrinsics& K, int& x, int& y, int& depth)
{
	float xx = x;
	float yy = y;
	float zz = depth;
	xx -= K_K0.u0;
	yy -= K_K0.v0;
	xx /= K_K0.fu;
	yy /= K_K0.fv;
	xx = xx * zz;
	yy = yy * zz;
	return Point3f(xx,yy,zz);
}
void init_GenerateMesh2(string File_Pose_0, string File_Pose_1, string File_Output_K0, string File_Output_K1, int k)
{
	vector<float> indepth(DEPTH_RAW_WIDTH * DEPTH_RAW_HEIGHT);
	vector<float> outdepth(DEPTH_RAW_WIDTH * DEPTH_RAW_HEIGHT);

	int x_start = 512;
	int clip_height_by_maxmin_depth = 0;
	for(int i = 0 ; i < FRAME_NUM; i++ )
	{
		memset(indepth.data(), 0, sizeof(float) * DEPTH_RAW_HEIGHT * DEPTH_RAW_WIDTH);
		memset(outdepth.data(), 0, sizeof(float) * DEPTH_RAW_HEIGHT * DEPTH_RAW_WIDTH);
		string s(File_Pose_0 + to_string(i) + ".png");
		if(k == 6 || k == 2)
		{
			PNG_IO::PNG_to_array_depth(s.c_str(), indepth.data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT, 0,DEPTH_RAW_WIDTH - 0 ,0,DEPTH_RAW_HEIGHT , true,  500, 1750  );	
		}
		else if(k == 0 || k == 4)
		{
			//PNG_IO::PNG_to_array_depth(s.c_str(), indepth.data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT, 30,DEPTH_RAW_WIDTH - 60 ,0,DEPTH_RAW_HEIGHT , true,  500, 1750);	
			PNG_IO::PNG_to_array_depth(s.c_str(), indepth.data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT, 60,DEPTH_RAW_WIDTH - 60 ,0,DEPTH_RAW_HEIGHT , true,  500, 1750 );	
		}
		else
		{
			PNG_IO::PNG_to_array_depth(s.c_str(), indepth.data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT, 60,DEPTH_RAW_WIDTH - 20 ,0,DEPTH_RAW_HEIGHT , true,  500, 1750 );	
		}

		std::cout<<"+";
		undistort(indepth.data(), outdepth.data(), K_K0, D_K0, 0, NULL, true, k); 

#if 1
		//clip face 2 and face 6 of the upper camera
		if(k == 2 || k == 6)
		{
			if(i == 0)
			{
				for(int x = 511 ; x > 0; x --)
				{
					for(int y = 100; y < 300; y ++)
					{
						int curr = outdepth[y * w + x];
						if( curr > 500 && curr < 2000)
						{
							x_start = x;
							break;
						}
					}
					if(x_start != 512)
						break;
				}
			}
			int thr = ( k == 6 || k == 2) ? 210: 210;
			//assume center exactly at 1250 mm away from the camera
			Concurrency::parallel_for(0u, (UINT) w * h, [&](UINT j){
				int x = j % w;
				//if(x > x_start - 160) return;
				if(x > x_start - thr) return;
				if(outdepth[j] < 500 || outdepth[j] > 2000)	return;
				if(k == 6 || k == 2)
				{
					if(outdepth[j] > 980  && x < 100)
					{
						outdepth[j] = UINT16_MAX;
						return;
					}
					if(outdepth[j] > 980 )
						outdepth[j] = UINT16_MAX;
				}
			});
		}
#endif
#if 1
		//clip face 1 and face 7 of the upper camera
		if(k == 1 || k == 7 || k == 3 || k == 5)
		{
			int minx = 0;
			int miny = 0;
			if(i == 0)
			{
				int min_depth = INT_MAX;
				int height = 0;
				for(int x = 0; x < 512 ; x +=3)
				{
					for(int y = 0; y < 424 ; y +=3)
					{
						int curr = outdepth[y * w + x];
						if( curr > 500 && curr < 2000)
						{
							if(curr < min_depth)
							{
								min_depth = curr;
								Point3f a = PointBackProjection(K_K0, x, y, curr);
								height = a.X();
								miny = y;
								minx = x;
							}
						}
					}
					if(x_start != 512)
						break;
				}

				if(k == 3 || k == 5)
					clip_height_by_maxmin_depth = height - 100;
				else 
					clip_height_by_maxmin_depth = height - 100;//50;
			}
			//assume center exactly at 1250 mm away from the camera
			Concurrency::parallel_for(0u, (UINT) w * h, [&](UINT j){
				int x = j % w;

				if(outdepth[j] < 500 || outdepth[j] > 2000)	return;
				int curr = outdepth[j];
				int y = j / w;
				Point3f a = PointBackProjection(K_K0, x, y, curr);

				/*	if(outdepth[j] > 1400 || (outdepth[j] < 1100 && x > 50))
				return;*/
				if(outdepth[j] > 1400   && x > 80)
					return;

				if(k == 1)
				{
					//if(outdepth[j] < 1110 && x < 120 && x > 10)		return;
					if(outdepth[j] < 1040    && x < 199 && x >= 10)
						return;
				}
				else if(k == 7)
				{
					//if(outdepth[j] < 1070 && x < 120 && x > 10)		return;
					if(outdepth[j] < 1030    && x < 185 && x >= 10)
						return;
				}

				if(outdepth[j] < 1000    && x < 150 && x > 10)
					return;
				//
				if(k != 7)
				{
					if(outdepth[j] < 1025    && x < 240 && x >= 100)
						return;
				}
				else 
				{
					if(outdepth[j] < 1030    && x < 240 && x >= 100 )
						return;
				}
				if(a.X() < clip_height_by_maxmin_depth ) //male 150
				{

					outdepth[j] = UINT_MAX;
				}
			});
		}
#endif
		PNG_IO::depth_array_to_PNG(outdepth.data(), string(File_Output_K0 + to_string(i) + ".png").c_str(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT);
	}


	Vector2i Up;
	Vector2i Lo;
	Vector2i Pv;
	Up.setZero();
	Lo.setZero();
	Pv.setZero();
	Lo.y() = 424;
	Vector2i keyPoint;
	float lk; 
	float lb;

	for(int i = 0 ; i < FRAME_NUM ; i++ )
	{
		memset(indepth.data(), 0, sizeof(float) * DEPTH_RAW_HEIGHT * DEPTH_RAW_WIDTH);
		memset(outdepth.data(), 0, sizeof(float) * DEPTH_RAW_HEIGHT * DEPTH_RAW_WIDTH);
		string s(File_Pose_1 + to_string(i) + ".png");
		auto ss = 10;
		if(k == 1 || k == 7)
			PNG_IO::PNG_to_array_depth(s.c_str(), indepth.data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT,40 + ss,DEPTH_RAW_WIDTH, 10, DEPTH_RAW_HEIGHT - 10, true, 500, 1650   );
		else if(k == 0 || k == 4)
			PNG_IO::PNG_to_array_depth(s.c_str(), indepth.data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT,40 + ss,DEPTH_RAW_WIDTH, 10, DEPTH_RAW_HEIGHT - 10, true, 500, 1750  );
		else if(k == 2 || k == 6)
			PNG_IO::PNG_to_array_depth(s.c_str(), indepth.data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT,20 + ss,DEPTH_RAW_WIDTH, 10, DEPTH_RAW_HEIGHT - 10, true, 500, 1750  );
		else 
			PNG_IO::PNG_to_array_depth(s.c_str(), indepth.data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT,40 + ss,DEPTH_RAW_WIDTH, 10, DEPTH_RAW_HEIGHT - 10, true, 300, 1750  );
		std::cout<<"+";
		undistort(indepth.data(), outdepth.data(), K_K1, D_K1, 0, NULL, false, k);

		//clip face 3 and face 5 of the lower camera
#if 1
		if(k == 3 || k == 5 || k == 7 || k == 1)
		{
			if(i == 0)
			{
				depthclean::unwanted_mask(outdepth.data(), k, Up, Lo, Pv);

				if(k == 3 || k == 7)
				{
					//Up = Vector2i(380, 179);
					keyPoint = Up;
					lk = tanf(12 * 3.14 / 180);
					lb = keyPoint.y() - lk * keyPoint.x();
				}
				else
				{
					keyPoint = Lo;
					lk = tanf(-15 * 3.14 / 180);
					lb = keyPoint.y() - lk * keyPoint.x();
				}
				cout<<lk<<"  "<<lb<<endl;
			}

			Concurrency::parallel_for(0u, (UINT) w * h, [&](UINT j){
				int x = j % w;
				int y = j / w;
				if(x < Pv.x())
					return;
				if((k == 7 || k == 1) && (outdepth[j] < 1000))
				{
					outdepth[j] = UINT16_MAX;
					return;
				}
				if(x < Pv.x() + 100 && outdepth[j] < 1360   && abs(y - Pv.y()) < 50)
					return;
				/*			if(outdepth[j] > 1400 && x > 256)
				{
				outdepth[j] = UINT16_MAX;
				return;
				}*/

				if((outdepth[j] > 1310   && x > 250) || x > 490 ||(outdepth[j] > 1350   && x <= 250))
				{
					outdepth[j] = UINT16_MAX;
					return;
				}

				if(k == 1)
				{
					keyPoint = Vector2i(512 - 100 - 20, 241);
					lb = keyPoint.y() - lk * keyPoint.x();
					if(keyPoint.x() < x && keyPoint.y() < y - 10)
					{
						outdepth[j] = UINT16_MAX;
						return;
					}
				}
				else if(k == 3)
				{
					keyPoint = Vector2i(512 - 108, 154);
					lb = keyPoint.y() - lk * keyPoint.x();
					if(keyPoint.x() < x && keyPoint.y() > y)
					{
						outdepth[j] = UINT16_MAX;
						return;
					}
				}
				else if(k == 5)
				{
					keyPoint = Vector2i(512 - 115 - 20, 232);
					lb = keyPoint.y() - lk * keyPoint.x();
					if(keyPoint.x() < x && keyPoint.y() < y)
					{
						outdepth[j] = UINT16_MAX;
						return;
					}
				}
				else
				{
					keyPoint = Vector2i(512 - 107 - 20 , 154);
					lb = keyPoint.y() - lk * keyPoint.x();
					if(keyPoint.x() < x && keyPoint.y() > y + 20)
					{
						outdepth[j] = UINT16_MAX;
						return;
					}
				}


				if( (k == 3 ) && (int) (x * lk + lb - y) >= 0 && y < keyPoint.y() + 10 )//&& x < keyPoint.x() + 20)
				{
					outdepth[j] = UINT16_MAX;
					return;
				}
				if( (k == 5 ) && (int) (x * lk + lb - y) <= 0 && y > keyPoint.y() + 10)// && x < keyPoint.x() + 20)
				{
					outdepth[j] = UINT16_MAX;
					return;
				}
				if( (k == 5 ) && x > 300 && outdepth[j] < 1025  )// && x < keyPoint.x() + 20)
				{
					outdepth[j] = UINT16_MAX;
					return;
				}

				if(k == 7 && outdepth[j] < 1000   )
				{
					outdepth[j] = UINT16_MAX;
					return;
				}
				if( (k == 7) && (int) (x * lk + lb - y) >= 0 && y < keyPoint.y() )// && x < keyPoint.x() + 20)
				{
					outdepth[j] = UINT16_MAX;
					return;
				}

				if( (k == 1) && (int) (x * lk + lb - y) <= 0 && y > keyPoint.y())// && x < keyPoint.x() + 20)
				{
					outdepth[j] = UINT16_MAX;

				}
			});
		}
#endif
		PNG_IO::depth_array_to_PNG(outdepth.data(), string(File_Output_K1 + to_string(i) + ".png").c_str(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT);
	}




	std::cout<<endl;
} 

void meshSubdivided(CMeshO& mesh, Color4b& Label);
void processMesh(string File_Pose_0, string File_Pose_1, string File_Pose_0_rgb, string File_Pose_1_rgb, string file_out_fus_png, string file_out_fus_ply, string File_Debug, int k)
{
	for(int i = 0; i < FRAME_NUM; i++)
	{
		PNG_IO::PNG_to_array_depth(string(File_Pose_0 + to_string(i) + ".png").c_str(), depth_buff_K0[i].data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT, 0, DEPTH_RAW_WIDTH , 0, DEPTH_RAW_HEIGHT, false, 500, 2000);
		std::cout<<"-";
		//depth_buff_K1
		PNG_IO::PNG_to_array_depth(string(File_Pose_1 + to_string(i) + ".png").c_str(), depth_buff_K1[i].data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT, 0, DEPTH_RAW_WIDTH, 0, DEPTH_RAW_HEIGHT, false, 500, 2000);
		std::cout<<"=";
	}
	cout<<endl;
	cout<<File_Pose_0_rgb<<endl;
	cout<<File_Pose_1_rgb<<endl;
	PNG_IO::PNG_to_array_BYTE(File_Pose_0_rgb.c_str(), color_buff_K0.data(), COLOR_RAW_WIDTH, COLOR_RAW_HEIGHT, true);
	PNG_IO::PNG_to_array_BYTE(File_Pose_1_rgb.c_str(), color_buff_K1.data(), COLOR_RAW_WIDTH, COLOR_RAW_HEIGHT, true);

	Timer timerr;
	T_tsdf_cw_K0.SetIdentity();
	CMeshO Mesh_K0, Mesh_K1;
	//Generate Mesh K0
	timerr.Start();
	loo::BoundingBox box_adp_K0 = estimateBounding(depth_buff_K0[0], K_K0, Mat4::Identity());
	float3 volcenter_K0 = make_float3(
		box_adp_K0.Max().x * 0.5 + box_adp_K0.Min().x * 0.5, 
		box_adp_K0.Max().y * 0.5 + box_adp_K0.Min().y * 0.5, 
		box_adp_K0.Max().z * 0.5 + box_adp_K0.Min().z * 0.5
		);
	generateMesh_AdaptVolume(Mesh_K0, &depth_buff_K0, NULL, 192, volcenter_K0, box_adp_K0, T_tsdf_cw_K0, T_tsdf_cw_K0);

	//string(file_out_fus_png + "_Up.png")
	textureMapping_SignleMesh(Mesh_K0, "",  color_buff_K0, Mat_extr_ir_rgb_K0,Mat4::Identity(), CK_K0, CD_K0);
	timerr.Print("Debug-Mesh0");

	//Generate Mesh K1
	timerr.Start();
	loo::BoundingBox box_adp_K1 = estimateBounding(depth_buff_K1[0], K_K1, Mat4::Identity());
	float3 volcenter_K1= make_float3(
		box_adp_K1.Max().x * 0.5 + box_adp_K1.Min().x * 0.5, 
		box_adp_K1.Max().y * 0.5 + box_adp_K1.Min().y * 0.5, 
		box_adp_K1.Max().z * 0.5 + box_adp_K1.Min().z * 0.5
		);
	generateMesh_AdaptVolume(Mesh_K1, NULL, &depth_buff_K1, 192, volcenter_K1, box_adp_K1, T_tsdf_cw_K0, T_tsdf_cw_K0);
	//string(file_out_fus_png + "_Dw.png")
	textureMapping_SignleMesh(Mesh_K1, "",  color_buff_K1, Mat_extr_ir_rgb_K1,Mat4::Identity(), CK_K1, CD_K1);
	timerr.Print("Debug-Mesh1");
	Log::LogMesh_CMeshO(&Mesh_K0, string(File_Debug + to_string(k) + "_Mesh_K0.ply").c_str(), true, true, true);
	Log::LogMesh_CMeshO(&Mesh_K1, string(File_Debug + to_string(k) + "_Mesh_K1.ply").c_str(), true, true, true);

	//ICP
	CMeshO Mesh_K1_Copy;
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(Mesh_K1_Copy, Mesh_K1);

	vcg::Matrix44f Tr_extr;
	Tr_extr.FromEigenMatrix(Mat4(Extrinstic_ir_ir.inverse()));
	vcg::tri::UpdatePosition<CMeshO>::Matrix(Mesh_K1, Tr_extr);


	if(k == 2 || k == 6)
	{
		tri::UpdateTopology<CMeshO>::FaceFace(Mesh_K0);
		std::pair<int,int> delInfo=tri::Clean<CMeshO>::RemoveSmallConnectedComponentsSize(Mesh_K0, Mesh_K0.vert.size() * 0.1);
		int unref2 = vcg::tri::Clean<CMeshO>::RemoveUnreferencedVertex(Mesh_K0);
		tri::UpdateNormal<CMeshO>::PerVertex(Mesh_K0);
		tri::UpdateNormal<CMeshO>::NormalizePerVertex(Mesh_K0);
	}
	Log::LogMesh_CMeshO(&Mesh_K0, string(File_Debug + to_string(k) + "_Mesh_K0.ply").c_str(), true, true, true);
	Log::LogMesh_CMeshO(&Mesh_K1, string(File_Debug + to_string(k) + "_Mesh_K1.ply").c_str(), true, true, true);

	icprefine.icpAlignPair(Mesh_K0, Mesh_K1, 40, 150, 1500, 500);
	//icpAlignPair(vcg::CMeshO& source, vcg::CMeshO& target, int max_iter, float Min_Distance = 50, float Sample_size = 1500, float thr_color = 173)


	Mat4 ICP_ICP;
	Mesh_K0.Tr.ToEigenMatrix(ICP_ICP);

	float refine_thr = 150;
	if(abs(ICP_ICP(0, 3)) > refine_thr || abs(ICP_ICP(1, 3)) > refine_thr || abs(ICP_ICP(2, 3)) > refine_thr)
	{
		ICP_ICP.setIdentity();
	}
	Mat4 World_Camera_K1_refine = World_Camera_K1 * ICP_ICP;

	for(int r = 0 ; r < 3 ; r ++)
	{
		for(int c = 0 ; c < 4 ; c++)
		{
			T_tsdf_cw_K1(r,c) = World_Camera_K1_refine(r,c);
		}
	}

	Tr_extr.FromEigenMatrix(Mat4(World_Camera_K1_refine.inverse()));
	vcg::tri::UpdatePosition<CMeshO>::Matrix(Mesh_K1_Copy, Tr_extr);

	Mesh_K0.Tr.SetIdentity();
	Mesh_K1_Copy.Tr.SetIdentity();

	Log::LogMesh_CMeshO(&Mesh_K1_Copy, string(File_Debug + to_string(k) + "_Mesh_K1_ICP.ply").c_str(), true, true, true);

	//Log::LogMesh_CMeshO(&Mesh_K1_Copy, string(File_Debug + to_string(k) + "_Mesh_K1.ply").c_str(), true, true, true);
	//Fuse 2 Mesh
	for(int i = 0; i < FRAME_NUM; i++)
	{
		PNG_IO::PNG_to_array_depth(string(File_Pose_0 + to_string(i) + ".png").c_str(), depth_buff_K0[i].data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT, 0, DEPTH_RAW_WIDTH, 0, DEPTH_RAW_HEIGHT, false, 500, 2000);
	}
	timerr.Start();
	CMeshO Mesh_K;
	vcg::tri::UpdateBounding<CMeshO>::Box(Mesh_K0);
	vcg::tri::UpdateBounding<CMeshO>::Box(Mesh_K1_Copy);
	vcg::Box3f Box_K = Mesh_K0.bbox;
	Box_K.Add(Mesh_K1_Copy.bbox);
	loo::BoundingBox box_adp_K;
	box_adp_K.Max() = make_float3(Box_K.max.X(), Box_K.max.Y(), Box_K.max.Z());
	box_adp_K.Min() = make_float3(Box_K.min.X(), Box_K.min.Y(), Box_K.min.Z());
	float3 volcenter_K = make_float3(Box_K.Center().X(), Box_K.Center().Y(), Box_K.Center().Z());

	generateMesh_AdaptVolume(Mesh_K, &depth_buff_K0, &depth_buff_K1, 384, volcenter_K, box_adp_K, T_tsdf_cw_K0, T_tsdf_cw_K1);

	timerr.Print("Fusion-Mesh");

	if(k == 0)
	{
		string strtest(FUSFILE + "test.ply");
		Log::LogMesh_CMeshO(&Mesh_K, file_out_fus_ply.c_str(), true, true, false);
		Log::LoadMesh(file_out_fus_ply.c_str(), Mesh_K);
		//Processing;
		meshSubdivided(Mesh_K, (Color4b)Color4b::Magenta);
		Log::LogMesh_CMeshO(&Mesh_K, strtest.c_str(), true, true, false);
	}

	Mat4 goundUp;
	textureMapping(Mesh_K, file_out_fus_png, World_Camera_K1_refine, k);
	/////////////////////////
	PoseGroundUp(Mesh_K, ground_plane_param, &goundUp);
	Log::LogMesh_CMeshO(&Mesh_K, file_out_fus_ply.c_str(), true, true, false);
}

void meshSubdivided(CMeshO& mesh, Color4b& Label)
{
	vcg::tri::UpdateNormal<CMeshO>::PerVertex(mesh);
	CVertexO * vfirst = &mesh.vert[0];
	int num = mesh.face.size();
	vector<Point3i> oldFaces(mesh.face.size());
	vector<Point3i> newFaces;
	vector<CVertexO> newVerts;

	int kk = 0;
	for(int i = 0 ; i < num; i++)
	{
		CFaceO * fcurr = &mesh.face[i];
		oldFaces[i].X() = mesh.face[i].V(0) - mesh.vert.data();
		oldFaces[i].Y() = mesh.face[i].V(1) - mesh.vert.data();
		oldFaces[i].Z() = mesh.face[i].V(2) - mesh.vert.data();

		if(fcurr->V(0)->C().Equal(Label) && fcurr->V(1)->C().Equal(Label) && fcurr->V(2)->C().Equal(Label))
		{
			//Create and insert the new vertex
			CVertexO newVertex;
			newVertex.P() = (fcurr->V(0)->P() + fcurr->V(1)->P() + fcurr->V(2)->P()) / 3.;
			newVertex.N() = (fcurr->V(0)->N() + fcurr->V(1)->N() + fcurr->V(2)->N()) / 3.;
			newVertex.N().Normalize();
			newVertex.C() = Color4b::Gray150;
			newVerts.push_back(newVertex);

			newFaces.emplace_back(fcurr->V(1) - mesh.vert.data(), fcurr->V(2) - mesh.vert.data(), mesh.vert.size() + kk);
			newFaces.emplace_back(fcurr->V(2) - mesh.vert.data(), fcurr->V(0) - mesh.vert.data(), mesh.vert.size() + kk);

			CVertexO* ptr_Last = mesh.vert.data() + mesh.vert.size();
			fcurr->V(2) = ptr_Last;
			kk++;
		}
	}
	mesh.vert.reserve(newVerts.size() + mesh.vert.size());
	mesh.face.reserve(mesh.face.size() + newFaces.size());
	for(int i = 0; i < newVerts.size(); i++)
	{
		mesh.vert.push_back(newVerts[i]);
	}
	for(int i = 0; i < oldFaces.size(); i++)
	{
		mesh.face[i].V(0) = mesh.vert.data() + oldFaces[i].X();
		mesh.face[i].V(1) = mesh.vert.data() + oldFaces[i].Y();
		mesh.face[i].V(2) = mesh.vert.data() + oldFaces[i].Z();
	}
	for(int i = 0; i < newFaces.size(); i++)
	{
		CFaceO currf;
		currf.V(0) = &mesh.vert[newFaces[i].X()];
		currf.V(1) = &mesh.vert[newFaces[i].Y()];
		currf.V(2) = &mesh.vert[newFaces[i].Z()];
		mesh.face.push_back(currf);
	}
	mesh.vn = mesh.vert.size();
	mesh.fn = mesh.face.size();
}

void generateGround()
{
	vector<Point3f> groundpoints;
	for(int i = 0 ; i < FRAME_NUM; i++ )
	{
		vector<float> indepth(DEPTH_RAW_WIDTH * DEPTH_RAW_HEIGHT);
		PNG_IO::PNG_to_array_depth(string(GRUFILE + to_string(i) + ".png").c_str(), indepth.data(), DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT, 0, DEPTH_RAW_WIDTH - 0, 50, DEPTH_RAW_HEIGHT - 50, true, 600, 1500);
		std::cout<<"import depth image "<< i <<endl;
		depth_buff_K1[i].resize(DEPTH_RAW_WIDTH * DEPTH_RAW_HEIGHT);
		cvUndistort::cv_undistort(indepth.data(), depth_buff_K1[i].data(), K_K1, D_K1, DEPTH_RAW_WIDTH, DEPTH_RAW_HEIGHT);
	}

	T_tsdf_cw_K0.SetIdentity();
	CMeshO Mesh_Ground;
	loo::BoundingBox box_adp_Ground = estimateBounding(depth_buff_K1[0], K_K1, Mat4::Identity());
	float3 volcenter_K0 = make_float3(
		box_adp_Ground.Max().x * 0.5 + box_adp_Ground.Min().x * 0.5, 
		box_adp_Ground.Max().y * 0.5 + box_adp_Ground.Min().y * 0.5, 
		box_adp_Ground.Max().z * 0.5 + box_adp_Ground.Min().z * 0.5
		);
	generateMesh_AdaptVolume(Mesh_Ground, NULL, &depth_buff_K1, 384, volcenter_K0, box_adp_Ground, T_tsdf_cw_K0, T_tsdf_cw_K0);
	vcg::tri::Clean<CMeshO>::RemoveUnreferencedVertex(Mesh_Ground);
	vcg::tri::Clean<CMeshO>::RemoveDuplicateVertex(Mesh_Ground);
	Log::LogMesh_CMeshO(&Mesh_Ground, FILE_GROUND_PLY.c_str());
}

void boundaryCheck( CMeshO * m, vector<int> & boundary_idx);
void MeshProcessingCleanBoundary(CMeshO& ms_label, CMeshO& mt_label, int raidus);
int main(int argc, char** argv )
{
#if 1
	Timer timerrr;

	initParameters();
	//generateGround();
	vector<int> flag(8,1);
	//flag[4] = 1;
	//flag[5] = 0;
	//flag[1] = 1;
	//flag[2] = 1;
	//flag[2] = 1;
	//flag[0] = 0;
	//flag[0] = 1;
	//flag[6] = 1;
	////estmateGroundParam();
	ground_plane_param.x = 44.5617;
	ground_plane_param.y = -0.00309972;
	ground_plane_param.z = -1 - 0.4;
	ground_plane_param.w = 31766.5;
	for(int k = 0 ; k < 8 ; k +=1)
	{
		if(flag[k] == 0)
			continue;
		string str_idx_Ka1(to_string(k + 1));
		string str_idx_K(to_string(k));
		string file_depth_dir0(DEPTHK0 + "Pose" + str_idx_Ka1 + "_");
		string file_depth_dir1(DEPTHK1 + "Pose" + str_idx_Ka1 + "_");
		string file_depth_dir0_clean(DEPTHK0_C + "Pose" + str_idx_Ka1 + "_");
		string file_depth_dir1_clean(DEPTHK1_C + "Pose" + str_idx_Ka1 + "_");
		init_GenerateMesh2(file_depth_dir0, file_depth_dir1, file_depth_dir0_clean, file_depth_dir1_clean, k);
	}

	timerrr.Start();
	for(int k = 0 ; k < 8 ; k +=1)
	{
		if(flag[k] == 0)
			continue;
		std::cout<<"--------------------------------"<<k<<"--------------------------------"<<endl;
		string str_idx_Ka1(to_string(k + 1));
		string str_idx_K(to_string(k));
		string file_depth_dir0_clean(DEPTHK0_C + "Pose" + str_idx_Ka1 + "_");
		string file_depth_dir1_clean(DEPTHK1_C + "Pose" + str_idx_Ka1 + "_");
		string file_rgb_dir0(DEPTHK0 + "CPose" + str_idx_Ka1 + "_0.png"); 


		string file_rgb_dir1(DEPTHK1 + "CPose" + str_idx_Ka1 + "_0.png");
		string file_out_fus_ply(FUSFILE + str_idx_K + ".ply");
		string file_out_fus_png(FUSFILE + str_idx_K + "img");
		string file_out_debug = DBGFILE;
		processMesh(file_depth_dir0_clean, file_depth_dir1_clean, file_rgb_dir0, file_rgb_dir1, file_out_fus_png, file_out_fus_ply, file_out_debug, k);
	}
	timerrr.Print("fuse");

	//Init pose

#endif
	PoseInit Pinit;
	Pinit.init(FUSFILE);
	for(int k = 0; k < NUM_FACE; k++)
	{
		CMeshO temp;
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(temp, *Pinit._cmv.at(k));
		MeshProcessingCleanBoundary(temp, *Pinit._cmv.at(k), 0);
	}
	//clean the boundary	
	Pinit.poseProcess();
	for(int k = 0; k < NUM_FACE ; k++)
	{
		//vcg::tri::Clean<CMeshO>::RemoveSmallConnectedComponentsSize(*Pinit._cmv[k], Pinit._cmv[k]->vert.size() * 0.05);
		Log::LogMesh_CMeshO(Pinit._cmv[k], string(ICPFILE +to_string(k) + ".ply").c_str());
	}
	Log::LogGNdeformd( &Pinit.mat, string(ICPFILE + "Info_ICP_TR.txt").c_str());


	for(int k = 0; k < NUM_FACE; k++)
	{
		CMeshO tempm, tempm0;
		CMeshO tempi, tempi0;
		Log::LoadMesh(string(ICPFILE + to_string(k) + ".ply").c_str(), tempm0);
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(tempm, tempm0);
		//MeshProcessingCleanBoundary(tempm, tempm0, 10);
		//Log::LogMesh_CMeshO(&tempm, string(ICPFILE +to_string(k) + ".ply").c_str());

		vcg::tri::UpdateColor<CMeshO>::PerVertexConstant(tempm, Color4b::Gray150);
		Log::LogMesh_CMeshO(&tempm, string(ICPFILE + "Mark_"  +to_string(k) + ".ply").c_str());

	}
	//vector<CMeshO*> _cmvjoint(8);
	//for(int k = 0; k < NUM_FACE ; k++)
	//{
	//	_cmvjoint[k] = new CMeshO;
	//	Log::LoadMesh(string(ICPFILE +to_string(k) + ".ply").c_str() , *_cmvjoint[k]);
	//	vcg::tri::UpdateColor<CMeshO>::PerVertexConstant(*_cmvjoint[k], Color4b::Gray150);
	//	Log::LogMesh_CMeshO(_cmvjoint[k], string(ICPFILE + "Mark_" + to_string(k) + ".ply").c_str());
	//}

	//JointEstimate2 JE2;
	//CMeshO JointMapFront, JointMapBack;
	//Log::LoadMesh(string(ROOT + "f00_Scale_Jointmap.ply").c_str(), JointMapFront);
	//Log::LoadMesh(string(ROOT + "f40_Scale_Jointmap.ply").c_str(), JointMapBack);
	//JE2.init(&_cmvjoint, JointMapFront, JointMapBack);
	////JE2.AutoSegment_Simple();
	//JE2.AutoSegment_Complete();
	//JE2.LogCurrentStatus(ICPFILE);

	////Joint Estimate
	//JointEstimate JE;
	//JE.init(&_cmvjoint, argc, argv, false);
	//JE.estimateJoints(Color4b::Orange, Color4b::Yellow, Color4b::Magenta, Color4b::Blue, Color4b::LightPurple, Color4b::Cyan);
	//JE.estimateHands(Color4b::DarkDarkPurple, Color4b::DarkDarkBlue);
	//JE.estimateFeet(Color4b::DarkDarkRed, Color4b::DarkDarkGreen);

	//for(int k = 0; k < NUM_FACE ; k++)
	//{
	//	Log::LogMesh_CMeshO(JE._cm->at(k), string(ICPFILE + "Mark_"+to_string(k) + ".ply").c_str());
	//}
	//timerr.Print("all");
	return 0;

}


void MeshProcessingCleanBoundary(CMeshO& ms_label, CMeshO& mt_label, int raidus)
{
	Nanoknn nano_knn(ms_label);
	nano_knn.buildNano();
	vector<int> boundary_idx;
	boundaryCheck(&ms_label, boundary_idx);
	vcg::tri::UpdateFlags<CMeshO>::VertexClearB(ms_label);

	if(raidus != 0)
	{
		for(int i = 0 ; i < boundary_idx.size(); i++)
		{
			int idx = boundary_idx[i];
			vector<int> knnidx;
			nano_knn.findKNNNoSelf(ms_label.vert.at(idx), raidus, knnidx);
			ms_label.vert[idx].SetB();
			for(int i = 0 ; i < knnidx.size() ; i++)
			{
				ms_label.vert[knnidx[i]].SetB();
			}
		}
		Concurrency::parallel_for(0u, (UINT)ms_label.vert.size(), [&](UINT i){
			if(ms_label.vert[i].IsB())
			{
				if(ms_label.vert[i].P().Y() < 500 || ms_label.vert[i].C().Equal((Color4b)Color4b::Red) || ms_label.vert[i].C().Equal((Color4b)Color4b::Green))
				{
					return;
				}
				ms_label.vert[i].P().X() = sqrtf(-1);
				ms_label.vert[i].P().Y() = sqrtf(-1);
				ms_label.vert[i].P().Z() = sqrtf(-1);
			}
		});

		vcg::tri::UpdateFlags<CMeshO>::VertexClearB(ms_label);
		vcg::tri::Clean<CMeshO>::RemoveDegenerateVertex(ms_label);
	}
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(mt_label, ms_label);
}

void boundaryCheck( CMeshO * m, vector<int> & boundary_idx)
{
	cout<<"boundary check..."<<endl;
	vcg::tri::UpdateFlags<CMeshO>::Clear(*m);
	vcg::tri::UpdateTopology<CMeshO>::FaceFace(*m);
	vcg::tri::UpdateFlags<CMeshO>::VertexClearV(*m);
	boundary_idx.clear();
	boundary_idx.reserve(5000);
	CVertexO* first = m->vert.data();

	for(int k = 0 ; k < (*m).FN(); k++)
	{
		int a[3] = {0};
		if(face::IsBorder((*m).face[k],0)) //edge 0-1
		{
			if(!(*m).face[k].V(0)->IsV())
			{
				a[0] = 1;
				(*m).face[k].V(0)->SetV();
			}
			if(!(*m).face[k].V(1)->IsV())
			{
				a[1] = 1;
				(*m).face[k].V(1)->SetV();
			}
		}
		if(face::IsBorder((*m).face[k],1)) //edge 1-2
		{
			if(!(*m).face[k].V(2)->IsV())
			{
				a[2] = 1;
				(*m).face[k].V(2)->SetV();
			}
			if(!(*m).face[k].V(1)->IsV())
			{
				a[1] = 1;
				(*m).face[k].V(1)->SetV();
			}
		}
		if(face::IsBorder((*m).face[k],2)) //edge 2-0
		{
			if(!(*m).face[k].V(2)->IsV())
			{
				a[2] = 1;
				(*m).face[k].V(2)->SetV();
			}
			if(!(*m).face[k].V(0)->IsV())
			{
				a[0] = 1;
				(*m).face[k].V(0)->SetV();
			}
		} 
		if(a[0])
			boundary_idx.push_back((*m).face[k].V(0) - first);
		if(a[1])
			boundary_idx.push_back((*m).face[k].V(1) - first);
		if(a[2])
			boundary_idx.push_back((*m).face[k].V(2) - first);
	}
}