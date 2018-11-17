/*
coordinate:
omnicamera coordinate has the same rotation with the 0 camera,its origin is the average of the optic center of all camera
*/
#ifndef MAPPOINT2D_H
#define MAPPOINT2D_H

#include "common_include.h"
#include "camera.h"

namespace map2d
{
	class Map2DSence
	{
	public:
		typedef std::shared_ptr<Map2DSence> Ptr;
		
		int camnum_;
		cv::Size2i imgsize_;
		
		vector<Mat> Color_;//前5个是校正后的侧面图，第6个是未校正的顶图
		vector<Mat> Depth_Conv_;
		
		SE3 T_w_c_;
		vector<Vector3d> neighbor_location_;
		int distance_;
		
		vector<vector<vector<Vector3d>>> projection_cam_;
		
	public:
		Map2DSence(vector<vector<vector<Vector3d>>> projection_init, vector<Mat> Color, SE3 T_c_w, mixmap::Camera::Ptr camera);
		
		void CheckandDelete();//according to the neighbor_pos_ , we delete the point other camera can find in a little depth
		
		PointCloud::Ptr GeneratePointCloud( int num );//generate the point cloud of this sence
	};
	
}

#endif