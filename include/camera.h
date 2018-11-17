#ifndef CAMERA_H
#define CAMERA_H

#include "common_include.h"
#include "vo_core/orbforsys.h"
//存储必要的相机参数信息，包括相机去畸变变换矩阵(包括了顶部两个相机），相机去畸变后内参(包括了顶部两个相机），各相机之间位姿关系(包括了顶部两个相机），双目之间的关系（不包括顶部两个相机）
//相机个数按对数算，所有函数中的camnum只考虑每一对里面主相机的情况
namespace mixmap
{
	class Camera
	{
	public:
		typedef std::shared_ptr<Camera> Ptr;
		enum stereotype {
        HORIZONTAL = 0,
        VERTICAL = 1
		};
		stereotype camtype_;
		int camerapair_;
		cv::Size2i imgsize_;
		cv::Size2i imgsize_ori_;
		vector<OMNI_CAMERA_PARA> CamInPara_;
		vector<Mat> map1_;
		vector<Mat> map2_;                      //the map for undistort and 0,5 is 1st pair
		vector<Mat> OptCamInPara_;
		vector<SE3> T_0_c_;						//this is the SE3 for point in cam n coordinate change to the cam0 coordinate
		OMNI_CAMERA_PARA cameraup_;
		SE3 T_0_5_;
		
		vector<SO3> R_m_;                      //the inverse of the rotation of main camera pixel
		vector<double> baseline_;

		double scale_;
		int max_distance_;
		
		vector<Mat> maskforfeature_;
		//vector<vector<vector<Vector3d>>> p_oc_;
		Camera( stereotype camtype, int camerapair, vector<Mat> map1, vector<Mat> map2, vector<SE3> T_0_c, vector<SO3> R_m, vector<double> baseline,
				double scale, int max_distance, vector<Mat> OptCamInPara, vector<OMNI_CAMERA_PARA> CamInPara, cv::Size2i imgsize, cv::Size2i imgsize_ori,
				OMNI_CAMERA_PARA cameraup, SE3 T_0_5, vector<Mat> mask);		
		
		//to cam指的是到该像素（空间点）所属镜头的相机系
		//Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
		Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w, int camnum );
		
		Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w, int camnum );
		
		Vector2d camera2pixel( const Vector3d& p_c, int camnum );
		Vector2d camera_r2pixel( const Vector3d& p_c_r, int camnum );
		
		Vector3d pixel2camera( const Vector2d& p_p,  int camnum, double depth = 1 );
		
		Vector2d world2pixel( const Vector3d& p_w, const SE3& T_c_w, int camnum );
		
		Vector3d pixel2world( const Vector2d& p_p, const SE3& T_c_w, int camnum, double depth =1 );
		//vector<bool> checkPoint3d( vector<Vector3d> p_w, const SE3& T_c_w, int camnum );
		//void initial2Dmap();
	};
	

}

#endif