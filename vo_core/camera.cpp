#include "camera.h"

namespace mixmap
{
	Camera::Camera( stereotype camtype, int camerapair, vector<Mat> map1, vector<Mat> map2, vector<SE3> T_0_c, vector<SO3> R_m, vector<double> baseline,
					double scale, int max_distance, vector<Mat> OptCamInPara, vector<OMNI_CAMERA_PARA> CamInPara, cv::Size2i imgsize, cv::Size2i imgsize_ori,
				    OMNI_CAMERA_PARA cameraup, SE3 T_0_5, vector<Mat> mask):
	camtype_(camtype), camerapair_(camerapair), map1_(map1), map2_(map2), T_0_c_(T_0_c), R_m_(R_m), baseline_(baseline), 
	scale_(scale), max_distance_(max_distance), OptCamInPara_(OptCamInPara), CamInPara_(CamInPara), imgsize_(imgsize), imgsize_ori_(imgsize_ori),
	cameraup_(cameraup), T_0_5_(T_0_5), maskforfeature_(mask)
	{}
	
	Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3& T_c_w, int camnum )
	{
		return T_0_c_[camnum].inverse() * T_c_w * p_w;
	}
	Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3& T_c_w, int camnum )
	{
		return T_c_w.inverse() * T_0_c_[camnum] * p_c;
	}
	Vector2d Camera::camera2pixel( const Vector3d& p_c, int camnum )
	{
		Vector3d p_c_r = R_m_[camnum] * p_c;
		return Vector2d (
               OptCamInPara_[camnum].at<double>(0,0) * p_c_r ( 0,0 ) / p_c_r ( 2,0 ) + OptCamInPara_[camnum].at<double>(0,2),
               OptCamInPara_[camnum].at<double>(1,1) * p_c_r ( 1,0 ) / p_c_r ( 2,0 ) + OptCamInPara_[camnum].at<double>(1,2)
			   );
	}
	Vector2d Camera::camera_r2pixel( const Vector3d& p_c_r, int camnum )
	{
		return Vector2d (
               OptCamInPara_[camnum].at<double>(0,0) * p_c_r ( 0,0 ) / p_c_r ( 2,0 ) + OptCamInPara_[camnum].at<double>(0,2),
               OptCamInPara_[camnum].at<double>(1,1) * p_c_r ( 1,0 ) / p_c_r ( 2,0 ) + OptCamInPara_[camnum].at<double>(1,2)
			   );
	}
	
	
	Vector3d Camera::pixel2camera( const Vector2d& p_p, int camnum, double depth )
	{
		Vector3d p_c_r = Vector3d (	( p_p ( 0,0 )-OptCamInPara_[camnum].at<double>(0,2) ) *depth/ OptCamInPara_[camnum].at<double>(0,0),
									( p_p ( 1,0 )-OptCamInPara_[camnum].at<double>(1,2) ) *depth/ OptCamInPara_[camnum].at<double>(1,1),
									depth);
		return R_m_[camnum].inverse()*p_c_r;
	}
	
	Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3& T_c_w, int camnum)
	{
		return camera2pixel ( world2camera(p_w, T_c_w, camnum), camnum );
	}

	Vector3d Camera::pixel2world( const Vector2d& p_p, const SE3& T_c_w,  int camnum, double depth)
	{
 		return camera2world ( pixel2camera ( p_p, camnum, depth), T_c_w, camnum );
	}
	
}