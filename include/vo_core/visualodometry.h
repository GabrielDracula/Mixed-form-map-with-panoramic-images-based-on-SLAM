#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "common_include.h"
#include "vo_core/localmap.h"
#include "camera.h"
#include "vo_core/orbforsys.h"

namespace vocore
{
	class VisualOdometry
	{
	public:
		typedef std::shared_ptr<VisualOdometry> Ptr;
		enum VOState{
			INITIALIZING = -1,
			OK = 0,
			LOST
		};
		VOState state_;
		LocalMap::Ptr localmap_;
		
		mixmap::Camera::Ptr camera_;
		mixmap::Frame::Ptr ref_key_;
		mixmap::Frame::Ptr curr_;
		
		ORBforVO::Ptr orb_vo_;
		vector<vector<cv::KeyPoint>> keypoints_curr_;
		vector<Mat> descriptor_curr_;
		
		vector<vector<MapPoint3d::Ptr>> match_3dpts_;
		vector<vector<int>> match_2dkp_index_;
		
		int max_match_cam_;
		SE3 T_c_w_estimated_;
		SE3 T_c_w_last_;
		int num_inliers_;
		int num_lost_;
		
		int max_num_lost_;
		int min_inliers_;
		double key_frame_min_rot;
		double key_frame_min_trans;
		double map_point_erase_ratio_;
		
		ofstream posefile;

	public:
		VisualOdometry();
		~VisualOdometry()
		{}
		bool addFrame( mixmap::Frame::Ptr frame );
		
	protected:
		void extractKeyPoints();
		void computeDescriptors();
		void featureMatching_Stereo_add();
		
		void featureMatching();
		void poseEstimationPnP(); 
		void optimizeMap();
    
		void addKeyFrame();
		void addMapPoints();
		bool checkEstimatedPose(); 
		bool checkKeyFrame();
    
		//double getViewAngle( Frame::Ptr frame, MapPoint3d::Ptr point );
	};
}

#endif