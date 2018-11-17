#ifndef FRAME_H
#define FRAME_H

#include "common_include.h"
#include "camera.h"

namespace vocore
{
	class Mappoint3d;
}
//以0号相机坐标系为整体坐标系
namespace mixmap
{
	class Frame
	{
	public:
		typedef std::shared_ptr<Frame> Ptr;
		unsigned long id_;
		double time_stamp_;
		//相机系经过T_c_w变为世界系
		SE3 T_c_w_; 
		Camera::Ptr camera_;
		vector<Mat> undistimage_;
		vector<Mat> srcimg_;
		bool is_key_frame_;
		
		vector<Mat> depth_;
		vector<Mat> depth_conv_;
		
	public:
		Frame();
		Frame( unsigned long id, double timestamp = 0, SE3 T_c_w = SE3(),Camera::Ptr camera=nullptr );
		~Frame();
		
		static Frame::Ptr createFrame(); 
		
		// Get Camera Center
		Vector3d getCamCenter( int camnum ) const;
		
		void setPose( const SE3& T_c_w );	
		
		bool isInFrame( const Vector3d& pt_world );
		
	public:
		cv::Ptr<cv::StereoSGBM> mSGBM;
		
	public:
		cv::Mat GetDepthImage();
		
		
	};
}

#endif