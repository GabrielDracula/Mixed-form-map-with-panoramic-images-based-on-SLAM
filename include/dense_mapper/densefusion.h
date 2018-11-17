#ifndef DENSEFUSION_H
#define DENSEFUSION_H

#include "common_include.h"
#include "camera.h"

namespace dense
{
	class DenseFusion
	{
	public:
		typedef std::shared_ptr<DenseFusion> Ptr;
		
 		vector<Mat> mImage_Ref;
		float mScale_;
		
		vector<Mat> mDepth_est;
		vector<Mat> mDepth_obs;
		vector<Mat> mDepth_final;
		
		vector<double> bf;
		vector<Mat> mSigma2_est;
		vector<Mat> mSigma2_obs;
		
		float mc_radius;
		float mc_pixels;
		
		float ma_init;
		float mb_init;
		vector<Mat> ma;
		vector<Mat> mb;
		
		vector<Mat> mDepthmap_conv;
		vector<Mat> mDepthmap_div;
		
		mixmap::Camera::Ptr camera_;
		
 		double mDisplacementThres;
		
		float mSigma2Thres;
		float mInliersThres;
		float mOutliersThres;
		
 		SE3 T_Ref_w;
		cv::Size imgsize_;
		
		long n_conv;
		long n_div;
		long minus_depth;
		long outofimage;
		
		float min_sigma2;
		float max_rho;
		
		Mat mStatus;
	public:
		DenseFusion( double scale, mixmap::Camera::Ptr camera, double displacementThres, cv::Size imgsizeScale );
		~DenseFusion() {}
		
		bool checkNewRef( SE3 T_c_w );
		vector<Mat> GetSigma2FromDepthmap( vector<Mat> DepthImg );
		void InitRefInfo( vector<Mat> Image_Ref, vector<Mat> Depth_Ref, SE3 T_c_w );
		
		void ProjectDepthToRef( vector<Mat> DepthImg, vector<Mat> Sigma2, SE3 T_c_w );
		
		void updateDepth( vector<Mat> Depth_obs, SE3 T_c_w );
		
	};
}

#endif