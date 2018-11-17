#ifndef DENSEMATCHER_H
#define DENSEMATCHER_H

#include "camera.h"
#include "common_include.h"

namespace dense
{
	class DenseMatcher
	{
	public:
		typedef std::shared_ptr<DenseMatcher> Ptr;
		mixmap::Camera::Ptr camera_;
		
		cv::Ptr<cv::StereoSGBM> mSGBM = cv::StereoSGBM::create();;
		//imgLeft_crop、imgRight_crop、mDepthImage是经过了resize的
		vector<Mat> imgLeft_grey;
		vector<Mat> imgRight_grey;
		float mScale_;
		vector<double> mDisp_thres;
		vector<Mat> maskfordepth;
		
		vector<Mat> mDispImage;
		vector<Mat> mDispShow;
		vector<Mat> mDepthImage;
		
		long n_succ;
	public:
		DenseMatcher( int SADWindowSize, int cn, mixmap::Camera::Ptr camera, float mScale, int distance );
		
		void ComputeDisparity( vector<Mat> imgLeft, vector<Mat> imgRight);
	};
}


#endif