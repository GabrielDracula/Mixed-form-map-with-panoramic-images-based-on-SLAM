#ifndef DENSEMAPPER_H
#define DENSEMAPPER_H

#include "common_include.h"
#include "frame.h"
#include "dense_mapper/densematcher.h"
#include "dense_mapper/densefusion.h"

namespace dense
{
	class DenseMapper
	{
	public:
		typedef std::shared_ptr<DenseMapper> Ptr;
		mixmap::Camera::Ptr camera_;
		DenseMatcher::Ptr densematcher_;
		DenseFusion::Ptr densefusion_;
		int numofref_;
		
		vector<Mat> Depth_Conv_before;
				
		mixmap::Frame::Ptr curr_;
		PointCloud::Ptr densepointcloud_;
		
		double gridsize;
		bool First_;
	public:
		DenseMapper(int SADWindowSize, int cn, mixmap::Camera::Ptr camera, float mScale, int distance, cv::Size imgsize );
		
		bool FrameDenseMap( mixmap::Frame::Ptr frame );
		void GenerateDensePointCloud( vector<Mat> imgmain_, vector<Mat> imgdepth_, float mScale );

	};
}

#endif