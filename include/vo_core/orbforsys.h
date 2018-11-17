#ifndef ORBFORVO_H
#define ORBFORVO_H

#include "common_include.h"
#include "config.h"
#include "opencv2/features2d/features2d.hpp"

namespace vocore
{
	class ORBforVO
	{
	public:
		typedef std::shared_ptr<ORBforVO> Ptr;
		cv::Ptr<cv::ORB> orb_;
		
		int num_of_features_;
		double scale_factor_;
		int level_pyramid_;
		float match_ratio_;
		
		cv::FlannBasedMatcher matcher_flann_;
	public:
		ORBforVO():
		matcher_flann_( new cv::flann::LshIndexParams ( 5,10,2 ) )
		{
			num_of_features_ = mixmap::Config::get<int>( "num_of_features" );
			scale_factor_ = mixmap::Config::get<double>( "scale_factor" );
			level_pyramid_ = mixmap::Config::get<int>( "level_pyramid" );
			match_ratio_ = mixmap::Config::get<float>( "match_ratio" );
			orb_ = cv::ORB::create( num_of_features_, scale_factor_, level_pyramid_ );
		}
		~ORBforVO()
		{}
	};
}

#endif