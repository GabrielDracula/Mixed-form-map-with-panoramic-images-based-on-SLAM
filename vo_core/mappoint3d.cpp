#include "common_include.h"
#include "vo_core/mappoint3d.h"

namespace vocore
{
	MapPoint3d::MapPoint3d()
	: id_(-1), pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)), good_(true), visible_times_(0), matched_times_(0)
	{}

	MapPoint3d::MapPoint3d ( long unsigned int id, const Vector3d& position, const Vector3d& norm, mixmap::Frame* frame, const Mat& descriptor )
	: id_(id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor)
	{
		observed_frames_.push_back(frame);
	}

	MapPoint3d::Ptr MapPoint3d::createLocalMapPoint()
	{
		return MapPoint3d::Ptr( 
			new MapPoint3d( factory_id_++, Vector3d(0,0,0), Vector3d(0,0,0) )
		);
	}

	MapPoint3d::Ptr MapPoint3d::createLocalMapPoint ( 
		const Vector3d& pos_world, 
		const Vector3d& norm, 
		const Mat& descriptor, 
		mixmap::Frame* frame )
	{
		return MapPoint3d::Ptr( 
			new MapPoint3d( factory_id_++, pos_world, norm, frame, descriptor )
		);
	}

	unsigned long MapPoint3d::factory_id_ = 0;
}