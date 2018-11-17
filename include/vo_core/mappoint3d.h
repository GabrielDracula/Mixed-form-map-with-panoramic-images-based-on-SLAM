#ifndef MAPPOINT3D_H
#define MAPPOINT3D_H

#include "common_include.h"

namespace mixmap
{
	class Frame;
}
namespace vocore
{
	
	class MapPoint3d
	{
	public:
		typedef std::shared_ptr<MapPoint3d> Ptr;
		unsigned long id_; // ID
		static unsigned long factory_id_;    // factory id
		bool good_;      // wheter a good point 
		Vector3d pos_;       // Position in world
		Vector3d norm_;      // Normal of viewing direction
		Mat descriptor_; // Descriptor for matching
		
		list<mixmap::Frame*>    observed_frames_;   // frames that can observe this point 
		
		int         matched_times_;     // being an inliner in pose estimation
		int         visible_times_;     // being visible in current frame 
	public:
		MapPoint3d();
		MapPoint3d( 
			unsigned long id, 
			const Vector3d& position, 
			const Vector3d& norm, 
			mixmap::Frame* frame=nullptr, 
			const Mat& descriptor=Mat() 
		);

		inline cv::Point3f getPositionCV() const {
			return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
		}
    
		static MapPoint3d::Ptr createLocalMapPoint();
		static MapPoint3d::Ptr createLocalMapPoint( 
			const Vector3d& pos_world, 
			const Vector3d& norm_,
			const Mat& descriptor,
			mixmap::Frame* frame );
	};
}

#endif