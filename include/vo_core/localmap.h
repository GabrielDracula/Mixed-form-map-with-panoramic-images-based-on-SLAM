#ifndef LOCALMAP_H
#define LOCALMAP_H

#include "common_include.h"
#include "frame.h"
#include "vo_core/mappoint3d.h"

namespace vocore
{
	class LocalMap
	{
	public:
		typedef shared_ptr<LocalMap> Ptr;
		unordered_map<unsigned long, MapPoint3d::Ptr >  map_points_;        // all landmarks
		unordered_map<unsigned long, mixmap::Frame::Ptr >     keyframes_;         // all key-frames

		LocalMap() {}
    
		void insertKeyFrame( mixmap::Frame::Ptr frame );
		void insertMapPoint( MapPoint3d::Ptr map_point );
	};
}

#endif