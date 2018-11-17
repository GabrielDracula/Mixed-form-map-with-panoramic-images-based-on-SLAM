#ifndef MAP2D_H
#define MAP2D_H

#include "common_include.h"
#include "camera.h"
#include "map2d/map2Dsence.h"

namespace map2d
{
	class Map2Dall
	{
	public:
		typedef std::shared_ptr<Map2Dall> Ptr;
		mixmap::Camera::Ptr camera_;
		vector<Map2DSence::Ptr>  map_sences_;
		vector<vector<vector<Vector3d>>> projection_init_;
		
	public:
		Map2Dall( mixmap::Camera::Ptr camera );
		void insertMap2DSence( vector<Mat> Color, SE3 T_c_w );
		void define2Dcontent( vector<Mat> depth_conv );
		
		void delete_other_region();
		PointCloud::Ptr Generateall2D();
	};
}

#endif