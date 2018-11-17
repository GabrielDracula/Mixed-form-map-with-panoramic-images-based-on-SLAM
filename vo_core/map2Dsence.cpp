/*#include "map2d/map2Dsence.h"

namespace mixmap
{
	
	void Map2DSence::CheckandDelete()
	{
		
	}
	
	PointCloud::Ptr Map2DSence::GeneratePointCloud(const SE3& T_oc_w, vector<Mat> images)
	{
		PointCloud::Ptr cloud ( new PointCloud );
		for(int cam = 0 ; cam < images.size() ; cam++)
		{
			for(int r = 0 ; r < images[cam].rows ; r++)
			{
				for(int c = 0 ; c < images[cam].cols ; c++)
				{
					if( abs(p_oc_[cam][r][c](1)) > 1e-8 || abs(p_oc_[cam][r][c](2)) > 1e-8 )
					{
						PointT p;
						Vector3d p_w;
						p_w = T_oc_w * p_oc_[cam][r][c];
						p.x = p_w(0);
						p.y = p_w(1);
						p.z = p_w(2);
						p.b = images[cam].ptr<uchar>(r)[c*3];
						p.g = images[cam].ptr<uchar>(r)[c*3+1];
						p.r = images[cam].ptr<uchar>(r)[c*3+2];
						cloud->points.push_back( p );
					}
				}
			}
		}
		
		cloud->height = 1;
		cloud->width = cloud->points.size();
		cloud->is_dense = false;

		return cloud;
	}
	
}*/