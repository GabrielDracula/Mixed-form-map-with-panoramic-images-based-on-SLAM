#include "map2d/map2Dsence.h"

namespace map2d
{
	Map2DSence::Map2DSence(vector<vector<vector<Vector3d>>> projection_init, vector<Mat> Color, SE3 T_c_w, mixmap::Camera::Ptr camera):
	projection_cam_(projection_init), Color_(Color), T_w_c_(T_c_w.inverse()),
	camnum_(camera->camerapair_), imgsize_(camera->imgsize_), distance_(camera->max_distance_)
	{}
	
	void Map2DSence::CheckandDelete()
	{
		for( int i = 0 ; i < neighbor_location_.size() ; i++ )
		{
			cout << "neighbor_location " << i << " is " << endl << neighbor_location_[i] << endl;
			Vector3d target = T_w_c_.inverse() * neighbor_location_[i];
			for( int cam = 0 ; cam <= camnum_ ; cam++ )
			{
#pragma omp parallel for
				for( int r = 0 ; r < projection_cam_[cam].size() ; r++)
				{
#pragma omp parallel for
					for( int c = 0 ; c < projection_cam_[cam][r].size() ; c++ )
					{
						if( abs(projection_cam_[cam][r][c](0)) < 1e-4 && 
							abs(projection_cam_[cam][r][c](1)) < 1e-4 && 
							abs(projection_cam_[cam][r][c](2)) < 1e-4)
							continue;
						
						if( cam < 5 )
						{
							if( Depth_Conv_.size() > 0 )
							{
								if( Depth_Conv_[cam].at<uchar>(r,c) == 1)
									projection_cam_[cam][r][c] = Eigen::Vector3d::Zero();
							}

						}							
						
							if( (projection_cam_[cam][r][c] - target).norm() < distance_ )
								projection_cam_[cam][r][c] = Eigen::Vector3d::Zero();
						
					}
				}
			}
		}
	}
	
	PointCloud::Ptr Map2DSence::GeneratePointCloud( int num )
	{
		PointCloud::Ptr cloud ( new PointCloud );
		cout << "camnum : " << projection_cam_.size() << endl;
		for(int cam = 0 ; cam < projection_cam_.size() ; cam++)
		{
			for(int r = 0 ; r < Color_[cam].rows ; r++)
			{
				for(int c = 0 ; c < Color_[cam].cols ; c++)
				{
					
					if( abs(projection_cam_[cam][r][c](0)) < 1e-4 && 
						abs(projection_cam_[cam][r][c](1)) < 1e-4 && 
						abs(projection_cam_[cam][r][c](2)) < 1e-4)
						continue;
					
					else
					{
						PointT p;
						Vector3d p_w;
						p_w = T_w_c_ * projection_cam_[cam][r][c];
						
						p.x = p_w(0);
						p.y = p_w(1);
						p.z = p_w(2);
						
						p.b = Color_[cam].ptr<uchar>(r)[c*3];
						p.g = Color_[cam].ptr<uchar>(r)[c*3+1];
						p.r = Color_[cam].ptr<uchar>(r)[c*3+2];
						/*
						p.b = cam*50;
						p.r = 255-cam*50;
						p.g = 0;
						*/
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
}