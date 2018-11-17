#include "map2d/map2D.h"
#include "camera.h"
#include "frame.h"
#include "map2d/map2Dsence.h"

namespace map2d
{
	Map2Dall::Map2Dall( mixmap::Camera::Ptr camera ):
	camera_(camera)
	{
		vector<Eigen::Matrix3d> Rotation;
		Vector3d t_c_cen = Eigen::Vector3d::Zero();
		for( int i = 0; i < camera->camerapair_ ; i++ )
		{
			t_c_cen = t_c_cen + camera->T_0_c_[i].translation();
			Rotation.push_back( camera->T_0_c_[i].rotation_matrix());
		}//将SE3转换为旋转矩阵和平移向量
		t_c_cen = t_c_cen + camera->T_0_5_.translation();
		Rotation.push_back( camera->T_0_5_.rotation_matrix());
		
		t_c_cen(0) = t_c_cen(0)/Rotation.size();
		t_c_cen(1) = t_c_cen(1)/Rotation.size();
		t_c_cen(2) = t_c_cen(2)/Rotation.size();
		projection_init_.clear();
		
		for( int i = 0; i < camera->camerapair_ ; i++ )
		{
			vector<vector<Vector3d>> pro_camera_;
			Mat K;
			
			K = camera->OptCamInPara_[i];
			
#pragma omp parallel for
			for( int r = 0 ; r < camera->imgsize_.height ; r++ )
			{
				vector<Vector3d> row_;
#pragma omp parallel for
				for( int c = 0 ; c < camera->imgsize_.width ; c++ )
				{
					Vector3d col_;
					if( camera->maskforfeature_[i].at<unsigned char>(r,c) == 0 )
						col_ = Eigen::Vector3d::Zero();
					else
					{
						double x,y;//the location in norm planar under xi
						x = ( c - K.at<double>(0,2) ) / K.at<double>(0,0);
						y = ( r - K.at<double>(1,2) ) / K.at<double>(1,1);
						Vector3d before_rotate,after_rotate;
						before_rotate << x, y, 1;
						after_rotate = camera->R_m_[i].inverse() * before_rotate;
						x = after_rotate(0)/after_rotate(2);
						y = after_rotate(1)/after_rotate(2);
						double A,B,C;
						Vector3d undercam;//the 3D location under single camera coordinate
						//we neglect the translation because it is small
						A = Rotation[i](0,0) * x + Rotation[i](0,1) * y + Rotation[i](0,2);
						B = Rotation[i](1,0) * x + Rotation[i](1,1) * y + Rotation[i](1,2);
						C = Rotation[i](2,0) * x + Rotation[i](2,1) * y + Rotation[i](2,2);
						//求得相机系下该像素的三维位置
						undercam(2) = (double)camera->max_distance_ / sqrt( A*A + B*B + C*C );
						undercam(1) = undercam(2) * y;
						undercam(0) = undercam(2) * x;
						//求得未对齐整体系下像素的三维位置
						col_ = Rotation[i] * undercam;
						//以下部分需要根据相机个数进行修改
						
						double angle = atan2(col_(0), col_(2));//the angle in camera0 coordinate
						
						if( (angle < ((2*( (i+2)%5 )-5)*CV_PI/5)) || (angle > ((2*( (i+2)%5 )-3)*CV_PI/5)) )
							col_ = Eigen::Vector3d::Zero();
						else if( sqrt(col_(0)*col_(0) + col_(2)*col_(2)) <= camera->max_distance_*2/3 )
							col_ = Eigen::Vector3d::Zero();
						else
							col_ = col_ + t_c_cen;
						
					}
					
					row_.push_back(col_);
				}//then we get the 2D point in the camera0 coordinate
				pro_camera_.push_back(row_);
			}
			projection_init_.push_back(pro_camera_);
		}
		
		vector<vector<Vector3d>> pro_camera_;
		#pragma omp parallel for
		for( int r = 0 ; r < camera->imgsize_ori_.height ; r++ )
		{
			vector<Vector3d> row_;
			#pragma omp parallel for
			for( int c = 0 ; c < camera->imgsize_ori_.width ; c++ )
			{
				Vector3d col_;
				cv::Vec2f u_v_ = cv::Vec2f(c,r);
				vector<cv::Vec2f> u_v;
				vector<cv::Vec2f> x_y;
				u_v.push_back(u_v_);
				
				cv::omnidir::undistortPoints( u_v, x_y, camera->cameraup_.K_, camera->cameraup_.dist_, camera->cameraup_.xi_, Mat::eye(3,3,CV_64F));
				
				double x,y;//the location in norm planar under xi
				x = x_y[0](0);
				y = x_y[0](1);
				
				double xs,ys,zs,dir;//the location on the unit sphere
				
				dir = ( camera->cameraup_.xi_ + sqrt(1+ (1 - camera->cameraup_.xi_*camera->cameraup_.xi_) * (x*x + y*y)) ) / (x*x + y*y + 1);
				xs = dir*x;
				ys = dir*y;
				zs = dir - camera->cameraup_.xi_;
				
				double x1,y1;//the location in norm planar under projection origin
				x1 = xs/zs;
				y1 = ys/zs;
				
				double A,B,C;
				Vector3d undercam;//the 3D location under single camera coordinate
				//we neglect the translation because it is small
				A = Rotation[5](0,0) * x1 + Rotation[5](0,1) * y1 + Rotation[5](0,2);
				B = Rotation[5](1,0) * x1 + Rotation[5](1,1) * y1 + Rotation[5](1,2);
				C = Rotation[5](2,0) * x1 + Rotation[5](2,1) * y1 + Rotation[5](2,2);
				
				//求得相机系下该像素的三维位置
				undercam(2) = (double)camera->max_distance_ / sqrt( A*A + B*B + C*C );
				undercam(1) = undercam(2) * y1;
				undercam(0) = undercam(2) * x1;
				//求得未对齐整体系下像素的三维位置
				col_ = 	Rotation[5] * undercam;
				if( sqrt(col_(0)*col_(0) + col_(2)*col_(2)) > camera->max_distance_*2/3 )
					col_ = Eigen::Vector3d::Zero();
				else
					col_ = col_ + t_c_cen;
				row_.push_back(col_);
			}
			pro_camera_.push_back(row_);
		}
		projection_init_.push_back(pro_camera_);
	}
	
	void Map2Dall::insertMap2DSence( vector<Mat> Color, SE3 T_c_w )
	{
		
		Map2DSence::Ptr sence_ = Map2DSence::Ptr (new Map2DSence(projection_init_, Color, T_c_w, camera_));
		
		map_sences_.push_back(sence_);
		
	}
	
	void Map2Dall::delete_other_region()
	{
		for( int i = 0 ; i < map_sences_.size() ; i++ )
		{
			cout << "Now check map_sence " << i << endl;
			for( int j = i+1 ; j < map_sences_.size() ; j++ )
			{
				if( (map_sences_[i]->T_w_c_.translation() - map_sences_[j]->T_w_c_.translation()).norm() < 10*camera_->max_distance_)
				{
					map_sences_[i]->neighbor_location_.push_back(map_sences_[j]->T_w_c_.translation());
					map_sences_[j]->neighbor_location_.push_back(map_sences_[i]->T_w_c_.translation());
				}
			}
			map_sences_[i]->CheckandDelete();
		}
	}
	
	PointCloud::Ptr Map2Dall::Generateall2D()
	{
		PointCloud::Ptr map2dfinal( new PointCloud );
		for( int i = 0 ; i < map_sences_.size() ; i++ )
		{
			PointCloud::Ptr map2d_sence = map_sences_[i]->GeneratePointCloud(i);
			*map2dfinal += *map2d_sence;
		}
		return map2dfinal;
	}
}