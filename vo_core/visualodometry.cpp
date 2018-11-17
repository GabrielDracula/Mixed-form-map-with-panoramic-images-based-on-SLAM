#include "vo_core/visualodometry.h"
#include "config.h"
#include "vo_core/g2o_types.h"
#include "common_include.h"
#include "camera.h"
Mat test;
namespace vocore
{
	VisualOdometry::VisualOdometry():
	state_ ( INITIALIZING ), ref_key_ ( nullptr ), curr_ ( nullptr ), localmap_ ( new LocalMap ), num_lost_ ( 0 ), num_inliers_ ( 0 ), orb_vo_( new ORBforVO )
	{
		max_num_lost_       = mixmap::Config::get<float> ( "max_num_lost" );
		min_inliers_        = mixmap::Config::get<int> ( "min_inliers" );
		key_frame_min_rot   = mixmap::Config::get<double> ( "keyframe_rotation" );
		key_frame_min_trans = mixmap::Config::get<double> ( "keyframe_translation" );
		map_point_erase_ratio_ = mixmap::Config::get<double> ( "map_point_erase_ratio" );
		T_c_w_estimated_ = SE3( Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
		
		//posefile.open( "./pose.txt" );
	}
	bool VisualOdometry::addFrame ( mixmap::Frame::Ptr frame )
	{
		switch ( state_ )
		{
			case INITIALIZING:
			{
				state_ = OK;
				curr_ = ref_key_ = frame;
				frame->T_c_w_ = T_c_w_estimated_;
				// extract features from first frame and add them into map
				extractKeyPoints();
				computeDescriptors();
				featureMatching_Stereo_add();
				addKeyFrame();      // the first frame is a key-frame
				T_c_w_last_ = T_c_w_estimated_;
				
				/*for( int i = 0 ; i < 12 ; i++ )
				{
					posefile << (T_c_w_estimated_.inverse()).matrix()(i/4,i%4);
					if( i == 11 )
						posefile << '\n';
					else
						posefile << " ";
				}*/
				
				break;
			}
			case OK:
			{
				//fortest
				curr_ = frame;
				curr_->T_c_w_ = ref_key_->T_c_w_;
				extractKeyPoints();
				computeDescriptors();
				featureMatching();
				poseEstimationPnP();
				//cv::imshow( "test", test );
				if ( checkEstimatedPose() == true ) // a good estimation
				{
					curr_->T_c_w_ = T_c_w_estimated_;
					featureMatching_Stereo_add();
					optimizeMap();
					num_lost_ = 0;
					T_c_w_last_ = T_c_w_estimated_;
					
					if ( checkKeyFrame() == true ) // is a key-frame
					{
						addKeyFrame();
					}
					
					/*for( int i = 0 ; i < 12 ; i++ )
					{
						posefile << (T_c_w_estimated_.inverse()).matrix()(i/4,i%4);
						if( i == 11 )
							posefile << '\n';
						else
							posefile << " ";
					}*/
				}	
				else // bad estimation due to various reasons
				{
					num_lost_++;
					if ( num_lost_ > max_num_lost_ )
					{
						state_ = LOST;
					}
					return false;
				}
				break;
			}
			case LOST:
			{
				cout<<"vo has lost."<<endl;
				break;
			}
		}
		
		return true;
	}
	
	void VisualOdometry::extractKeyPoints()
	{
		boost::timer timer;
		keypoints_curr_.clear();
		for( int i = 0 ; i < camera_->camerapair_*2 ; i++ )
		{
			vector<cv::KeyPoint> keypoints_;
			orb_vo_->orb_->detect ( curr_->undistimage_[i], keypoints_, camera_->maskforfeature_[i] );
			keypoints_curr_.push_back( keypoints_ );
		}
				
		cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
	}
	
	void VisualOdometry::computeDescriptors()
	{
		boost::timer timer;
		descriptor_curr_.clear();
		for( int i = 0 ; i < camera_->camerapair_*2 ; i++ )
		{
			Mat descriptors_;
			orb_vo_->orb_->compute ( curr_->undistimage_[i], keypoints_curr_[i], descriptors_ );
			descriptor_curr_.push_back( descriptors_ );
		}
		cout<<"compute descriptors cost time: "<<timer.elapsed() <<endl;
	}
	
	void VisualOdometry::featureMatching_Stereo_add()
	{
		//cv::namedWindow( "matches_stereo", cv::WINDOW_NORMAL );
		boost::timer timer;
		int addnum=0, addcamnum=0;
		
		cout << "Add mappoint" << endl;
		for( int i = 0 ; i < camera_->camerapair_ ; i++ )
		{
  			if( i == 2 )
  				continue;
			vector<cv::DMatch> matches;
			orb_vo_->matcher_flann_.match ( descriptor_curr_[i], descriptor_curr_[i+camera_->camerapair_], matches );
			vector<cv::DMatch> candidate;
			for ( cv::DMatch& m : matches )
			{
 				if( camera_->camtype_ == mixmap::Camera::HORIZONTAL )
				{
					if( abs( keypoints_curr_[i][m.queryIdx].pt.y - keypoints_curr_[i+camera_->camerapair_][m.trainIdx].pt.y ) < 3 )
					candidate.push_back(m);
				}
			}
			
			float min_dis = std::min_element (
                        candidate.begin(), candidate.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
			{
				return m1.distance < m2.distance;
			} )->distance;
			
			
			vector<cv::DMatch> show_wrong,show_right;
			
			for ( cv::DMatch& m : candidate )
			{
				//goodmatches
				if ( m.distance < max<float> ( min_dis*orb_vo_->match_ratio_, 30.0 ) )
				{
					if( camera_->camtype_ == mixmap::Camera::HORIZONTAL )
					{
						int disparity = keypoints_curr_[i][m.queryIdx].pt.x - keypoints_curr_[i+camera_->camerapair_][m.trainIdx].pt.x;
						double depth = (double) camera_->baseline_[i] * camera_->OptCamInPara_[i].at<double>(0,0) / disparity;
						
						//cout << keypoints_curr_[i][m.queryIdx].pt.x << "\t" << keypoints_curr_[i+camera_->camerapair_][m.trainIdx].pt.x << "\t";
						//cout << "depth of " << addnum << " : " << depth << endl;
						
						if( depth > camera_->max_distance_ || depth <= 0 )
						{
							show_wrong.push_back(m);
							continue;
						}
						//cout << keypoints_curr_[i][m.queryIdx].pt.x << "\t" << keypoints_curr_[i+camera_->camerapair_][m.trainIdx].pt.x << "\t";
						//cout << "depth of " << addnum << " : " << depth << " ";
						
						Vector2d p_p;
						Mat p_p_cv(keypoints_curr_[i][m.queryIdx].pt);
						cv::cv2eigen(p_p_cv, p_p);
						
						Vector3d p_w = camera_->pixel2world( p_p, T_c_w_estimated_, i, depth);
						/*if(i==2)
						cout << "p p : " << p_p.transpose() << "\t" << "p_c : " << camera_->pixel2camera( p_p, i, depth ).transpose() << "\t" 
							<<"p_w : " << p_w.transpose() << endl;*/
						//cout << "point " << addnum << " : " << p_w.transpose() << " distance : " << sqrt(p_w(0)*p_w(0)+p_w(2)*p_w(2)) << endl ;
						
						Vector3d n = p_w - ref_key_->getCamCenter(i);
						n.normalize();
						MapPoint3d::Ptr map_point = MapPoint3d::createLocalMapPoint(p_w, n, descriptor_curr_[i].row(m.queryIdx).clone(), curr_.get());
						show_right.push_back(m);
						localmap_->insertMapPoint( map_point );
						addnum++;
						addcamnum++;
					}
				}
			}
			cout << "cam " << i << " : " << addcamnum << "\t";
			addcamnum = 0;
			/*
			cv::drawMatches( curr_->undistimage_[i], keypoints_curr_[i], 
							 curr_->undistimage_[i+camera_->camerapair_], keypoints_curr_[i+camera_->camerapair_], matches, test);
			cv::imshow("matches_stereo", test);
			cv::waitKey(0);
	
			cv::drawMatches( curr_->undistimage_[i], keypoints_curr_[i], 
							 curr_->undistimage_[i+camera_->camerapair_], keypoints_curr_[i+camera_->camerapair_], show_wrong, test);
			cv::imshow("matches_stereo", test);
			cv::waitKey(0);
			
			cv::drawMatches( curr_->undistimage_[i], keypoints_curr_[i], 
							 curr_->undistimage_[i+camera_->camerapair_], keypoints_curr_[i+camera_->camerapair_], show_right, test ,
							cv::Scalar(0,0,255));
			cv::imshow("matches_stereo", test);
			cv::waitKey(0);*/
		}
		cout << endl << "Add mappoint with stereo : " << addnum << endl;
		cout<<"Matching Stereo features and add mappoint cost time: "<<timer.elapsed() <<endl;
	}
	
	void VisualOdometry::addKeyFrame()
	{
		localmap_->insertKeyFrame ( curr_ );
		ref_key_ = curr_;
		cout << "This is a Key Frame." << endl;
	}
	
	void VisualOdometry::featureMatching()
	{
		boost::timer timer;
		
		Mat desp_map;
		vector<cv::Vec3f> pts_pos_;
		vector<MapPoint3d::Ptr> candidate;
		for ( auto& allpoints: localmap_->map_points_ )
		{
			MapPoint3d::Ptr& p = allpoints.second;
			// check if p in curr frame image 
			if ( curr_->isInFrame(p->pos_) )
			{
				// add to candidate 
				p->visible_times_++;
				candidate.push_back( p );
				cv::Vec3f pos_ = cv::Vec3f( p->pos_(0), p->pos_(1), p->pos_(2) );
				pts_pos_.push_back( pos_ );
				desp_map.push_back( p->descriptor_ );
			}
		}
		
		match_3dpts_.clear();
		match_2dkp_index_.clear();
		for( int i = 0 ; i < camera_->camerapair_ ; i++ )
		{
			vector<MapPoint3d::Ptr> deleteback;
			SE3 T_c_w_n = camera_->T_0_c_[i].inverse() * curr_->T_c_w_;
			Vector3d tvec_ei = T_c_w_n.translation();
			Eigen::Matrix3d rvec_ei = T_c_w_n.rotation_matrix();
			Mat rvec, tvec;
			cv::eigen2cv( rvec_ei, rvec );
			cv::eigen2cv( tvec_ei, tvec );
			cv::Rodrigues( rvec, rvec );
			vector<cv::Vec2f> imgpts;
			cv::omnidir::projectPoints( pts_pos_, imgpts, rvec, tvec,
										camera_->CamInPara_[i].K_, camera_->CamInPara_[i].xi_, camera_->CamInPara_[i].dist_ );
			Mat desp_map_cam;
			for( int pt = 0 ; pt < imgpts.size() ; pt++ )
			{
				if( imgpts[pt](0) > 0 && imgpts[pt](0) < camera_->imgsize_ori_.width &&
					imgpts[pt](1) > 0 && imgpts[pt](1) < camera_->imgsize_ori_.height )
				{
					deleteback.push_back( candidate[pt] );
					desp_map_cam.push_back( desp_map.row(pt) );
				}
			}
			cout << "After delete back point : " << deleteback.size() << endl;	
			vector<MapPoint3d::Ptr> match_3dpts_singlecam;
			vector<int> match_2dkp_index_singlecam;
			vector<cv::DMatch> matches;
			float min_dis;
			if( deleteback.size() > 4 )
			{
				orb_vo_->matcher_flann_.match ( desp_map_cam, descriptor_curr_[i], matches );
				// select the best matches
				min_dis = std::min_element (
									matches.begin(), matches.end(),
									[] ( const cv::DMatch& m1, const cv::DMatch& m2 )
				{
					return m1.distance < m2.distance;
				} )->distance;
				cout << "min_dis : " << min_dis << endl;
			}

			for ( cv::DMatch& m : matches )
			{
				if ( m.distance < max<float> ( min_dis*orb_vo_->match_ratio_, 30.0 ) )
				{
					match_3dpts_singlecam.push_back( deleteback[m.queryIdx] );
					match_2dkp_index_singlecam.push_back( m.trainIdx );
				}
			}
			cout<<"good matches for cam " << i << " : " << match_3dpts_singlecam.size() <<endl;
			match_3dpts_.push_back( match_3dpts_singlecam );
			match_2dkp_index_.push_back( match_2dkp_index_singlecam );
		}
		cout<<"match cost time: "<<timer.elapsed() <<endl;
	}
	
	void VisualOdometry::poseEstimationPnP()
	{
		boost::timer timer;
		int num_inliers_max = 0;
		Mat rvec, tvec, max_inliers;
		vector<cv::Point3f> pts3d_max_match;
		vector<cv::Point2f> pts2d_max_match;
		cout<<"pnp inliers cam :" << endl;
		for( int i = 0 ; i < camera_->camerapair_ ; i++ )
		{
 			if( i == 2 )
 				continue;
			Mat inliers;
			vector<cv::Point3f> pts3d;
			vector<cv::Point2f> pts2d;
			
			for ( int index:match_2dkp_index_[i] )
			{
				pts2d.push_back ( keypoints_curr_[i][index].pt );
			}
			for( MapPoint3d::Ptr pt:match_3dpts_[i] )
			{
				pts3d.push_back ( pt->getPositionCV() );
			}
			Mat K = camera_->OptCamInPara_[i];
			/*
			Eigen::Matrix3d rvec_ei;
			Vector3d tvec_ei;
			rvec_ei = curr_->T_c_w_.rotation_matrix();
			cv::eigen2cv( rvec_ei, rvec );
			cv::Rodrigues( rvec, rvec );
			tvec_ei = curr_->T_c_w_.translation();
			cv::eigen2cv( tvec_ei, tvec );
			*/
			if( pts3d.size() >= 4)
			{
				cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
				if( num_inliers_max < inliers.rows)
				{
					num_inliers_max = inliers.rows;
					max_inliers = inliers;
					max_match_cam_ = i;
					T_c_w_estimated_ = SE3 (SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ),
											Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
													);	
					
					pts3d_max_match = pts3d;
					pts2d_max_match = pts2d;
				}
				// set the inlier map points 
				for( int j = 0; j < inliers.rows; j++ )
				{
					int index = inliers.at<int> ( j,0 );
					match_3dpts_[i][index]->matched_times_++;
					//cout << "3dpts " << index << " : " << match_3dpts_[i][index]->pos_ << endl;
				}
			}
			
			cout<< i << " : "<< inliers.rows << "\t";	
		}
		cout << endl;
		num_inliers_ = num_inliers_max;
		cout << "The camera with max match is " << max_match_cam_ << endl;
		//cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl << T_c_w_estimated_ << endl;
		
		typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 2> > Block;	
		std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>() );
		std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm (solver);
		
		g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
		pose->setId ( 0 );
		pose->setEstimate ( g2o::SE3Quat (
			T_c_w_estimated_.rotation_matrix(), T_c_w_estimated_.translation()
		));
		optimizer.addVertex ( pose );

		// edges
		for ( int i=0; i<max_inliers.rows; i++ )
		{
			int index = max_inliers.at<int> ( i,0 );
			// 3D -> 2D projection
			EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
			edge->setId ( i );
			edge->setVertex ( 0, pose );
			edge->camera_ =  curr_->camera_.get();
			edge->max_match_cam_ = max_match_cam_;
			edge->point_ = Vector3d ( pts3d_max_match[index].x, pts3d_max_match[index].y, pts3d_max_match[index].z );
			edge->setMeasurement ( Vector2d ( pts2d_max_match[index].x, pts2d_max_match[index].y ) );
			edge->setInformation ( Eigen::Matrix2d::Identity() );
			optimizer.addEdge ( edge );

		}

		optimizer.initializeOptimization();
		optimizer.optimize ( 10 );

		//这里估计的结果是经过双目校正的某相机的位姿
		SE3 R_m_max;
				
		T_c_w_estimated_ = SE3 (pose->estimate().rotation(), pose->estimate().translation()	);
		//cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl << T_c_w_estimated_ << endl;
		
		R_m_max = SE3( camera_->R_m_[max_match_cam_].inverse(), Vector3d(0,0,0));
		
		//T_c_w_estimated_ = camera_->T_0_c_[max_match_cam_] * (camera_->R_m_[max_match_cam_] * T_c_w_estimated_);
		T_c_w_estimated_ = camera_->T_0_c_[max_match_cam_] * R_m_max * T_c_w_estimated_;
		
		cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix() << endl;
		cout<<"SolvePnP cost time: "<<timer.elapsed() <<endl;
	}
	
	void VisualOdometry::optimizeMap()
	{
		cout<<"map points before optimize : "<<localmap_->map_points_.size()<<endl;
		for ( auto iter = localmap_->map_points_.begin(); iter != localmap_->map_points_.end(); )
		{
			if( !curr_->isInFrame( iter->second->pos_ ) )
			{
				iter = localmap_->map_points_.erase(iter);
				continue;
			}
			float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
			if ( match_ratio < map_point_erase_ratio_ )
			{
				iter = localmap_->map_points_.erase(iter);
				continue;
			}
			iter++;
		}
		
		if ( localmap_->map_points_.size() > 10000 )  
		{
			
			map_point_erase_ratio_ += 0.05;
		}
		else 
			map_point_erase_ratio_ = mixmap::Config::get<double> ( "map_point_erase_ratio" );
		cout<<"map points: "<<localmap_->map_points_.size()<<endl;
	}
	
	bool VisualOdometry::checkEstimatedPose()
	{
		// check if the estimated pose is good
		if ( num_inliers_ < min_inliers_ )
		{
			cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
			return false;
		}
		// if the motion is too large, it is probably wrong
		SE3 T_r_c = ref_key_->T_c_w_ * T_c_w_estimated_.inverse();
		
		Sophus::Vector6d d = T_r_c.log();
		//向量的模长
		if ( d.norm() > 20.0 )
		{
			cout<<"reject because motion is too large: "<<d.norm() <<endl;
			return false;
		}
		
		SE3 T_l_c = T_c_w_last_ * T_c_w_estimated_.inverse();
		d = T_l_c.log();
		if( d.norm() > 1.0 )
		{
			cout <<"reject because motion between frames is too large: " << d.norm() << endl;
			return false;
		}
		return true;
	}
	
	bool VisualOdometry::checkKeyFrame()
	{
		SE3 T_r_c = ref_key_->T_c_w_ * T_c_w_estimated_.inverse();
		Sophus::Vector6d d = T_r_c.log();
		Vector3d trans = d.head<3>();
		Vector3d rot = d.tail<3>();
		if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
			return true;
		return false;
	}
}
