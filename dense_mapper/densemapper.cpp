#include "dense_mapper/densemapper.h"

namespace dense
{
	DenseMapper::DenseMapper( int SADWindowSize, int cn, mixmap::Camera::Ptr camera, float mScale, int distance, cv::Size imgsize ):
	camera_(camera), First_(true), numofref_(0)
	{
		densepointcloud_ = PointCloud::Ptr (new PointCloud);
		densematcher_ = DenseMatcher::Ptr (new DenseMatcher(SADWindowSize, cn, camera, mScale, distance));
		
		cv::Size imgsizeScale = cv::Size( imgsize.width/mScale, imgsize.height/mScale );
		double FusionDisplacement = (double)distance/4;
		densefusion_ = DenseFusion::Ptr (new DenseFusion(mScale, camera, FusionDisplacement, imgsizeScale) );
		gridsize = mixmap::Config::get<double>("voxel_grid");
		vector<Mat> Depth_Conv_before_(camera->camerapair_);
		Depth_Conv_before = Depth_Conv_before_;
	}
	
	bool DenseMapper::FrameDenseMap( mixmap::Frame::Ptr frame )
	{
		bool ref_flag = false;
		
		curr_ = frame;
		Mat imgmain_crop_;
		vector<Mat> imgmain_crop;
		vector<Mat> imagemain, imagesub;
		for( int camnum = 0; camnum < densematcher_->camera_->camerapair_ ; camnum++)
		{
			imagemain.push_back( frame->undistimage_[camnum] );
			imagesub.push_back( frame->undistimage_[camnum + densematcher_->camera_->camerapair_] );
			imgmain_crop_ = frame->undistimage_[camnum].clone();
			cv::resize( imgmain_crop_, imgmain_crop_, cv::Size( imgmain_crop_.cols/densematcher_->mScale_, imgmain_crop_.rows/densematcher_->mScale_ ));
			imgmain_crop.push_back( imgmain_crop_ );
		}

		densematcher_->ComputeDisparity( imagemain, imagesub );
		cout << "depth_succ : " << densematcher_->n_succ << endl; 

		if( densefusion_->checkNewRef(frame->T_c_w_) || First_ == true )
		{
			if( First_ == false )
				for( int cam = 0; cam < camera_->camerapair_; cam++)
					Depth_Conv_before[cam] = densefusion_->mDepthmap_conv[cam].clone();
				
			cout << "Set new reference frame for Dense Mapper!" << std::endl;
			numofref_++;
			densefusion_->InitRefInfo( imgmain_crop, densematcher_->mDepthImage, frame->T_c_w_ );
			First_ = false;
			ref_flag = true;
		}
		else
		{
			densefusion_->updateDepth( densematcher_->mDepthImage, frame->T_c_w_ );
			cout<< "min_sigma2 = " << densefusion_->min_sigma2 << endl
				<< "max_rho = " << densefusion_->max_rho << endl;
			cout << "n_conv = " << densefusion_->n_conv << endl << "n_div = " << densefusion_->n_div << endl;
			cout << "minus_depth = " << densefusion_->minus_depth << endl << "outofimage = " << densefusion_->outofimage << endl;
			ref_flag = false;
		}
		GenerateDensePointCloud( imgmain_crop, densefusion_->mDepth_final, densematcher_->mScale_ );
		//if( ref_flag == true )
			//GenerateDensePointCloud( imgmain_crop, densematcher_->mDepthImage, densematcher_->mScale_ );
		cv::imshow( "convdiv", densefusion_->mStatus);
		return ref_flag;
	}
	
	void DenseMapper::GenerateDensePointCloud( vector<Mat> imgmain_, vector<Mat> imgdepth_, float mScale )
	{
		
		for( int camnum = 0; camnum < densematcher_->camera_->camerapair_; camnum++ )
		{
			PointCloud::Ptr ptscloudnew(new PointCloud());
			#pragma omp parallel for
			for( int m = 0; m < imgmain_[camnum].rows; m++ )
			{
				#pragma omp parallel for
				for( int n = 0; n < imgmain_[camnum].cols; n++ )
				{
					if( imgdepth_[camnum].ptr<float>(m)[n] < 0.2 )
						continue;
					PointT p;
					
					Vector3d pts3d = Vector3d(
						(n-camera_->OptCamInPara_[camnum].at<double>(0,2)/mScale)*mScale*imgdepth_[camnum].ptr<float>(m)[n]/camera_->OptCamInPara_[camnum].at<double>(0,0),
						(m-camera_->OptCamInPara_[camnum].at<double>(1,2)/mScale)*mScale*imgdepth_[camnum].ptr<float>(m)[n]/camera_->OptCamInPara_[camnum].at<double>(1,1),
						imgdepth_[camnum].ptr<float>(m)[n]);
					pts3d = camera_->R_m_[camnum].inverse() * pts3d;
					
					p.x = pts3d(0);
					p.y = pts3d(1);
					p.z = pts3d(2);
					p.b = imgmain_[camnum].ptr<uchar>(m)[n*3];
					p.g = imgmain_[camnum].ptr<uchar>(m)[n*3+1];
					p.r = imgmain_[camnum].ptr<uchar>(m)[n*3+2];
					ptscloudnew->points.push_back( p );
				}
			}
			
			// 合并点云
			SE3 T2w = densefusion_->T_Ref_w.inverse() * camera_->T_0_c_[camnum];
			//SE3 T2w = camera_->T_0_c_[camnum];
			
			PointCloud::Ptr output(new PointCloud());
			pcl::transformPointCloud( *ptscloudnew, *output, T2w.matrix() );
			*densepointcloud_ += *output;
		}
		/*
		static pcl::VoxelGrid<PointT> voxel;
		voxel.setLeafSize( gridsize, gridsize, gridsize );
		voxel.setInputCloud( densepointcloud_ );
		PointCloud::Ptr tmp( new PointCloud() );
		voxel.filter( *tmp );
		densepointcloud_ = tmp;*/
		cout << "The size of densemap is " << densepointcloud_->points.size() << endl;
	}
	
}