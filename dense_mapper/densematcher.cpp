#include "dense_mapper/densematcher.h"

namespace dense
{
	DenseMatcher::DenseMatcher( int SADWindowSize, int cn, mixmap::Camera::Ptr camera, float mScale, int distance ):
	camera_(camera), mScale_(mScale)
	{
		mSGBM->setMinDisparity(0);                         //确定匹配搜索从哪里开始  默认值是0  
		mSGBM->setNumDisparities(128) ;    //  即最大视差值与最小视差值之差, 大小必须是16的整数倍  //在该数值确定的视差范围内进行搜索,视差窗口  
		mSGBM->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 3);                          //SAD窗口大小  
		mSGBM->setP1(4*cn*SADWindowSize*SADWindowSize);  
		mSGBM->setP2(32*cn*SADWindowSize*SADWindowSize);                                          
		//mSGBM->setTextureThreshold(1000);                  //保证有足够的纹理以克服噪声  
		mSGBM->setUniquenessRatio(10);                      //使用匹配功能模式  
		mSGBM->setSpeckleWindowSize(100);                  //检查视差连通区域变化度的窗口大小, 值为0时取消 speckle 检查  
		mSGBM->setSpeckleRange(16);                        // 视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零  
		mSGBM->setDisp12MaxDiff(1);
		
		maskfordepth.clear();
		for( int camnum = 0; camnum < camera->camerapair_ ; camnum++ )
		{
			Mat maskfordepth_;
			cv::resize( camera->maskforfeature_[camnum], maskfordepth_, cv::Size( camera->imgsize_.width/mScale_, camera->imgsize_.height/mScale_ ));
			maskfordepth.push_back( maskfordepth_ );
			
		}
		
		if( camera->camtype_ == mixmap::Camera::HORIZONTAL )
			for( int i = 0 ; i < camera_->camerapair_ ; i++ )
			{
				double mDisp_thres_ = camera->baseline_[i] * camera->OptCamInPara_[i].at<double>(0,0) /(double)distance ;
				mDisp_thres.push_back(mDisp_thres_);
			}
			//TODO VERTICAL
	}
	
	void DenseMatcher::ComputeDisparity(vector<Mat> imgLeft, vector<Mat> imgRight)
	{
		n_succ = 0;
		
		
		imgLeft_grey.clear();
		if(imgLeft[0].channels()==3)
		{
			for( int camnum = 0; camnum < imgLeft.size() ; camnum++ )
			{
				Mat imgLeft_grey_;
				cv::cvtColor(imgLeft[camnum],imgLeft_grey_,CV_BGR2GRAY);
				cv::resize( imgLeft_grey_, imgLeft_grey_, cv::Size( imgLeft[camnum].cols/mScale_, imgLeft[camnum].rows/mScale_ ));
				imgLeft_grey.push_back( imgLeft_grey_ );
			}
		} 
		imgRight_grey.clear();
		if(imgRight[0].channels()==3)
		{
			for( int camnum = 0; camnum < imgRight.size() ; camnum++ )
			{
				Mat imgRight_grey_;
				cv::cvtColor(imgRight[camnum],imgRight_grey_,CV_BGR2GRAY);
				cv::resize( imgRight_grey_, imgRight_grey_, cv::Size( imgRight[camnum].cols/mScale_, imgRight[camnum].rows/mScale_ ));
				imgRight_grey.push_back( imgRight_grey_ );
			}
		}		

		mDispImage.clear();
		mDispShow.clear();
		for( int camnum = 0; camnum < imgLeft_grey.size() ; camnum++ )
		{
			Mat disp_;
			mSGBM->compute(imgLeft_grey[camnum], imgRight_grey[camnum], disp_);
			disp_.convertTo(disp_,CV_32F,1.0/16);
			mDispImage.push_back(disp_);
			
			
				
			
		}
		
		
		
		
		
		mDepthImage.clear();
		for( int camnum = 0; camnum < imgLeft_grey.size() ; camnum++ )
		{
			Mat mDepth_ = cv::Mat::zeros( mDispImage[camnum].rows, mDispImage[camnum].cols, CV_32F );
			mDepthImage.push_back(mDepth_);
		}
		
		
		if( camera_->camtype_ == mixmap::Camera::HORIZONTAL )
		{
			for( int camnum = 0; camnum < camera_->camerapair_ ; camnum++ )
			{
#pragma omp parallel for
				for (int m = 0; m < mDispImage[camnum].rows; m++)
				{
#pragma omp parallel for
					for (int n=0; n < mDispImage[camnum].cols; n++)
					{
						if( maskfordepth[camnum].ptr<unsigned char>(m)[n] == 0 )
							continue;
						if(mDispImage[camnum].ptr<float>(m)[n]<=(float)mDisp_thres[camnum])
							continue;
						mDepthImage[camnum].ptr<float>(m)[n] = 
						(float)(camera_->baseline_[camnum]*(camera_->OptCamInPara_[camnum].at<double>(0,0)/mScale_)/mDispImage[camnum].ptr<float>(m)[n]);
						
						
						if( mDepthImage[camnum].ptr<float>(m)[n] < 0.2 )
							continue;
						n_succ++;
					}
				}
				
				Mat dispshow( mDepthImage[camnum].size(), CV_8UC3, cv::Scalar(0,0,0));
				for( int r = 0 ; r < mDepthImage[camnum].rows ; r++ )
				{
					for( int c = 0 ; c < mDepthImage[camnum].cols ; c++)
					{
						
						if( mDepthImage[camnum].ptr<float>(r)[c] < 0.2 )
							continue;
						dispshow.ptr<unsigned char>(r)[c*3] = 255-mDepthImage[camnum].ptr<float>(r)[c]*5;
						dispshow.ptr<unsigned char>(r)[c*3+2] = mDepthImage[camnum].ptr<float>(r)[c]*5;
					}
				}
				mDispShow.push_back(dispshow);
			}
			
 			/*cv::namedWindow( "disparity", cv::WINDOW_NORMAL );
 			cv::namedWindow( "depth", cv::WINDOW_NORMAL );
 			cv::imshow( "disparity", mDispImage[0] );
			cv::imshow( "depth", mDepthImage[0] );
 			cv::waitKey(0);*/
		}
		
		cv::imshow( "0", mDispShow[0] );
		cv::imshow( "1", mDispShow[1] );
		cv::imshow( "2", mDispShow[2] );
		cv::imshow( "3", mDispShow[3] );
		cv::imshow( "4", mDispShow[4] );
			//TODO VERTICAL
					
	}
       

}