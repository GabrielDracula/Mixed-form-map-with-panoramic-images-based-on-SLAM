#include "dense_mapper/densefusion.h"

namespace dense
{
	DenseFusion::DenseFusion( double scale, mixmap::Camera::Ptr camera, double displacementThres, cv::Size imgsizeScale ):
	mScale_(scale), camera_(camera), mDisplacementThres(displacementThres), imgsize_(imgsizeScale), min_sigma2(1), max_rho(0)
	{
		mSigma2Thres = 0.07;
		mInliersThres = 0.75;
		mOutliersThres = 0.5;
		
		T_Ref_w = SE3( Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

		ma_init = 1;
		mb_init = 1;

		mc_radius = 1.0f/3000;
    	mc_pixels = 1;
		
		for( int camnum = 0 ; camnum < camera->camerapair_ ; camnum++ )
		{
			double bf_ ;
			if( camera->camtype_ == mixmap::Camera::HORIZONTAL )//TODO VERTICAL
				bf_ = camera->baseline_[camnum] * camera->OptCamInPara_[camnum].at<double>(0,0);
			bf.push_back( bf_ );
		}
		
		mImage_Ref = vector<Mat>(camera->camerapair_);
		mDepth_est = vector<Mat>(camera->camerapair_);
		mDepth_obs = vector<Mat>(camera->camerapair_);
		mDepth_final = vector<Mat>(camera->camerapair_);
		mSigma2_est = vector<Mat>(camera->camerapair_);
		mSigma2_obs = vector<Mat>(camera->camerapair_);
		ma = vector<Mat>(camera->camerapair_);
		mb = vector<Mat>(camera->camerapair_);
		mDepthmap_conv = vector<Mat>(camera->camerapair_);
		mDepthmap_div = vector<Mat>(camera->camerapair_);
	}
	
	bool DenseFusion::checkNewRef( SE3 T_c_w )
	{
		SE3 T_Ref_c = T_Ref_w * T_c_w.inverse();
		Vector3d T_Ref_c_t = T_Ref_c.translation();
		if( T_Ref_c_t.norm() > mDisplacementThres )
			return true;
		else
			return false;
	}
	
	vector<Mat> DenseFusion::GetSigma2FromDepthmap( vector<Mat> DepthImg )
	{
		vector<Mat> sigma2forall;
		for( int camnum = 0; camnum < camera_->camerapair_ ; camnum++ )
		{
			Mat sigma2 = cv::Mat::zeros( imgsize_, CV_32FC1 );
#pragma omp parallel for
			for( int m = 0; m < imgsize_.height ; m++ )
			{
#pragma omp parallel for
				for( int n = 0; n < imgsize_.width ; n++ )
				{
					float z = DepthImg[camnum].ptr<float>(m)[n];
					float d_sigma = fabs( bf[camnum]/((mc_pixels/mScale_) + bf[camnum]/z ) - z );
					float radius2center =
					sqrt((m-camera_->OptCamInPara_[camnum].at<double>(1,2)/mScale_)*(m-camera_->OptCamInPara_[camnum].at<double>(1,2)/mScale_)
						+(n-camera_->OptCamInPara_[camnum].at<double>(0,2)/mScale_)*(n-camera_->OptCamInPara_[camnum].at<double>(0,2)/mScale_));
					float r_sigma = radius2center * (mc_radius/mScale_);
					
					float sigma = r_sigma + d_sigma;
					sigma2.ptr<float>(m)[n] = sigma * sigma;
				}
			}
			sigma2forall.push_back(sigma2);
		}
		
		return sigma2forall;
	}
	
	void DenseFusion::InitRefInfo( vector<Mat> Image_Ref, vector<Mat> Depth_Ref, SE3 T_c_w )
	{
		for( int camnum = 0 ; camnum < camera_->camerapair_ ; camnum++ )
		{
			mImage_Ref[camnum] = Image_Ref[camnum].clone();
			mDepth_est[camnum] = Depth_Ref[camnum].clone();
			ma[camnum] = cv::Mat( imgsize_, CV_32FC1, cv::Scalar(ma_init) );
			mb[camnum] = cv::Mat( imgsize_, CV_32FC1, cv::Scalar(mb_init) );
			mDepth_final[camnum] = cv::Mat::zeros( imgsize_, CV_32FC1 );
			mDepthmap_conv[camnum] = cv::Mat( imgsize_, CV_8UC1, cv::Scalar(0) );
			mDepthmap_div[camnum] = cv::Mat( imgsize_, CV_8UC1, cv::Scalar(0) );
		}
		mSigma2_est = GetSigma2FromDepthmap( Depth_Ref );
		T_Ref_w = T_c_w;
		
		mStatus = mImage_Ref[0].clone();
	}
		
	void DenseFusion::ProjectDepthToRef( vector<Mat> DepthImg, vector<Mat> Sigma2, SE3 T_c_w )
	{
		SE3 T_Ref_c = T_Ref_w * T_c_w.inverse();
		cout << "T_Ref_c = " << endl << T_Ref_c.matrix() << endl;
		
		for( int camnum = 0 ; camnum < camera_->camerapair_ ; camnum++ )
		{
			Mat Depth_obs_Ref = cv::Mat::zeros( imgsize_, CV_32FC1 );
			Mat Sigma_obs_Ref = cv::Mat::zeros( imgsize_, CV_32FC1 );
			mDepth_obs[camnum] = Depth_obs_Ref.clone();
			mSigma2_obs[camnum] = Sigma_obs_Ref.clone();
		}
		
		for( int camnum = 0 ; camnum < camera_->camerapair_ ; camnum++ )
		{
			
#pragma omp parallel for
			for(int m = 0 ; m < imgsize_.height ; m++ )
			{
#pragma omp parallel for
				for( int n = 0 ; n < imgsize_.width ; n++ )
				{
					Vector3d pts3d = Vector3d(
						(n-camera_->OptCamInPara_[camnum].at<double>(0,2)/mScale_)*mScale_*DepthImg[camnum].ptr<float>(m)[n]/camera_->OptCamInPara_[camnum].at<double>(0,0),
						(m-camera_->OptCamInPara_[camnum].at<double>(1,2)/mScale_)*mScale_*DepthImg[camnum].ptr<float>(m)[n]/camera_->OptCamInPara_[camnum].at<double>(1,1),
						DepthImg[camnum].ptr<float>(m)[n]);
										
					pts3d = camera_->R_m_[camnum].inverse() * pts3d;
					pts3d = T_Ref_c * camera_->T_0_c_[camnum] * pts3d;
					
					switch( camera_->camerapair_ )
					{
						case 1:
						{
							pts3d = camera_->R_m_[camnum] * pts3d;
							
							Vector2d pts2d_ref = Vector2d(
								((camera_->OptCamInPara_[camnum].at<double>(0,0)/mScale_) * (pts3d(0)/pts3d(2)) + camera_->OptCamInPara_[camnum].at<double>(0,2)/mScale_),
								((camera_->OptCamInPara_[camnum].at<double>(1,1)/mScale_) * (pts3d(1)/pts3d(2)) + camera_->OptCamInPara_[camnum].at<double>(1,2)/mScale_));
							
								
							int u_ref = (int)pts2d_ref(0);
							int v_ref = (int)pts2d_ref(1);
							if( u_ref < 0 || u_ref > imgsize_.width - 1 || v_ref < 0 || v_ref > imgsize_.height - 1 )
								continue;

							if( (mDepth_obs[0].at<float>(v_ref,u_ref) <= 0) || (Sigma2[camnum].at<float>(m,n) < mSigma2_obs[0].at<float>(v_ref,u_ref)) )
							{
								mDepth_obs[0].at<float>(v_ref,u_ref) = (float)pts3d(2);
								mSigma2_obs[0].at<float>(v_ref,u_ref) = Sigma2[camnum].at<float>(m,n);
							}
							
							break;
						}
						case 5:
						{
							int refcamnum;
							double angle = atan2(pts3d(0), pts3d(2)) + M_PI/5;
							if( angle < 0 )
								angle = angle + 2*M_PI;
							refcamnum = (int)floor( angle / (M_PI*2/5));
 							pts3d = camera_->T_0_c_[refcamnum].inverse() * pts3d;
							pts3d = camera_->R_m_[refcamnum] * pts3d;
							
							Vector2d pts2d_ref = Vector2d(
								((camera_->OptCamInPara_[refcamnum].at<double>(0,0)/mScale_) * (pts3d(0)/pts3d(2)) + camera_->OptCamInPara_[refcamnum].at<double>(0,2)/mScale_),
								((camera_->OptCamInPara_[refcamnum].at<double>(1,1)/mScale_) * (pts3d(1)/pts3d(2)) + camera_->OptCamInPara_[refcamnum].at<double>(1,2)/mScale_));
							
							int u_ref = (int)pts2d_ref(0);
							int v_ref = (int)pts2d_ref(1);
							
							if( u_ref < 0 || u_ref > imgsize_.width - 1 || v_ref < 0 || v_ref > imgsize_.height - 1 )
								continue;

							if( (mDepth_obs[refcamnum].at<float>(v_ref,u_ref) <= 0) || (Sigma2[camnum].at<float>(m,n) < mSigma2_obs[refcamnum].at<float>(v_ref,u_ref)) )
							{
								mDepth_obs[refcamnum].at<float>(v_ref,u_ref) = (float)pts3d(2);
								mSigma2_obs[refcamnum].at<float>(v_ref,u_ref) = Sigma2[camnum].at<float>(m,n);
							}					
							break;
						}
						default:
							break;
					}
				}
			}
		}
	}
	
	void DenseFusion::updateDepth( vector<Mat> DepthImg, SE3 T_c_w )
	{
		n_conv = 0;
		n_div = 0;
		minus_depth = 0;
		outofimage = 0;
		vector<Mat> sigma2 = GetSigma2FromDepthmap( DepthImg );
		ProjectDepthToRef( DepthImg, sigma2, T_c_w );
		
		for( int camnum = 0 ; camnum < camera_->camerapair_ ; camnum++ )
		{
#pragma omp parallel for
			for( int m = 0 ; m < imgsize_.height ; m++ )
			{
#pragma omp parallel for
				for( int n = 0 ; n < imgsize_.width ; n++ )
				{
					float depth = mDepth_obs[camnum].ptr<float>(m)[n];
					uchar if_converged = mDepthmap_conv[camnum].ptr<uchar>(m)[n];
					uchar if_diverged = mDepthmap_div[camnum].ptr<uchar>(m)[n];
					
					if( depth <= 0 )
						minus_depth++;
					
					if( depth <= 0 || if_converged == 1 || if_diverged == 1 )
						continue;
					
					float mu = mDepth_est[camnum].ptr<float>(m)[n];
					float sigma2 = mSigma2_est[camnum].ptr<float>(m)[n];
					float a = ma[camnum].ptr<float>(m)[n];
					float b = mb[camnum].ptr<float>(m)[n];
					
					float tau2 = mSigma2_obs[camnum].ptr<float>(m)[n];
					
					const float s_2 = (tau2 * sigma2) / (tau2 + sigma2);
					const float mm = s_2 * (mu / sigma2 + depth / tau2 );
					const float phi = sigma2 + tau2;
					float pdf_temp = 1/(sqrt(2.0f*M_PI*phi)) * (exp(-(depth-mu)*(depth-mu) / (2.0f*phi)));
					float c1 = (a / (a+b)) * pdf_temp;
					float c2 = (b / (a+b)) * (1.0f / 20.0f);   // assume: 5 meters
					const float norm_const = c1 + c2;
					c1 = c1 / norm_const;
					c2 = c2 / norm_const;
					const float f = c1 * ((a + 1.0f) / (a + b + 1.0f)) + c2 * (a / (a + b + 1.0f));
					const float e = c1 * (( (a + 1.0f)*(a + 2.0f)) / ((a + b + 1.0f) * (a + b + 2.0f ))) +
									c2 * (a*(a + 1.0f) / ((a + b + 1.0f) * (a + b + 2.0f)));
									
					if( isnan(c1*mm) )
						continue;

					float mu_prime = c1 * mm + c2 * mu;
					float sigma2_prime = c1 * (s_2 + mm*mm ) + c2 * (sigma2 + mu*mu) - mu_prime*mu_prime;
					float a_prime = ( e - f ) / ( f - e/f );
					float b_prime = a_prime * ( 1.0f - f ) / f;
					
					mDepth_est[camnum].at<float>(m,n) = mu_prime;
					mSigma2_est[camnum].at<float>(m,n) = sigma2_prime;
					ma[camnum].at<float>(m,n) = a_prime;
					mb[camnum].at<float>(m,n) = b_prime;
					
					if( min_sigma2 > sigma2_prime )
						min_sigma2 = sigma2_prime;
					
					float rho = a_prime / (a_prime + b_prime);
					
					if( max_rho < rho )
						max_rho = rho;
					
					if( sigma2_prime < mSigma2Thres && mInliersThres < rho )
					{
						mDepth_final[camnum].at<float>(m,n) = mu_prime;
						mDepthmap_conv[camnum].at<uchar>(m,n) = 1;
						
						mStatus.at<cv::Vec3b>(m,n)[0] = 255;
						mStatus.at<cv::Vec3b>(m,n)[1] = 113;
						mStatus.at<cv::Vec3b>(m,n)[2] = 83;
						n_conv ++;
					}
					if( rho < mOutliersThres )
					{
						mDepthmap_div[camnum].at<uchar>(m,n) = 1;
						
						mStatus.at<cv::Vec3b>(m,n)[0] = 89;
						mStatus.at<cv::Vec3b>(m,n)[1] = 85;
						mStatus.at<cv::Vec3b>(m,n)[2] = 255;
						n_div ++;
					}
					
					if( sigma2_prime > mSigma2Thres && mInliersThres < rho )
					{
						mStatus.at<cv::Vec3b>(m,n)[0] = 113;
						mStatus.at<cv::Vec3b>(m,n)[1] = 255;
						mStatus.at<cv::Vec3b>(m,n)[2] = 83;
					}
					if( sigma2_prime < mSigma2Thres && mInliersThres > rho && rho > mOutliersThres )
					{
						mStatus.at<cv::Vec3b>(m,n)[0] = 125;
						mStatus.at<cv::Vec3b>(m,n)[1] = 125;
						mStatus.at<cv::Vec3b>(m,n)[2] = 125;
					}
					
				}
			}
		}
	}
}