#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 
#include <opencv2/imgproc/imgproc.hpp>

#include "config.h"
#include "frame.h"
#include "imgreader.h"
#include "dense_mapper/densemapper.h"
#include "vo_core/orbforsys.h"
#include "map2d/map2D.h"

Mat getOptiCamMat( cv::Size origin_, cv::Size img_, Mat K, Mat dist, double xi, Mat R1 )
{
	cv::Vec2f leftup( 0, 0);
	cv::Vec2f rightup( origin_.width-1, 0);
	cv::Vec2f leftdown( 0, origin_.height-1);
	cv::Vec2f rightdown( origin_.width-1, origin_.height-1 );
	vector<cv::Vec2f> testin(4);
	vector<cv::Vec2f> testout(4);
	testin[0] = leftup;
	testin[1] = rightup;
	testin[2] = leftdown;
	testin[3] = rightdown;
	cv::omnidir::undistortPoints( testin, testout, K, dist, xi, R1 );//Mat::eye(3,3,CV_64F) R1
	vector<double> norm_x;
	vector<double> norm_y;
	for( int i = 0 ; i < 4 ; i++ )
	{
		double x = testin[i](0);
		double y = testin[i](1);
		//cout << "x = " << x << " y = " << y << endl;
		x = testout[i](0);
		y = testout[i](1);
		//cout << "x = " << x << " y = " << y << endl;
		double xs,ys,zs,dir;//the location on the unit sphere
			
		dir = ( xi + sqrt(1+ (1 - xi*xi) * (x*x + y*y)) ) / (x*x + y*y + 1);
		xs = dir*x;
		ys = dir*y;
		zs = dir - xi;

		double x1,y1;//the location in norm planar under projection origin
		x1 = xs/zs;
		y1 = ys/zs;
		norm_x.push_back(x1);
		norm_y.push_back(y1);
	}
	
	double xmin = norm_x[0], xmax= norm_x[0], ymin= norm_y[0], ymax = norm_y[0];
	for( int i = 0 ; i < 4 ; i++ )
	{
		if( norm_x[i] < xmin )
			xmin = norm_x[i];
		if( norm_x[i] > xmax )
			xmax = norm_x[i];
		if( norm_y[i] < ymin )
			ymin = norm_y[i];
		if( norm_y[i] > ymax )
			ymax = norm_y[i];
	}
	if( xmin < -10 ) xmin = -10;
	if( xmax > 10 ) xmax = 10;
	if( ymin < -10 ) ymin = -10;
	if( ymax > 10 ) ymax = 10;
	//cout << "xmin = " << xmin << " xmax = " << xmax << " ymin = " << ymin << " ymax = " << ymax << endl;
	/*
	double xmin = (norm_x[0] < norm_x[2]) ? norm_x[0]:norm_x[2];
	double xmax = (norm_x[1] > norm_x[3]) ? norm_x[1]:norm_x[3];
	double ymin = (norm_y[0] < norm_y[1]) ? norm_y[0]:norm_y[1];
	double ymax = (norm_y[2] > norm_y[3]) ? norm_y[2]:norm_y[3];
	*/
	double fx_ = (img_.width-1)/(xmax-xmin);
	double cx_ = -fx_*xmin;
	double fy_ = (img_.height-1)/(ymax-ymin);
	double cy_ = -fy_*ymin;
	Mat K1 = (cv::Mat_<double>(3,3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
	return K1;
}

int main( int argc, char** argv )
{
	if ( argc != 2 )
    {
        cout<<"usage: mixmap parameter_file"<<endl;
        return 1;
    }
    cout << "Openning : " << argv[1] << endl;
    mixmap::Config::setParameterFile( argv[1] );
	
	//图像数据位置
	string dataset_dir = mixmap::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
	int datanum = mixmap::Config::get<int> ( "datanum" );
	cout << "datanum = " << datanum << endl;
	
	vector<string> leftname_, rightname_;
	//KITTIReadereasy( dataset_dir, datanum, leftname_, rightname_);
	
	int numofcamera = mixmap::Config::get<int>( "camera_number" );
	cv::Size2i imgsize;
	imgsize.width = mixmap::Config::get<int>( "width" );
	imgsize.height = mixmap::Config::get<int>( "height" );
	int camtype = mixmap::Config::get<int>( "stereotype" );
	
	double scale = mixmap::Config::get<double>( "scale" );
	
	vector<OMNI_CAMERA_PARA> cameras;
	vector<SE3> T_0_c;
	vector<SE3> T_m_c;
	cout << "initializing the parameter of cameras..." << endl;
	/*获取多相机内参以及各相机外参*/
	for( int i = 0 ; i < numofcamera*2 ; i++ )
	{
		OMNI_CAMERA_PARA camera;
		stringstream fx_, fy_, cx_, cy_, xi_, k1_, k2_, r1_, r2_;
		fx_ << "camera" << i << ".fx";fy_ << "camera" << i << ".fy";cx_ << "camera" << i << ".cx";cy_ << "camera" << i << ".cy";xi_ << "camera" << i << ".xi";
		k1_ << "camera" << i << ".k1";k2_ << "camera" << i << ".k2";r1_ << "camera" << i << ".r1";r2_ << "camera" << i << ".r2";
		string fx, fy, cx, cy, xi, k1, k2, r1, r2;
		fx_ >> fx;fy_ >> fy;cx_ >> cx;cy_ >> cy;xi_ >> xi;
		k1_ >> k1;k2_ >> k2;r1_ >> r1;r2_ >> r2;
		
		camera.K_ = (cv::Mat_<double>(3,3)<<mixmap::Config::get<double>(fx),0,mixmap::Config::get<double>(cx),
											0, mixmap::Config::get<double>(fy), mixmap::Config::get<double>(cy),
											0,0,1);
		camera.dist_ = (cv::Mat_<double>(1,4)<< mixmap::Config::get<double>(k1),mixmap::Config::get<double>(k2),
												mixmap::Config::get<double>(r1),mixmap::Config::get<double>(r2));
		camera.xi_ = mixmap::Config::get<double>( xi );
		cameras.push_back(camera);
		cout << "camera " << i << " :" << endl;
		cout << "intris:" << endl << camera.K_ << endl;
		cout << "distort:" << endl << camera.dist_ << endl;
		cout << "xi:" << camera.xi_ << endl;
		
		stringstream q1_, q2_, q3_, q4_, t1_, t2_, t3_;
		q1_ << "camera" << i << ".q1";q2_ << "camera" << i << ".q2";q3_ << "camera" << i << ".q3";q4_ << "camera" << i << ".q4";
		t1_ << "camera" << i << ".t1";t2_ << "camera" << i << ".t2";t3_ << "camera" << i << ".t3";
		string q1, q2, q3, q4, t1, t2, t3;
		q1_ >> q1;q2_ >> q2;q3_ >> q3;q4_ >> q4;
		t1_ >> t1;t2_ >> t2;t3_ >> t3;
		
		//t_c_u用于点坐标变换，点在x+1系上的坐标转化为在x系上的坐标
		Eigen::Quaterniond q_ = Eigen::Quaterniond(mixmap::Config::get<double>( q4 ),mixmap::Config::get<double>( q1 ),mixmap::Config::get<double>( q2 ),mixmap::Config::get<double>( q3 ));
		//cout << "The quaterniond is " << endl << q_.coeffs() << endl;
		Eigen::Matrix3d R_ = q_.toRotationMatrix();
		Vector3d t_ = Vector3d(mixmap::Config::get<double>( t1 ), mixmap::Config::get<double>( t2 ), mixmap::Config::get<double>( t3 ) );
		t_ = t_ / scale;
		SE3 t_c_u( R_, t_ );
		if( i < numofcamera )
		{
			if( i == 0 )
				T_0_c.push_back(t_c_u.inverse());
			else
				T_0_c.push_back(T_0_c[i-1]*t_c_u.inverse());

			cout << "The out parameter of camera " << i << " is " << endl << T_0_c[i].matrix() << endl;
		}
		else
		{
			T_m_c.push_back(t_c_u.inverse());
			cout << "The out parameter of subcamera " << i-numofcamera << " is " << endl << T_m_c.back().matrix() << endl;
		}
		
	}
	cout << "the parameters of cameras read complete!" << endl;
	
	double distance =(double) mixmap::Config::get<int>( "2Ddistance" ) / scale;
	
	cv::Size2i optsize;
	optsize.width = 3*imgsize.height;
	optsize.height = 3*imgsize.width;
	//如果用ladybug则选1.5倍长宽
	//主副相机都获取
	vector<Mat> OptCamInPara;
	vector<Mat> R1, R2;
	vector<SO3> R_m;
	vector<Mat> map1, map2;
	vector<double> baseline;
	//这里需要注意的是有可能会在双目校正上出错
	for( int i = 0 ; i < numofcamera ; i++ )
	{
		SE3 T_c_m = T_m_c[i].inverse();
		Eigen::Matrix3d R_e = T_c_m.rotation_matrix();
		Eigen::Vector3d t_e = T_c_m.translation();
		Mat R_, t_;
		cv::eigen2cv( R_e, R_ );
		cv::eigen2cv( t_e, t_ );
		Mat R1_,R2_;
		//若是左右，则是R1在左，R2在右，若是上下，则是R1在上，R2在下
		cv::omnidir::stereoRectify( R_, t_, R1_, R2_ );
		R1.push_back(R1_);
		R2.push_back(R2_);
		Eigen::Matrix3d R_m_ei;
		cv::cv2eigen(R1_, R_m_ei);
		SO3 R_m_(R_m_ei);
		
		R_m.push_back( R_m_ );
		baseline.push_back( t_e.norm() );
		cout << "R1 of cam " << i << " : " << endl << R1_ << endl << "R2 of cam " << i << " : " << endl << R2_  << endl;
	}

	for( int i = 0 ; i < numofcamera *2 ; i++ )
	{
		Mat K_ = cameras[i].K_;
		Mat dist_ = cameras[i].dist_;
		double xi_ = cameras[i].xi_;
		
		Mat map1_, map2_;
		if( i < numofcamera )
		{
			OptCamInPara.push_back( getOptiCamMat( imgsize, optsize, K_, dist_, xi_ , R1[i]));
			cout << "Opt Camera IN Para is " << endl << OptCamInPara.back() << endl;
			cv::omnidir::initUndistortRectifyMap( K_, dist_, xi_, R1[i], OptCamInPara.back(), optsize, CV_16SC2, map1_, map2_, cv::omnidir::RECTIFY_PERSPECTIVE);
			//OptCamInPara.back()
		}
		else
			cv::omnidir::initUndistortRectifyMap( K_, dist_, xi_, R2[i-numofcamera], OptCamInPara[i-numofcamera], optsize, CV_16SC2, map1_, map2_, cv::omnidir::RECTIFY_PERSPECTIVE);
		//OptCamInPara[i-numofcamera]
		map1.push_back( map1_ );
		map2.push_back( map2_ );
	}
	
	OMNI_CAMERA_PARA cameraup;
	stringstream fx_, fy_, cx_, cy_, xi_, k1_, k2_, r1_, r2_;
	fx_ << "cameraup" << ".fx";fy_ << "cameraup" << ".fy";cx_ << "cameraup" << ".cx";cy_ << "cameraup" << ".cy";xi_ << "cameraup" << ".xi";
	k1_ << "cameraup" << ".k1";k2_ << "cameraup" << ".k2";r1_ << "cameraup" << ".r1";r2_ << "cameraup" << ".r2";
	string fx, fy, cx, cy, xi, k1, k2, r1, r2;
	fx_ >> fx;fy_ >> fy;cx_ >> cx;cy_ >> cy;xi_ >> xi;
	k1_ >> k1;k2_ >> k2;r1_ >> r1;r2_ >> r2;
		
		cameraup.K_ = (cv::Mat_<double>(3,3)<<mixmap::Config::get<double>(fx),0,mixmap::Config::get<double>(cx),
											0, mixmap::Config::get<double>(fy), mixmap::Config::get<double>(cy),
											0,0,1);
		cameraup.dist_ = (cv::Mat_<double>(1,4)<< mixmap::Config::get<double>(k1),mixmap::Config::get<double>(k2),
												mixmap::Config::get<double>(r1),mixmap::Config::get<double>(r2));
		cameraup.xi_ = mixmap::Config::get<double>( xi );
		
		cout << "camera_up :" << endl;
		cout << "intris:" << endl << cameraup.K_ << endl;
		cout << "distort:" << endl << cameraup.dist_ << endl;
		cout << "xi:" << cameraup.xi_ << endl;
		
		stringstream q1_, q2_, q3_, q4_, t1_, t2_, t3_;
		q1_ << "cameraup" << ".q1";q2_ << "cameraup" << ".q2";q3_ << "cameraup" << ".q3";q4_ << "cameraup" << ".q4";
		t1_ << "cameraup" << ".t1";t2_ << "cameraup" << ".t2";t3_ << "cameraup" << ".t3";
		string q1, q2, q3, q4, t1, t2, t3;
		q1_ >> q1;q2_ >> q2;q3_ >> q3;q4_ >> q4;
		t1_ >> t1;t2_ >> t2;t3_ >> t3;
		
		//t_c_u用于点坐标变换，点在x+1系上的坐标转化为在x系上的坐标
		Eigen::Quaterniond q_up = Eigen::Quaterniond(mixmap::Config::get<double>( q4 ),mixmap::Config::get<double>( q1 ),mixmap::Config::get<double>( q2 ),mixmap::Config::get<double>( q3 ));
		//cout << "The quaterniond is " << endl << q_.coeffs() << endl;
		Eigen::Matrix3d R_up = q_up.toRotationMatrix();
		Vector3d t_up = Vector3d(mixmap::Config::get<double>( t1 ), mixmap::Config::get<double>( t2 ), mixmap::Config::get<double>( t3 ) );
		t_up = t_up / scale;
		SE3 T_5_4( R_up, t_up );
		cout << "T_4_5 : " << endl << T_5_4.matrix() << endl;
		cout << "T_0_c size : " << T_0_c.size() << endl;
		SE3 T_0_5 = T_0_c[4] * T_5_4.inverse();
		cout << "T_0_5 : " << endl << T_0_5 << endl;
		
		
		
	Mat mask( imgsize, CV_8UC1, 1);
	for( int r = 850; r < imgsize.height ; r++ )
	{
		for( int c = 0 ; c < imgsize.width ; c++ )
		{
			mask.at<unsigned char>(r,c) = 0;
		}
	}
	for( int r = 0; r < 20 ; r++ )
	{
		for( int c = 0 ; c < imgsize.width ; c++ )
		{
			mask.at<unsigned char>(r,c) = 0;
		}
	}
	for( int r = 10; r < 850 ; r++ )
	{
		for( int c = 0 ; c < 10 ; c++ )
		{
			mask.at<unsigned char>(r,c) = 0;
		}
		for( int c = imgsize.width-10 ; c < imgsize.width ; c++ )
		{
			mask.at<unsigned char>(r,c) = 0;
		}
	}
	vector<Mat> maskforplat;
	for( int cam = 0 ; cam < numofcamera*2 ; cam++)
	{
		Mat maskcam;
		cv::remap( mask, maskcam, map1[cam], map2[cam], cv::INTER_NEAREST );
		maskforplat.push_back( maskcam );
	}
	//cout << maskforplat[0] << endl;
	
	
	mixmap::Camera::Ptr camera ( new mixmap::Camera( (enum mixmap::Camera::stereotype)camtype, numofcamera, map1, map2, T_0_c, R_m, baseline, 
													scale, distance, OptCamInPara, cameras, optsize, imgsize,
													cameraup, T_0_5, maskforplat ) );
	
	
	dense::DenseMapper::Ptr dm( new dense::DenseMapper(9, 1, camera, 1.0, distance, optsize));
 	pcl::visualization::CloudViewer viewer("viewer");
	/*
	cout << "initializing for the 2D projection..." << endl;
	
	map2d::Map2Dall::Ptr map2d_( new map2d::Map2Dall( camera ));
	
	cout << "2D projection initializing complete!" << endl;
	*/
	
	
	vector<vector<string>> mainname_, subname_;
	vector<string> upname_;
	ladybugReader( dataset_dir, datanum, mainname_, subname_, upname_);
	vector<Mat> src, udimg;
	for( int cam = 0; cam < numofcamera ; cam++ )
		{
			Mat mainsrc = cv::imread( mainname_[cam][0] );
			
			src.push_back( mainsrc );
			Mat udmain;
			cv::remap( mainsrc, udmain, map1[cam], map2[cam], cv::INTER_LINEAR );
			udimg.push_back( udmain );
		}
		for( int cam = numofcamera; cam < 2*numofcamera ; cam++ )
		{
			Mat subsrc = cv::imread( subname_[cam-numofcamera][0] );
			
			src.push_back( subsrc );
			Mat udsub;
			cv::remap( subsrc, udsub, map1[cam], map2[cam], cv::INTER_LINEAR );
			udimg.push_back( udsub );
		}
		Mat undistortresult;
		/*
	cv::hconcat( udimg[0], udimg[5], undistortresult);
	cv::imshow( "undist", undistortresult );
	cv::imwrite( "reulst.jpg", undistortresult );
	*/
	/*
			mixmap::Frame::Ptr pFrame = mixmap::Frame::createFrame();
		pFrame->camera_ = camera;
		pFrame->srcimg_ = src;
		pFrame->undistimage_ = udimg;
		dm->FrameDenseMap( pFrame );
			viewer.showCloud( dm->densepointcloud_ );
			*/
		cout << "initializing for the 2D projection..." << endl;
		map2d::Map2Dall::Ptr map2d_( new map2d::Map2Dall( camera ));
		cout << "2D projection initializing complete!" << endl;
		vector<Mat> Color;
		Mat imgup = cv::imread( upname_[0] );
		Color.push_back(udimg[0]);
		Color.push_back(udimg[1]);
		Color.push_back(udimg[2]);
		Color.push_back(udimg[3]);
		Color.push_back(udimg[4]);
		Color.push_back(imgup);
		SE3 T_c_w(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
		map2d_->insertMap2DSence( Color, T_c_w );
		
		PointCloud::Ptr showPointCloud = map2d_->Generateall2D();
		
			viewer.showCloud( showPointCloud );
			pcl::io::savePCDFile( "./2dmap.pcd", *showPointCloud );
			
		//cv::namedWindow( "src0", cv::WINDOW_NORMAL);
		//cv::namedWindow( "src1", cv::WINDOW_NORMAL);
		//cv::namedWindow( "ud0", cv::WINDOW_NORMAL);
		//cv::namedWindow( "ud1", cv::WINDOW_NORMAL);
			
			
		for( int i = 0; i < 2*numofcamera ; i++)
		{
			stringstream srcname, udname;
			srcname << "src" << i;
			udname << "ud" << i;
			string srcuse, uduse;
			srcname >> srcuse;
			udname >> uduse;
			
			cv::imshow( srcuse, src[i] );
			cv::imshow( uduse, udimg[i] );
		}
		cv::waitKey(0);
		/*
		vocore::ORBforVO::Ptr orbuse( new vocore::ORBforVO );
		
			vector<cv::KeyPoint> keypoints_main_;
			
			//cout << "udimg[0] = " << endl << udimg[0] << endl;
			orbuse->orb_->detect( udmain, keypoints_main_, mask_main );//mask_main
			
			vector<cv::KeyPoint> keypoints_sub_;
			
			orbuse->orb_->detect( udsub, keypoints_sub_, mask_sub );
			
			Mat showKeymain;
			cv::drawKeypoints( udmain, keypoints_main_, showKeymain, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
			cv::imshow( "keymain", showKeymain );
			Mat showKeysub;
			cv::drawKeypoints( udsub, keypoints_sub_, showKeysub, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
			cv::imshow( "keysub", showKeysub );
			cv::waitKey(0);
			Mat descriptors_main;
			orbuse->orb_->compute( udmain, keypoints_main_, descriptors_main );
			Mat descriptors_sub;
			orbuse->orb_->compute( udsub, keypoints_sub_, descriptors_sub );
			
			vector<cv::DMatch> matches;
			orbuse->matcher_flann_.match ( descriptors_main, descriptors_sub, matches );
			
			
			Mat img_match;
			cv::drawMatches ( udmain, keypoints_main_, udsub, keypoints_sub_, matches, img_match );
			cv::imshow( "match", img_match );
			
			vector<cv::DMatch> candidate;
			for ( cv::DMatch& m : matches )
			{
 				if( abs( keypoints_main_[m.queryIdx].pt.y - keypoints_sub_[m.trainIdx].pt.y ) < 2 )
					candidate.push_back(m);
			}
			Mat match_same_row;
			cv::drawMatches( udmain, keypoints_main_, udsub, keypoints_sub_, candidate, match_same_row);
			cv::imshow( "match_same_row", match_same_row);

		cv::waitKey(0);
		/*
		mixmap::Frame::Ptr pFrame = mixmap::Frame::createFrame();
		pFrame->camera_ = camera;
		pFrame->srcimg_ = src;
		pFrame->undistimage_ = udimg;
		
		cout << "T_c_w : " << endl << pFrame->T_c_w_.matrix() << endl;
		
		Mat img_show = udimg[0].clone();
        
        cv::imshow ( "image", img_show ); 
		
		dm->FrameDenseMap( pFrame );
			viewer.showCloud( dm->densepointcloud_ );
			cv::waitKey(0);
			
			dm->densepointcloud_->height = 1;
    dm->densepointcloud_->width = dm->densepointcloud_->points.size();
    cout<<"point cloud size = "<<dm->densepointcloud_->points.size()<<endl;
    dm->densepointcloud_->is_dense = false;
    pcl::io::savePCDFile( "./pointcloud.pcd", *(dm->densepointcloud_) );
    */
			return 0;
}
