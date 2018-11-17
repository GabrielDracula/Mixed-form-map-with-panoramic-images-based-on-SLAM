#include "frame.h"

namespace mixmap
{
	Frame::Frame()
	: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
	{}
	
	Frame::Frame( unsigned long id, double timestamp, SE3 T_c_w ,Camera::Ptr camera )
	:id_(id), time_stamp_(timestamp), T_c_w_(T_c_w), camera_(camera)
	{}
	
	Frame::~Frame()
	{}

	Frame::Ptr Frame::createFrame()
	{
		static long factory_id = 0;
		return Frame::Ptr( new Frame(factory_id++) );
	}
	
	Vector3d Frame::getCamCenter( int camnum ) const
	{
		return T_c_w_.inverse() * camera_->T_0_c_[camnum].translation();
	}
	
	void Frame::setPose ( const SE3& T_c_w )
	{	
		T_c_w_ = T_c_w;
	}
	
	bool Frame::isInFrame( const Vector3d& pt_world )
	{
		for( int i = 0 ; i < camera_->camerapair_ ; i++)
		{
			if( camera_->world2camera( pt_world, T_c_w_, i ).norm() > 2*camera_->max_distance_ )
				return false;
		}
		return true;
	}
}