#include "imgreader.h"

void KITTIReadereasy( string dataset_dir_, int datanum_, vector<string>& imgleft_, vector<string>& imgright_ )
{
	string extension = ".png";
	
	for( int i = 0 ; i < datanum_ ; i++ )
	{
		stringstream left_, right_;
		if( i < 10 )
		{
			left_ << dataset_dir_ << "image_2/00000" << i << extension;
			right_ << dataset_dir_ << "image_3/00000" << i << extension;
		}
		else if( i < 100 )
		{
			left_ << dataset_dir_ << "image_2/0000" << i << extension;
			right_ << dataset_dir_ << "image_3/0000" << i << extension;
		}
		else if(i < 1000)
		{
			left_ << dataset_dir_ << "image_2/000" << i << extension;
			right_ << dataset_dir_ << "image_3/000" << i << extension;
		}
		else
		{
			left_ << dataset_dir_ << "image_2/00" << i << extension;
			right_ << dataset_dir_ << "image_3/00" << i << extension;
		}
		string left,right;
		left_ >> left; right_ >> right;
				
		imgleft_.push_back( left );
		imgright_.push_back( right );
	}
}