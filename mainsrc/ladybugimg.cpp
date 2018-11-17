#include "imgreader.h"

void ladybugReader( string dataset_dir_, int datanum_, vector<vector<string>>& imgmain_, vector<vector<string>>& imgsub_, vector<string>& imgup_ )
{
	string extension = ".jpg";
	imgmain_.clear();
	imgsub_.clear();
	for( int cam = 0 ; cam < 5 ; cam++ )
	{
		vector<string> cammain_, camsub_;
		
		for( int i = 0 ; i < datanum_ ; i++)
		{
			stringstream main_, sub_;
			main_ << dataset_dir_ << "cammain" << cam << "/" << i << extension;
			sub_ << dataset_dir_ << "camsub" << cam << "/" << i << extension;
			string main_name, sub_name;
			main_ >> main_name; sub_ >> sub_name;
			cammain_.push_back(main_name); camsub_.push_back(sub_name);
		}
		imgmain_.push_back(cammain_); imgsub_.push_back(camsub_);
	}
	for( int i = 0 ; i < datanum_ ; i++ )
	{
		stringstream up_;
		up_ << dataset_dir_ << "camup/" << i << extension;
		string up_name;
		up_ >> up_name;
		imgup_.push_back(up_name);
	}
	
}