#ifndef IMGREADER_H
#define IMGREADER_H

#include "common_include.h"
#include <opencv2/highgui/highgui.hpp>

void KITTIReadereasy( string dataset_dir_, int datanum_, vector<string>& imgleft_, vector<string>& imgright_ );
void ladybugReader( string dataset_dir_, int datanum_, vector<vector<string>>& imgmain_, vector<vector<string>>& imgsub_, vector<string>& imgup_);

#endif