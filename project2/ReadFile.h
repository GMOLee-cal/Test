#pragma once
#ifndef READ_HEADERS_H
#define READ_HEADERS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "opencv_headers.h"

using namespace std;
using namespace cv;


pcl::PointCloud<pcl::PointXYZ>::Ptr ReadTxtFiletoCloud(const std::string& filename);
vector<Point3f> ReadTxtFile(const string& path);
int ReadMasterFile(string path, vector<Point3f>* ptr);
int ReadRealFile(string path, vector<Point3f>* ptr);

#endif // !