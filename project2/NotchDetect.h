#pragma once
#ifndef NOTCHDETECT_HEADERS_H
#define NOTCHDETECT_HEADERS_H

#include <iostream>
#include <vector>
#include "opencv_headers.h"

using namespace std;
using namespace cv;

vector<Point3f> detectNotch(const vector<Point3f>& pointCloud);
#endif // !