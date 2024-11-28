#pragma once
#include "ShadowPlane.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

vector<Point3f> calculateShadowPlane(Mat& shadowTime, vector<Mat>& outPlanes, Point3f point, Mat invHomography);
