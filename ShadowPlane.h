#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void calculateShadowPlane(const Mat& shadowTime, Point3f point, const Mat& homography, const Mat& worldCoorddinates, vector<Point3f>& outNormals, vector<float>& outPlaneTimes);
