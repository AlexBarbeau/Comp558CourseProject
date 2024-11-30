#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void recover3DPoints(const Mat& shadows, Point3f lightPosition, const vector<Point3f>& shadowPlaneNormals, const vector<float>& shadowPlaneTimes, Point3f cameraPosition, const Mat& planarCoordinates, Mat& outImageCoordinates, Mat& outMask);
