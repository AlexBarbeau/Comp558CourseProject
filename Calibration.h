#pragma once

#include <opencv2/opencv.hpp>
#include <vector>;
using namespace cv;
using namespace std;

void findHomographyForCheckerboard(const vector<Mat>& calibrationImages, const Size& patternSize, Mat& outMatrix, int scale = 1, const Point2f& Origin = Point2f(0, 0));