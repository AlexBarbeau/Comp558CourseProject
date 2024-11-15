#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;

void findHomographyForCheckerboard(const Mat& calibrationImage, const Size& patternSize, Mat& outMatrix, int scale = 1, const Point2f& Origin = Point2f(0, 0));

