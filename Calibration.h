#pragma once

#include <opencv2/opencv.hpp>

using namespace cv;

void findHomographyForCheckerboard(const Mat& calibrationImage, const Size& patternSize, Mat& outMatrix);

