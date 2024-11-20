#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

void findHomographyForCheckerboard(const Mat& calibrationImage, const Size& patternSize, Mat& outMatrix, int scale = 1, const Point2f& Origin = Point2f(0, 0));

bool isolateShadows(VideoCapture& videoCapture, Mat& outShadowless, Mat& outShadowed, vector<Mat>& outShadowMasks);