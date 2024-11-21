#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

bool isolateShadows(VideoCapture& videoCapture, Mat& outShadowless, Mat& outShadowed, vector<Mat>& outShadowMasks);

bool findShadowTime(const vector<Mat>& shadowMaskSequence, Mat& outShadowTime);