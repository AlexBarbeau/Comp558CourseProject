#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

void stabilizeSequence(const Mat& anchorFrame, vector<Mat>& inOutSequence);