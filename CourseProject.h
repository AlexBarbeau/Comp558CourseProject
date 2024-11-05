#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>


int main() {
	cv::Mat image = cv::imread("./palacio.png", 1);
	cv::namedWindow("OpenCV Test", cv::WINDOW_AUTOSIZE);
	cv::imshow("OpenCV Test", image);

	cv::waitKey(0);
	return 0;
}