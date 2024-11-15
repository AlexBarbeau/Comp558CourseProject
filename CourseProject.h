#pragma once

#include "Calibration.h"
#include "LightPointCalculation.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

using namespace cv;
using namespace std;

const Size calibrationPatternSize = Size(6, 9);
vector<vector<Point3f>> worldCalibrationPoints = { vector<Point3f>() };

int main() {
	namedWindow("OpenCV Test", WINDOW_NORMAL); 

	constexpr int SCALE = 100;

	Mat image = imread("./checkerboard.jpg", 1);
	Mat homography;
	findHomographyForCheckerboard(image, Size(6, 9), homography, SCALE, Point2f(-2, -2));

	vector<Point2f> worldBasis = { Point2f(0, 0), Point2f(SCALE, 0), Point2f(0, SCALE) };
	vector<Point2f> imageBasis{};

	Mat invHomography = homography.inv();
	perspectiveTransform(worldBasis, imageBasis, invHomography);

	arrowedLine(image, imageBasis[0], imageBasis[1], Scalar(0, 0, 255), 8);
	arrowedLine(image, imageBasis[0], imageBasis[2], Scalar(0, 255, 0), 8);

	imshow("OpenCV Test", image);
	waitKey(0);

	Mat warpedImage{};
	warpPerspective(image, warpedImage, homography, Size(image.cols, image.rows));

	imshow("OpenCV Test", warpedImage);
	waitKey(0);

	LightPointCalculation l = LightPointCalculation();
	cout << l.findLightPosition(homography, image, SCALE, image, SCALE);

	return 0;
}