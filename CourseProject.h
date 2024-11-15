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

	Mat image = imread("./checkerboard.jpg", 1);
	Mat homography;
	findHomographyForCheckerboard(image, Size(6, 9), homography);

	vector<Point2f> worldBasis = { Point2f(0, 0), Point2f(1, 0), Point2f(0, 1) };
	vector<Point2f> imageBasis{};

	Mat invHomography = homography.inv();
	perspectiveTransform(worldBasis, imageBasis, invHomography);

	arrowedLine(image, imageBasis[0], imageBasis[1], Scalar(0, 0, 255), 8);
	arrowedLine(image, imageBasis[0], imageBasis[2], Scalar(0, 255, 0), 8);

	imshow("OpenCV Test", image);
	waitKey(0);

	LightPointCalculation l = LightPointCalculation();
	Point3d lightPosition = l.findLightPosition(homography, image, 1, image, 1);

	vector<Point2d> lightXY = { Point2d(lightPosition.x, lightPosition.y) };
	vector<Point2d> lightImagePos;
	perspectiveTransform(lightXY, lightImagePos, invHomography);

	const Scalar lightColour = Scalar(0, 200, 255);
	circle(image, lightImagePos[0], 20, lightColour, 10);
	putText(
		image, format("Height: %.2f", lightPosition.z), lightImagePos[0], 
		FONT_HERSHEY_SIMPLEX, 2, lightColour * 0.8, 4
	);

	imshow("OpenCV Test", image);

	waitKey(0);

	return 0;
}