#pragma once

#include "Calibration.h"
#include "LightPointCalculation.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

using namespace cv;
using namespace std;

const Size calibrationPatternSize = Size(6, 9);

int main() {
	/*
	Mat calibrationImage = imread("./images/robot/checkerboard.jpg", 1);
	Mat homography;
	findHomographyForCheckerboard(calibrationImage, calibrationPatternSize, homography);

	vector<Point2f> worldBasis = { Point2f(0, 0), Point2f(1, 0), Point2f(0, 1) };
	vector<Point2f> imageBasis{};

	Mat invHomography = homography.inv();
	perspectiveTransform(worldBasis, imageBasis, invHomography);

	arrowedLine(calibrationImage, imageBasis[0], imageBasis[1], Scalar(0, 0, 255), 8);
	arrowedLine(calibrationImage, imageBasis[0], imageBasis[2], Scalar(0, 255, 0), 8);

	namedWindow("Calibration Image", WINDOW_NORMAL);
	imshow("Calibration Image", calibrationImage);
	waitKey(0);

	Mat shadowImage1 = imread("./images/robot/shadow1.jpg", 1);
	Mat shadowImage2 = imread("./images/robot/shadow2.jpg", 1);
	LightPointCalculation l = LightPointCalculation();
	Point3d lightPosition = l.findLightPosition(homography, shadowImage1, 5, shadowImage2, 5);

	vector<Point2d> lightXY = { Point2d(lightPosition.x, lightPosition.y) };
	vector<Point2d> lightImagePos;
	perspectiveTransform(lightXY, lightImagePos, invHomography);

	const Scalar lightColour = Scalar(0, 200, 255);
	circle(calibrationImage, lightImagePos[0], 20, lightColour, 10);
	putText(
		calibrationImage, format("Height: %.2f", lightPosition.z), lightImagePos[0], 
		FONT_HERSHEY_SIMPLEX, 2, lightColour * 0.8, 4
	);

	cout << lightPosition;
	imshow("Calibration Image", calibrationImage);
	waitKey(0);
	*/

	namedWindow("Video", WINDOW_NORMAL);

	const unsigned int maxFrames = 90;

	vector<Mat> sequence;
	VideoCapture videoCapture = VideoCapture("C:/Users/alex/Documents/Comp558CourseProject/images/robot/video.avi");
	
	while (videoCapture.isOpened() && sequence.size() < maxFrames)
	{
		if (!videoCapture.grab()) 
		{
			break;
		}
		sequence.emplace_back();
		videoCapture.retrieve(sequence.back());
	}

	int i = 0;
	while (i < sequence.size()) 
	{
		imshow("Video", sequence[i]);
		waitKey(0);
		i++;
	}

	return 0;
}