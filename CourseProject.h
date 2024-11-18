#pragma once

#include "Calibration.h"
#include "LightPointCalculation.h"
#include "StabilizeSequence.h"
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

	const unsigned int maxFrames = 90;

	vector<Mat> sequence;
	VideoCapture videoCapture = VideoCapture("./images/robot/video.avi");
	
	while (videoCapture.isOpened() && sequence.size() < maxFrames)
	{
		if (!videoCapture.grab()) 
		{
			break;
		}

		Mat frame;
		videoCapture.retrieve(frame);
		sequence.emplace_back();
		cvtColor(frame, sequence.back(), COLOR_BGR2GRAY);
	}

	Mat minValue = 255 * Mat::ones(Size(sequence[0].cols, sequence[0].rows), CV_8U);
	Mat maxValue = Mat::zeros(Size(sequence[0].cols, sequence[0].rows), CV_8U);

	namedWindow("Video", WINDOW_NORMAL);
	namedWindow("Min", WINDOW_NORMAL);
	namedWindow("Max", WINDOW_NORMAL);

	int i = 0;
	for (const Mat& frame : sequence)
	{
		maxValue = max(maxValue, frame);
		minValue = min(minValue, frame);

		imshow("Video", frame);
		imshow("Min", minValue);
		imshow("Max", maxValue);
		// waitKey(0);
		i++;
	}

	for (const Mat& frame : sequence)
	{
		Mat differenceImage;
		absdiff(maxValue, frame, differenceImage);
		imshow("Video", differenceImage);

		Mat shadowMask;
		threshold(differenceImage, shadowMask, 70, 255, CV_8U);
		imshow("Video", shadowMask);
		waitKey(0);
	}

	return 0;
}