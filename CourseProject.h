#pragma once

#include "Calibration.h"
#include "LightPointCalculation.h"
#include "StabilizeSequence.h"
#include "ShadowAnalysis.h"
#include "ShadowPlane.h"
#include "Recover3DPoints.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;

const Size calibrationPatternSize = Size(6, 9);

int main() {
	Mat calibrationImage = imread("./images/constructed2/checkerboard.png", 1);
	Mat homography;
	findHomographyForCheckerboard(calibrationImage, calibrationPatternSize, homography);

	vector<Point2f> worldBasis = { Point2f(0, 0), Point2f(1, 0), Point2f(0, 1) };
	vector<Point2f> imageBasis{};

	Mat invHomography = homography.inv();
	perspectiveTransform(worldBasis, imageBasis, invHomography);

	arrowedLine(calibrationImage, imageBasis[0], imageBasis[1], Scalar(255, 0, 0), 8);
	arrowedLine(calibrationImage, imageBasis[0], imageBasis[2], Scalar(0, 255, 0), 8);

	namedWindow("Calibration Image", WINDOW_NORMAL);
	imshow("Calibration Image", calibrationImage);
	waitKey(0);

	// Precompute expected world coordinates of each pixel when projected onto the plane
	Mat imageCoordinates = Mat::zeros(Size(calibrationImage.cols, calibrationImage.rows), CV_32FC2);
	for (int j = 0; j < imageCoordinates.cols; j++)
	{
		for (int i = 0; i < imageCoordinates.rows; i++)
		{
			imageCoordinates.at<Point2f>(Point2i(j, i)) = Point2f(j, i);
		}
	}

	Mat planarCoordinates;
	perspectiveTransform(imageCoordinates, planarCoordinates, homography);

	// add Z-axis
	Mat worldPlaneCoordinates;
	merge(
		vector<Mat>({ planarCoordinates, Mat::zeros(Size(calibrationImage.cols, calibrationImage.rows), CV_32F) }),
		worldPlaneCoordinates
	);
	imshow("Calibration Image", worldPlaneCoordinates / 6);
	waitKey(0);

	destroyWindow("Calibration Image");

	
	Mat shadowImage1 = imread("./images/constructed2/shadow1.png", 1);
	Mat shadowImage2 = imread("./images/constructed2/shadow2.png", 1);
	Point3d lightPosition = findLightPosition(homography, shadowImage1, 5, shadowImage2, 5);

	//vector<Point2d> lightXY = { Point2d(lightPosition.x, lightPosition.y) };
	//vector<Point2d> lightImagePos;
	//perspectiveTransform(lightXY, lightImagePos, invHomography);

	//const Scalar lightColour = Scalar(0, 200, 255);
	//circle(calibrationImage, lightImagePos[0], 20, lightColour, 10);
	//putText(
	//	calibrationImage, format("Height: %.2f", lightPosition.z), lightImagePos[0], 
	//	FONT_HERSHEY_SIMPLEX, 2, lightColour * 0.8, 4
	//);

	cout << lightPosition;
	//imshow("Calibration Image", calibrationImage);
	//waitKey(0);
	

	VideoCapture video = VideoCapture("./images/constructed2/sequence/sequence%d.png");
	Mat shadowless;
	Mat shadowed;
	vector<Mat> shadowMasks;
	bool bFoundShadows = isolateShadows(video, shadowless, shadowed, shadowMasks);

	namedWindow("Shadowless", WINDOW_NORMAL);
	imshow("Shadowless", shadowless);

	Mat shadowTime;
	findShadowTime(shadowMasks, shadowTime);
	namedWindow("Shadow Time", WINDOW_NORMAL);
	imshow("Shadow Time", shadowTime);
	waitKey(0);

	vector<Point3f> shadowPlaneNormals;
	vector<float> planeTimes;
	calculateShadowPlane(shadowTime, lightPosition, homography, worldPlaneCoordinates, shadowPlaneNormals, planeTimes);

	Point3f cameraPosition = Point3f(5, -9, 9);

	Mat recoveredCoordinates;
	Mat coordinateMask;
	recover3DPoints(shadowTime, lightPosition, shadowPlaneNormals, planeTimes, cameraPosition, worldPlaneCoordinates, recoveredCoordinates, coordinateMask);

	namedWindow("Recovered Geometry", WINDOW_NORMAL);
	imshow("Recovered Geometry", recoveredCoordinates);

	vector<Point2i> recoveredPoints;
	findNonZero(coordinateMask, recoveredPoints);
	ofstream pointCloudFile;
	pointCloudFile.open("PointCloud.csv");
	ofstream colourFile;
	colourFile.open("PointCloudColours.csv");
	for (Point2i pointIndex : recoveredPoints) 
	{
		Point3f position = recoveredCoordinates.at<Point3f>(pointIndex);
		pointCloudFile << position.x << ',' << position.y << ',' << position.z << endl;

		unsigned char* colour = &shadowless.at<unsigned char>(pointIndex);
		colourFile << (int) colour[2] << ',' << (int) colour[1] << ',' << (int) colour[0] << endl;
	}

	pointCloudFile.close();
	colourFile.close();

	cout << "saved PointCloud.csv";

	waitKey(0);

	return 0;
}