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
#include <string>
#include <memory>

using namespace cv;
using namespace std;

const Size calibrationPatternSize = Size(6, 9);

template <typename ... Args>
string formatString(const char* fmt, const Args& ... args)
{
	int pathLength = 1 + snprintf(nullptr, 0, fmt, args...);
	unique_ptr<char[]> buffer = unique_ptr<char[]>(new char[pathLength]);
	snprintf(buffer.get(), pathLength, fmt, args...);
	return string(buffer.get());
}

int main() {
	cout << "Please input image subfolder and image file extension separated by a space: ";
	string subfolder, imageExtension;
	cin >> subfolder >> imageExtension;

	string checkerboardPath = formatString("./images/%s/checkerboard.%s", subfolder, imageExtension);
	Mat calibrationImage = imread(checkerboardPath, 1);
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

	cout << "Press any key to continue" << endl;;
	waitKey(0);

	cout << "Please input, separated by spaces, the position of the camera in the displayed basis (x in blue, y in green, z-up): ";
	float cameraX, cameraY, cameraZ;
	cin >> cameraX >> cameraY >> cameraZ;

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
	// imshow("Calibration Image", worldPlaneCoordinates / 6);
	// cout << "Press any key to continue" << endl;;
	// waitKey(0);

	destroyWindow("Calibration Image");

	cout << "Please input the heights (in checkerboard squares) of the two shadow calibration objects separated by spaces: " << endl;
	float height1, height2;
	cin >> height1 >> height2;

	string shadow1Path = formatString("./images/%s/shadow1.%s", subfolder, imageExtension);
	string shadow2Path = formatString("./images/%s/shadow2.%s", subfolder, imageExtension);

	Mat shadowImage1 = imread(shadow1Path, 1);
	Mat shadowImage2 = imread(shadow2Path, 1);
	Point3d lightPosition = findLightPosition(homography, shadowImage1, height1, shadowImage2, height2);

	//vector<Point2d> lightXY = { Point2d(lightPosition.x, lightPosition.y) };
	//vector<Point2d> lightImagePos;
	//perspectiveTransform(lightXY, lightImagePos, invHomography);

	//const Scalar lightColour = Scalar(0, 200, 255);
	//circle(calibrationImage, lightImagePos[0], 20, lightColour, 10);
	//putText(
	//	calibrationImage, format("Height: %.2f", lightPosition.z), lightImagePos[0], 
	//	FONT_HERSHEY_SIMPLEX, 2, lightColour * 0.8, 4
	//);

	cout << "light found at: " << lightPosition << '\n';
	//imshow("Calibration Image", calibrationImage);
	//waitKey(0);
	

	string sequencePath = formatString("./images/%s/sequence/sequence%s.%s", subfolder, "%d", imageExtension);
	VideoCapture video = VideoCapture(sequencePath);
	Mat shadowless;
	Mat shadowed;
	vector<Mat> shadowMasks;
	bool bFoundShadows = isolateShadows(video, shadowless, shadowed, shadowMasks);

	imwrite("shadowless.jpg", shadowless);

	Mat shadowTime;
	findShadowTime(shadowMasks, shadowTime);

	vector<Point3f> shadowPlaneNormals;
	vector<float> planeTimes;
	calculateShadowPlane(shadowTime, lightPosition, homography, worldPlaneCoordinates, shadowPlaneNormals, planeTimes);

	Point3f cameraPosition = Point3f(cameraX, cameraY, cameraZ);

	Mat recoveredCoordinates;
	Mat coordinateMask;
	recover3DPoints(shadowTime, lightPosition, shadowPlaneNormals, planeTimes, cameraPosition, worldPlaneCoordinates, recoveredCoordinates, coordinateMask);

	namedWindow("Recovered Geometry", WINDOW_NORMAL);
	imshow("Recovered Geometry", recoveredCoordinates);

	imwrite("geometryMask.jpg", 255 * coordinateMask);
	imwrite("recoveredGeometry.jpg", 255 * recoveredCoordinates);

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

	cout << "Press any key to exit";
	waitKey(0);

	return 0;
}