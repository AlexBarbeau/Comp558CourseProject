#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

using namespace cv;
using namespace std;

const Size calibrationPatternSize = Size(6, 9);

vector<vector<Point3f>> worldCalibrationPoints = { vector<Point3f>() };

int main() {
	Mat image = cv::imread("./checkerboard.jpg", 1);
	namedWindow("OpenCV Test", cv::WINDOW_NORMAL);

	worldCalibrationPoints[0].reserve(calibrationPatternSize.width* calibrationPatternSize.height);
	for (int j = 0; j < calibrationPatternSize.height; j++)
	{
		for (int i = 0; i < calibrationPatternSize.width; i++)
		{
			worldCalibrationPoints[0].emplace_back(i, j, 0);
		}
	}

	vector<Point2f> chessboardCorners = vector<Point2f>();
	bool bFoundCorners = findChessboardCorners(image, calibrationPatternSize, chessboardCorners);
	drawChessboardCorners(image, calibrationPatternSize, chessboardCorners, bFoundCorners);
	for (int i = 0; i < calibrationPatternSize.width * calibrationPatternSize.height; i++)
	{
		Point3f point = worldCalibrationPoints[0][i];
		putText(image, format("(%.1f, %.1f)", point.x, point.y), 
			chessboardCorners[i], FONT_HERSHEY_COMPLEX, 
			2, Scalar(255, 255, 255), 1, LINE_AA);
	}
	
	imshow("OpenCV Test", image);

	Mat cameraMatrix = Mat::eye(Size(3, 3), CV_64F);
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	vector<Point3f> newObjPoints;

	double rms = calibrateCameraRO(
		worldCalibrationPoints, vector<vector<Point2f>>({ chessboardCorners }), 
		Size(image.cols, image.rows), -1, 
		cameraMatrix, distCoeffs, 
		rvecs, tvecs, 
		newObjPoints);
	
	waitKey(0);

	Mat imUndistorted;
	undistort(image, imUndistorted, cameraMatrix, distCoeffs);
	imshow("OpenCV Test", imUndistorted);

	waitKey(0);

	return 0;
}