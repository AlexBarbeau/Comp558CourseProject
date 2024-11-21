#include "Calibration.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

using namespace cv;
using namespace std;

void findHomographyForCheckerboard(const Mat& calibrationImage, const Size& patternSize, Mat& outMatrix, int scale, const Point2f& Origin)
{
	vector<Point3f> worldCalibrationPoints = vector<Point3f>() ;
	worldCalibrationPoints.reserve(patternSize.width * patternSize.height);
	for (float j = 0; j < patternSize.height; j++)
	{
		for (float i = 0; i < patternSize.width; i++)
		{
			worldCalibrationPoints.emplace_back((i - Origin.x) * scale, (j - Origin.y) * scale, 0);
		}
	}

	vector<Point2f> imagePoints = vector<Point2f>();
	bool bFoundCorners = findChessboardCorners(calibrationImage, patternSize, imagePoints, CALIB_CB_ADAPTIVE_THRESH);

	/*
	Mat cameraMatrix = Mat::eye(Size(3, 3), CV_64F);
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	vector<Point3f> newObjPoints;

	double rms = calibrateCameraRO(
		worldCalibrationPoints, vector<vector<Point2f>>({ imagePoints }),
		Size(calibrationImage.cols, calibrationImage.rows), -1,
		cameraMatrix, distCoeffs,
		rvecs, tvecs,
		newObjPoints);
	*/

	outMatrix = findHomography(imagePoints, worldCalibrationPoints);
}
