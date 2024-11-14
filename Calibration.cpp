#include "Calibration.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

using namespace cv;
using namespace std;

void findHomographyForCheckerboard(const Mat& calibrationImage, const Size& patternSize, Mat& outMatrix)
{
	vector<vector<Point3f>> worldCalibrationPoints = { vector<Point3f>() };
	worldCalibrationPoints[0].reserve(patternSize.width * patternSize.height);
	for (int j = 0; j < patternSize.height; j++)
	{
		for (int i = 0; i < patternSize.width; i++)
		{
			worldCalibrationPoints[0].emplace_back(i, j, 0);
		}
	}

	vector<Point2f> imagePoints = vector<Point2f>();
	bool bFoundCorners = findChessboardCorners(calibrationImage, patternSize, imagePoints);

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

	outMatrix = findHomography(imagePoints, worldCalibrationPoints[0]);
}