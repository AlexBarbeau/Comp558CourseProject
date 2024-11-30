#include "Calibration.h"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

using namespace cv;
using namespace std;

void findHomographyForCheckerboard(const vector<Mat>& calibrationImages, const Size& patternSize, Mat& outMatrix, int scale, const Point2f& Origin)
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

	bool bFoundCorners = true;
	vector<vector<Point2f>> imagePointsSet = vector<vector<Point2f>>(calibrationImages.size());
	for (int i = 0; i < calibrationImages.size(); i++)
	{
		bFoundCorners &= findChessboardCorners(calibrationImages[i], patternSize, imagePointsSet[i], CALIB_CB_ADAPTIVE_THRESH);
	}


	Mat cameraMatrix = Mat::eye(Size(3, 3), CV_64F);
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	vector<Point3f> newObjPoints;

	vector<vector<Point3f>> worldCalibrationPointsSet = vector<vector<Point3f>>(imagePointsSet.size());
	for (int i = 0; i < imagePointsSet.size(); i++) 
	{
		worldCalibrationPointsSet[i] = (worldCalibrationPoints);
	}

	cout << worldCalibrationPointsSet.size() << '\n';
	cout << imagePointsSet.size() << '\n';
	cout << worldCalibrationPointsSet[0].size() << '\n';
	cout << imagePointsSet[0].size() << '\n';

	double rms = calibrateCameraRO(
		worldCalibrationPointsSet, imagePointsSet,
		Size(calibrationImages[0].cols, calibrationImages[0].rows), -1,
		cameraMatrix, distCoeffs,
		rvecs, tvecs,
		newObjPoints);

	Mat rotationMatrix;
	cout << rvecs[0] << '\n';
	Rodrigues(-rvecs[0], rotationMatrix);
	cout << rotationMatrix * tvecs[0] << '\n';

	outMatrix = findHomography(imagePointsSet[0], worldCalibrationPoints);
}
