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

bool isolateShadows(VideoCapture& videoCapture, Mat& outShadowless, Mat& outShadowed, vector<Mat>& outShadowMasks)
{
	vector<Mat> sequence;
	while (videoCapture.isOpened())
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

	if (sequence.size() == 0) 
	{
		return false;
	}

	outShadowed = 255 * Mat::ones(Size(sequence[0].cols, sequence[0].rows), CV_8U);
	outShadowless = Mat::zeros(Size(sequence[0].cols, sequence[0].rows), CV_8U);

	for (const Mat& frame : sequence)
	{
		outShadowless = max(outShadowless, frame);
		outShadowed = min(outShadowed, frame);
	}

	vector<Mat> differenceSequence{};
	differenceSequence.reserve(sequence.size());
	for (const Mat& frame : sequence)
	{
		differenceSequence.emplace_back();
		absdiff(outShadowless, frame, differenceSequence.back());
	}

	namedWindow("Threshold Preview", WINDOW_KEEPRATIO);

	int i = differenceSequence.size() / 2;
	unsigned char thresh = 50;
	int keyPress = '\0';
	do {
		Mat shadowMask;
		threshold(differenceSequence[i], shadowMask, thresh, 255, CV_8U);

		imshow("Threshold Preview", shadowMask);
		keyPress = waitKey(0);

		if (keyPress == 'd')
		{
			i = (i + 1) % differenceSequence.size();
		}
		else if (keyPress == 'a')
		{
			i = (i - 1);
			if (i < 0)
			{
				i = differenceSequence.size() - 1;
			}
		}
		else if (keyPress == '=' && thresh < 255)
		{
			thresh++;
		}
		else if (keyPress == '-' && thresh > 0)
		{
			thresh--;
		}
	} while (keyPress != 'e');

	outShadowMasks.reserve(differenceSequence.size());
	for (const Mat& frame : differenceSequence)
	{
		outShadowMasks.emplace_back();
		threshold(differenceSequence[i], outShadowMasks.back(), thresh, 255, CV_8U);
	}

	return true;
}
