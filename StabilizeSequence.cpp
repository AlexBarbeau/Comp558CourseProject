#include "StabilizeSequence.h"

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

void stabilizeSequence(const Mat& anchorFrame, vector<Mat>& inOutSequence)
{
	/*
	Ptr<SIFT> sift = SIFT::create(1000);

	vector<KeyPoint> anchorPoints{};
	Mat anchorDescriptors;
	sift->detectAndCompute(anchorFrame, noArray(), anchorPoints, anchorDescriptors);

	namedWindow("Preview", WINDOW_NORMAL);
	namedWindow("Anchor", WINDOW_NORMAL);

	for (Mat& frame : inOutSequence) 
	{
		BFMatcher matcher = BFMatcher();
		vector<KeyPoint> framePoints{};
		Mat frameDescriptors;
		sift->detectAndCompute(frame, noArray(), framePoints, frameDescriptors);

		vector<vector<DMatch>> matches{};
		matcher.knnMatch(frameDescriptors, anchorDescriptors, matches, 2);

		vector<Point>goodAnchorPoints{};
		goodAnchorPoints.reserve(matches.size());
		vector<Point>goodFramePoints{};
		goodFramePoints.reserve(matches.size());

		for (const vector<DMatch> pair : matches) {
			if (pair[0].distance > 0.7 * pair[1].distance) 
			{
				continue;
			}

			goodAnchorPoints.push_back(anchorPoints[pair[0].queryIdx].pt);
			goodFramePoints.push_back(framePoints[pair[0].trainIdx].pt);
		}

		Mat M = findHomography(goodFramePoints, goodAnchorPoints);
		cout << M << '\n';

		Mat stabilizedFrame;
		warpPerspective(frame, stabilizedFrame, M, Size(anchorFrame.cols, anchorFrame.rows));
		// frame = stabilizedFrame;

		imshow("Preview", stabilizedFrame);
		waitKey(0);
	}
	*/
}
