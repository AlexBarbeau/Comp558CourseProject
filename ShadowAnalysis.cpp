#include "ShadowAnalysis.h"

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
		threshold(frame, outShadowMasks.back(), thresh, 255, CV_8U);
	}

	return true;
}

bool findShadowTime(const vector<Mat>& shadowMaskSequence, Mat& outShadowTime)
{
	if (shadowMaskSequence.size() == 0) 
	{
		return false;
	}

	Size imageSize = Size(shadowMaskSequence[0].cols, shadowMaskSequence[0].rows);
	outShadowTime = Mat::zeros(imageSize, CV_32F);
	for (unsigned char t = 0; t < shadowMaskSequence.size(); t++) 
	{
		for (int u = 0; u < imageSize.width; u++)
		{
			bool bFoundEdge = false;
			for (int v = 0; v < imageSize.height; v++)
			{
				Point pixel = Point2i(u, v);
				if (shadowMaskSequence[t].at<unsigned char>(pixel) > 0)
				{
					bFoundEdge = true;
					outShadowTime.at<float>(pixel) = t;
					break;
				}
			}
		}
	}

	return true;
}
