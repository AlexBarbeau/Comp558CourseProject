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
	namedWindow("Difference Preview", WINDOW_KEEPRATIO);

	cout << endl;
	cout << "Adjust the image threshold" << endl;
	cout << "Use 'a' and 'd' to seek through the sequence" << endl;
	cout << "Use '-' and '=' to adjust the threshold" << endl;
	cout << "Use 'o' and 'p' to adjust the blur radius" << endl;
	cout << "Press e to confirm and continue" << endl << endl;

	int i = differenceSequence.size() / 2;
	int sigma = 0;
	unsigned char thresh = 70;
	int keyPress = '\0';
	do {
		Mat blurredDifference;
		if (sigma > 0) {
			GaussianBlur(differenceSequence[i], blurredDifference, Size(33, 33), sigma, sigma);
		}
		else
		{
			blurredDifference = differenceSequence[i];
		}

		Mat shadowMask;
		threshold(blurredDifference, shadowMask, thresh, 255, CV_8U);

		cout << "Threshold:" << (int)thresh << " Sigma:" << sigma << '\n';
		imshow("Difference Preview", blurredDifference);
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
		else if (keyPress == 'p')
		{
			sigma++;
		}
		else if (keyPress == 'o')
		{
			if (sigma >= 1)
			{
				sigma--;
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

	destroyWindow("Threshold Preview");
	destroyWindow("Difference Preview");

	outShadowMasks.reserve(differenceSequence.size());
	for (const Mat& frame : differenceSequence)
	{
		Mat blurredFrame;
		if (sigma > 0) {
			GaussianBlur(frame, blurredFrame, Size(33, 33), sigma, sigma);
		}
		else
		{
			blurredFrame = frame;
		}

		outShadowMasks.emplace_back();
		threshold(blurredFrame, outShadowMasks.back(), thresh, 255, CV_8U);
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
