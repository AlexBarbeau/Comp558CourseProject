#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class LightPointCalculation
{
private:
	vector<Point> points;
	Mat image;

public:

	int main()
	{
		points.clear();
		PickShadowPoints("Pencil1.png");
		cout << points << endl;
	}

	vector<Point> PickShadowPoints(string ImgPath)
	{
		image = imread(ImgPath);

		if (image.empty()) {
			std::cout << "Could not open or find the image" << std::endl;
			throw invalid_argument("Could not open or find the image");
		}
		int screenWidth = 800;
		int screenHeight = 600;
		double scaleWidth = static_cast<double>(screenWidth) / image.cols;
		double scaleHeight = static_cast<double>(screenHeight) / image.rows;
		double scale = std::min(scaleWidth, scaleHeight);
		int displayWidth = static_cast<int>(image.cols * scale);
		int displayHeight = static_cast<int>(image.rows * scale);
		namedWindow("Display", WINDOW_NORMAL);
		resizeWindow("Display", displayWidth, displayHeight);
		setMouseCallback("Display", onMouse, this);

		while (points.size() < 2)
		{
			cv::imshow("Display", image);
			cv::waitKey(1);
		}

		return points;
	}

	static void onMouse(int event, int x, int y, int flags, void* userdata) {

		LightPointCalculation* self = static_cast<LightPointCalculation*>(userdata);  // Cast userdata back to ImageClicker*

		if (event == cv::EVENT_LBUTTONDOWN) { // Check for left mouse button click
			Mat* image = reinterpret_cast<Mat*>(userdata); // Cast userdata to cv::Mat*

			Vec3b pixel = image->at<cv::Vec3b>(y, x);

			self->points.push_back(Point(x, y));
		}
	}
};


