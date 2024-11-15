#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class LightPointCalculation
{
private:
	vector<Point> points;
	vector<Point3d> threeDPoints;
	Mat image;
	Mat K, R, t;

public:

	void calibrate()
	{

		K = (Mat_<double>(3, 3) << 1, 0, 1,
								   0, 1, 1,
								   0, 0, 1.0);

		R = (Mat_<double>(3, 3) << 
			1,0,0,
			0,0.707, -0.707,
			0,0.707 ,0.707);

		t = (Mat_<double>(3, 1) << 3, 26, -19);

		Point3d intersect;

		PickShadowPoints("Unity.png");
		Calculate3DPositions();

		//cout << points << endl;
		PickShadowPoints("Unity.png");
		Calculate3DPositions();

		closestPointsOnLines(threeDPoints[0] + Point3d(0,0,17), 
			threeDPoints[0] + Point3d(0,0,17) - threeDPoints[1],
			threeDPoints[3] + Point3d(0, 0, 17), 
			threeDPoints[3] + Point3d(0, 0, 17) - threeDPoints[2], 
			intersect, 100);

		cout << intersect << endl;
	}

	void Calculate3DPositions()
	{
		for (int i = 0; i < points.size(); i++)
		{
			Mat p_img = (Mat_<double>(3, 1) << points[i].x, points[i].y, 1.0);
			Mat d_camera = K.inv() * p_img;

			Mat d_world = R.t() * d_camera;
			double x = -(t.at<double>(2, 0) / d_world.at<double>(2, 0));

			Mat p_world = t + x * d_world;
			threeDPoints.push_back(Point3d(p_world.at<double>(0,0), p_world.at<double>(1, 0), p_world.at<double>(2, 0)));
		}
	}

	vector<Point> PickShadowPoints(string ImgPath)
	{
		points.clear();
		image = imread(ImgPath);

		if (image.empty()) {
			cout << "Could not open or find the image" << endl;
			throw invalid_argument("Could not open or find the image");
		}
		int screenWidth = 800;
		int screenHeight = 600;
		double scaleWidth = static_cast<double>(screenWidth) / image.cols;
		double scaleHeight = static_cast<double>(screenHeight) / image.rows;
		double scale = min(scaleWidth, scaleHeight);
		int displayWidth = static_cast<int>(image.cols * scale);
		int displayHeight = static_cast<int>(image.rows * scale);
		namedWindow("Display", WINDOW_NORMAL);
		resizeWindow("Display", displayWidth, displayHeight);

		setMouseCallback("Display", &LightPointCalculation::onMouse, this);

		while (points.size() < 2)
		{
			imshow("Display", image);
			waitKey(1);
		}

		setMouseCallback("Display", nullptr);

		Mat preview = image;
		arrowedLine(preview, points[0], points[1], Scalar(255, 0, 255), 8);
		imshow("Display", preview);
		waitKey(0);

		return points;
	}

	static void onMouse(int event, int x, int y, int flags, void* userdata) {

		LightPointCalculation* self = static_cast<LightPointCalculation*>(userdata);

		if (event == EVENT_LBUTTONDOWN) {
			Mat* image = reinterpret_cast<Mat*>(userdata);

			Vec3b pixel = image->at<Vec3b>(y, x);

			self->points.push_back(Point(x, y));
		}
	}

	bool closestPointsOnLines(const Point3d& P1, const Point3d& D1,
		const Point3d& P2, const Point3d& D2,
		Point3d& intersectionPoint, double tolerance = 1e-2) {
		Point3d P12 = P2 - P1;
		double d1d1 = D1.dot(D1);
		double d2d2 = D2.dot(D2);
		double d1d2 = D1.dot(D2);
		double p12d1 = P12.dot(D1);
		double p12d2 = P12.dot(D2);

		double denominator = d1d1 * d2d2 - d1d2 * d1d2;
		if (abs(denominator) < 1e-9) {
			// Lines are parallel, no unique intersection
			return false;
		}

		double t = (p12d2 * d1d2 - p12d1 * d2d2) / denominator;
		double s = (p12d2 + t * d1d2) / d2d2;

		Point3d closestPoint1 = P1 + t * D1;
		Point3d closestPoint2 = P2 + s * D2;

		double distance = norm(closestPoint1 - closestPoint2);

		// Check if the distance is within the tolerance
		if (distance <= tolerance) {
			intersectionPoint = (closestPoint1 + closestPoint2) * 0.5;
			return true;
		}

		return false;
	}
};


