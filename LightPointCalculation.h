#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class LightPointCalculation
{
private:
	vector<Point2d> points;
	vector<Point3d> threeDPoints;

public:

	Point3d findLightPosition(const Mat& homography, const Mat& image1, double caster1Height, const Mat& image2, double caster2Height)
	{
		Point3d intersect;

		PickShadowPoints(image1);
		vector<Point2d> worldPoints1 = vector<Point2d>();
		perspectiveTransform(points, worldPoints1, homography);

		PickShadowPoints(image2);
		vector<Point2d> worldPoints2 = vector<Point2d>();
		perspectiveTransform(points, worldPoints2, homography);

		closestPointsOnLines(
			Point3d(worldPoints1[0].x, worldPoints1[0].y, caster1Height),
			Point3d(worldPoints1[0].x - worldPoints1[1].x, worldPoints1[0].y - worldPoints1[1].y, caster1Height),
			Point3d(worldPoints2[0].x, worldPoints2[0].y, caster2Height),
			Point3d(worldPoints2[0].x - worldPoints2[1].x, worldPoints2[0].y - worldPoints2[1].y, caster2Height),
			intersect, 100);

		cout << intersect << endl;

		return intersect;
	}

	vector<Point2d> PickShadowPoints(const Mat& image)
	{
		points.clear();

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

		cout << P1 << '\n';
		cout << D1 << "\n\n";
		cout << P2 << '\n';
		cout << D2 << "\n\n";
		cout << distance << '\n';

		// Check if the distance is within the tolerance
		if (distance <= tolerance) {
			intersectionPoint = (closestPoint1 + closestPoint2) * 0.5;
			return true;
		}

		return false;
	}
};


