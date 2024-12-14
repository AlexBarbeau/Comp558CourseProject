#include "ShadowPlane.h"
#include <algorithm>
#include <vector>

using namespace cv;
using namespace std;

struct ShadowEdge {
	ShadowEdge(Point2f pixel1, Point2f pixel2, float t) : start(pixel1), end(pixel2), time(t) {}

	Point2f start;
	Point2f end;
	float time;
};

void adjustTrack(const Mat& shadows, int& trackPos) {
	int keyPress = '\0';
	cout << endl;
	cout << "Place the shadow rail (red) in a planar region" << endl;
	cout << "Use 'a' and 'd' to make small adjustments" << endl;
	cout << "Use 'z' and 'x' to make large adjustments" << endl;
	cout << "Press 'e' to confirm and continue" << endl << endl;

	do {
		Mat preview = shadows.clone();
		line(preview, Point2i(trackPos, 0), Point2i(trackPos, preview.rows), Scalar(255, 255, 255), 4);

		Mat fullPreview;
		merge(vector<Mat>({ shadows, shadows, preview }), fullPreview);

		imshow("Shadow Track", fullPreview);

		keyPress = waitKey(0);

		if (keyPress == 'a')
		{
			trackPos--;
		}
		else if (keyPress == 'd') 
		{
			trackPos++;
		}
		if (keyPress == 'z')
		{
			trackPos -= shadows.cols / 20;
		}
		else if (keyPress == 'x')
		{
			trackPos += shadows.cols / 20;;
		}

		trackPos = max(0, min(trackPos, shadows.cols - 1));
	} while (keyPress != 'e');
}

void calculateShadowPlane(const Mat& shadowTime, Point3f lightPoint, const Mat& homography, const Mat& worldCoordinates, vector<Point3f>& outNormals, vector<float>& outPlaneTimes)
{
	int track1Pos = min(50, shadowTime.cols - 1);
	int track2Pos = max(shadowTime.cols - 50, 0);

	namedWindow("Shadow Track", WINDOW_NORMAL);
	adjustTrack(shadowTime, track1Pos);
	adjustTrack(shadowTime, track2Pos);
	destroyWindow("Shadow Track");

	Mat track1 = shadowTime.col(track1Pos);
	Mat track2 = shadowTime.col(track2Pos);

	vector<Point2f> track1Intersects;
	vector<Point2f> track2Intersects;

	findNonZero(track1, track1Intersects);
	findNonZero(track2, track2Intersects);

	for (Point2f& intersect : track1Intersects) {
		intersect.x = track1Pos;
	}

	for (Point2f& intersect : track2Intersects) {
		intersect.x = track2Pos;
	}

	vector<ShadowEdge> shadowEdges;
	shadowEdges.reserve(min(track1Intersects.size(), track2Intersects.size()));

	for (Point2f& point1 : track1Intersects) 
	{
		for (Point2f& point2 : track2Intersects)
		{
			float time = shadowTime.at<float>(point1);
			if (abs(time - shadowTime.at<float>(point2)) < 0.0001)
			{
				shadowEdges.emplace_back(point1, point2, time);
				break;
			}
		}
	}

	Mat edgeImage = Mat::zeros(Size(shadowTime.cols, shadowTime.rows), shadowTime.type());

	for (ShadowEdge edge : shadowEdges)
	{
		line(edgeImage, edge.start, edge.end, Scalar(255, 255, 255), 2);

		Point3f p1 = worldCoordinates.at<Point3f>(edge.start);
		Point3f p2 = worldCoordinates.at<Point3f>(edge.end);
		
		Point3f v1 = p1 - lightPoint;
		Point3f v2 = p2 - lightPoint;

		Point3f normal(
			v1.y * v2.z - v1.z * v2.y,  // x component
			v1.z * v2.x - v1.x * v2.z,  // y component
			v1.x * v2.y - v1.y * v2.x   // z component
		);

		normal /= sqrt(normal.dot(normal));

		outNormals.push_back(normal);
		outPlaneTimes.push_back(edge.time);
	}

	Mat previewImage;
	merge(vector<Mat>{ shadowTime, shadowTime, edgeImage }, previewImage);

	namedWindow("Shadow Plane Preview", WINDOW_NORMAL);
	imshow("Shadow Plane Preview", previewImage);
	cout << "Press any key to continue" << endl;
	waitKey(0);
	destroyWindow("Shadow Plane Preview");
}