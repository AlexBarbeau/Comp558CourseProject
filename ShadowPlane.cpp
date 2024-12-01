#include "ShadowPlane.h"

using namespace cv;
using namespace std;

struct ShadowEdge {
	ShadowEdge(Point2f pixel1, Point2f pixel2, float t) : start(pixel1), end(pixel2), time(t) {}

	Point2f start;
	Point2f end;
	float time;
};

void calculateShadowPlane(const Mat& shadowTime, Point3f lightPoint, const Mat& homography, const Mat& worldCoordinates, vector<Point3f>& outNormals, vector<float>& outPlaneTimes)
{
	const int track1Pos = 140;
	const int track2Pos = 2500;

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
	waitKey(0);
}