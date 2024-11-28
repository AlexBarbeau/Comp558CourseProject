#include "ShadowPlane.h"

vector<Point3f> calculateShadowPlane(Mat& shadowTime, vector<Mat>& outPlanes, Point3f lightPoint, Mat homography)
{
	Mat processImg = shadowTime.clone();
	processImg.setTo(255, shadowTime != 0);

	std::vector<cv::Point> points;
	cv::findNonZero(processImg == 255, points); // Finds all points where intensity is 255

	int line_1 = 1500;
	int line_2 = 1800;

	vector<Point2f> line1Intersects;
	vector<Point2f> line2Intersects;

	cout << processImg.size().height << endl;
	waitKey();

	for (Point p : points)
	{
		if (p.x == line_1)
		{
			line1Intersects.push_back(p);
		}

		if (p.x == line_2)
		{
			line2Intersects.push_back(p);
		}
	}

	int PlaneCount = min(line1Intersects.size(), line2Intersects.size());

	vector<Point2f> line1PointsWorld; //this is all the points in first track
	vector<Point2f> line2PointsWorld; // this is all the points in second track. NOTICE: line1PointsWorld size and line2PointsWorld size may be different.

	perspectiveTransform(line1Intersects, line1PointsWorld, homography);
	perspectiveTransform(line2Intersects, line2PointsWorld, homography);

	cout << PlaneCount << endl;

	vector<Point3f> track1Vectors; //this is all the vector from light point to the intersect of first track
	vector<Point3f> track2Vectors; //this is all the vector from light point to the intersect of second track

	vector<Point3f> normals;


	for (int i = 0; i < PlaneCount; i++)
	{
		Point3f p1(line1PointsWorld[i].x, line1PointsWorld[i].y, 0);
		Point3f p2(line2PointsWorld[i].x, line2PointsWorld[i].y, 0);
		
		Point3f v1 = p1 - lightPoint;
		Point3f v2 = p2 - lightPoint;

		track1Vectors.push_back(v1);
		track2Vectors.push_back(v2);
	
		Point3f normal(
			v1.y * v2.z - v1.z * v2.y,  // x component
			v1.z * v2.x - v1.x * v2.z,  // y component
			v1.x * v2.y - v1.y * v2.x   // z component
		);

		normals.push_back(normal);

		cout << "vector 1: " << v1 << "vector 2: " << v2 << "vector 3: " << normal << endl;
	}

	return normals;
}