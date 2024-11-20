#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Point3d findLightPosition(const Mat& homography, const Mat& image1, double caster1Height, const Mat& image2, double caster2Height, double tolerance = 4);

void PickShadowPoints(const Mat& image, vector<Point2d>& selectedPoints);

bool closestPointsOnLines(const Point3d& P1, const Point3d& D1, const Point3d& P2, const Point3d& D2, Point3d& intersectionPoint, double tolerance = 1e-2);


