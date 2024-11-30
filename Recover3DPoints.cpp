#include "Recover3DPoints.h"

#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

void recover3DPoints(const Mat& shadows, Point3f lightPosition, const vector<Point3f>& shadowPlaneNormals, const vector<float>& shadowPlaneTimes, Point3f cameraPosition, const Mat& planarCoordinates, Mat& outImageCoordinates, Mat& outMask)
{
	Size imageSize = Size(shadows.cols, shadows.rows);
	outImageCoordinates = Mat::zeros(imageSize, CV_32FC3);
	outMask = Mat::zeros(imageSize, CV_8U);

	float minTime = INFINITY;
	float maxTime = 0;
	for (float time : shadowPlaneTimes)
	{
		minTime = min(time, minTime);
		maxTime = max(time, maxTime);
	}

	for (int i = 0; i < shadows.cols; i++)
	{
		for (int j = 0; j < shadows.rows; j++)
		{
			Point2i pixelPos = Point2i(i, j);
			float time = shadows.at<float>(pixelPos);
			if (time < minTime || time > maxTime)
			{
				continue;
			}

			Point3f shadowPlaneNormal;
			bool bFoundPlane = false;
			for (int i = 0; i < shadowPlaneNormals.size(); i++)
			{
				if (abs(time - shadowPlaneTimes[i]) < 0.0001)
				{
					shadowPlaneNormal = shadowPlaneNormals[i];
					bFoundPlane = true;
					break;
				}
			}

			if (!bFoundPlane)
			{
				continue;
			}

			Point3f cameraRayDir = planarCoordinates.at<Point3f>(pixelPos) - cameraPosition;
			float d = ((lightPosition - cameraPosition).dot(shadowPlaneNormal)) / (cameraRayDir.dot(shadowPlaneNormal));

			Point3f worldPos = cameraPosition + d * cameraRayDir;
			outImageCoordinates.at<Point3f>(pixelPos) = worldPos;
			outMask.at<unsigned char>(pixelPos) = 1;
		}
	}
}
