#include "NotchDetect.h"
#include "opencv_headers.h"


vector<Point3f> detectNotch(const vector<Point3f>& pointCloud)
{
	vector<Point3f> notchEdges;
	float threshold = 5.0;

	for (auto i = 1; i < pointCloud.size() - 1; i++)
	{
		float dz = abs(pointCloud[i].z - pointCloud[i - 1].z);
		if (dz > threshold)
		{
			notchEdges.push_back(pointCloud[i]);
		}
	}

	return notchEdges;
}
