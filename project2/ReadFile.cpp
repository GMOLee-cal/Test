#include "ReadFile.h"
#include "opencv_headers.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr ReadTxtFiletoCloud(const std::string& filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	ifstream file(filename);

	if (!file.is_open())
	{
		cout << "file open failed" << endl;
		return cloud;
	}

	string line;
	while (getline(file, line))
	{
		if (line.empty())
			continue;

		stringstream ss(line);
		float x, y, z;
		if (!(ss >> x >> y >> z))
			continue;

		pcl::PointXYZ point;
		point.x = x;
		point.y = y;
		point.z = z;
		cloud->points.push_back(point);
	}

	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->height = 1;
	cloud->is_dense = true;

	file.close();

	return cloud;
}

vector<Point3f> ReadTxtFile(const string& path)
{
	vector<Point3f> points;
	ifstream file(path);
	if (!file.is_open())
	{
		cout << "file opened fail" << endl;
		return points;
	}

	string line;
	while (getline(file, line))
	{
		stringstream ss(line);
		float x, y, z;
		ss >> x >> y >> z;
		points.push_back(Point3f(x, y, z));
	}

	file.close();

	return points;
}

int ReadMasterFile(string path, vector<Point3f>* ptr)
{
	auto idealDepthMap = imread(path, IMREAD_UNCHANGED);

	
	if (idealDepthMap.empty()) {
		cout << path << "     data is empty" << endl;
		return -1;
	}

	auto dataFullSize = idealDepthMap.total();
	auto imgPtr = idealDepthMap.ptr<Vec3f>(0);
	ptr->resize(dataFullSize);	// 읽은 데이터에 대한 vector resize

	for (auto i = 0; i < dataFullSize; ++i)
	{
		const auto& zyx = imgPtr[i];
		(*ptr)[i].x = zyx[2];
		(*ptr)[i].y = zyx[1];
		(*ptr)[i].z = zyx[0];
	}

	if (dataFullSize == 0 || imgPtr == nullptr)
	{
		return -1;
	}


	return static_cast<int>(dataFullSize);
}

int ReadRealFile(string path, vector<Point3f>* ptr)
{
	const float NULLVALUE = -999, SCALEX = 0.006, SCALEY = 0.1;

	auto realDepthMap = imread(path, IMREAD_UNCHANGED);
	auto width = realDepthMap.cols, height = realDepthMap.rows;
	auto dataFullSize = width * height;
	auto imgPtr = realDepthMap.ptr<float>(0);
	ptr->resize(dataFullSize);

	auto curIdx = 0;
	for (int j = 0; j < height; ++j)
	{
		auto step = width * j;
		for (int i = 0; i < width; ++i)
		{
			if (NULLVALUE != imgPtr[step + i])
			{
				(*ptr)[curIdx].x = SCALEX * i;
				(*ptr)[curIdx].y = SCALEY * j;
				(*ptr)[curIdx].z = -imgPtr[step + i];
				curIdx++;
			}
		}
	}

	ptr->resize(curIdx);

	return static_cast<int>(dataFullSize);
}