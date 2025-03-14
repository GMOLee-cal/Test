#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <Eigen/Dense>
#include "PCL_headers.h"
#include "ReadFile.h"
#include "opencv_headers.h"

void savePointCloudTotxt(const vector<Point3f>& points, const string& filename);
void ICPRegistration(
	pcl::PointCloud<pcl::PointXYZ>::Ptr notchRealCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr notchIdealCloud,
	const vector<Point3f>& realPC,
	const Eigen::Affine3f& T_initial,
	const Eigen::Matrix4f& T_ideal,
	const string& outputFileName);

int main()
{
	/// TOP Notch data File read
	// Notch 데이터 경로
	string segment_realTopPC_path = "NotchSeg_realTopPC.txt";									// cluster data using CloudCompare
	string segment_idealTopPC_path = "NotchSeg_idealTopPC.txt";									// cluster data using CloudCompare
	pcl::PointCloud<pcl::PointXYZ>::Ptr realTopCloud = ReadTxtFiletoCloud(segment_realTopPC_path);		// real top point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr idealTopCloud = ReadTxtFiletoCloud(segment_idealTopPC_path);	// ideal top point cloud

	/*//string segment_realSidePC_path = "../Data/NotchSeg_realSidePC.txt";
	//string segment_idealSidePC_path = "../Data/NotchSeg_idealSidePC.txt";
	//pcl::PointCloud<pcl::PointXYZ>::Ptr realSideCloud = ReadTxtFiletoCloud(segment_realSidePC_path);		// real top point cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr idealSideCloud = ReadTxtFiletoCloud(segment_idealSidePC_path);	// ideal top point cloud

	//string segment_realBottomPC_path = "../Data/NotchSeg_realBottomPC.txt";
	//string segment_idealBottomPC_path = "../Data/NotchSeg_idealBottomPC.txt";
	//pcl::PointCloud<pcl::PointXYZ>::Ptr realBottomCloud = ReadTxtFiletoCloud(segment_realBottomPC_path);		// real top point cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr idealBottomCloud = ReadTxtFiletoCloud(segment_idealBottomPC_path);	// ideal top point cloud
	///*/

	/// Real data File read
	// Real data 경로
	string Real_Top_path = "../Data/REAL_TOP.tiff";				// 8227350
	vector<Point3f> realTopPC;
	int realTopPC_size = ReadRealFile(Real_Top_path, &realTopPC);
	
	
	if (realTopCloud->points.empty() || idealTopCloud->points.empty())
	{
		cout << "point cloud empty!" << endl;
		return -1;
	}

	
	/*//string Real_Side_path = "../Data/REAL_SIDE.tiff";			// 8227350
	//vector<Point3f> realSidePC;
	//int realSidePC_size = ReadRealFile(Real_Side_path, &realSidePC);
	
	//string Real_Bottom_path = "../Data/REAL_BOTTOM.tiff";		// 8227350
	//vector<Point3f> realBottomPC;
	//int realBottomPC_size = ReadRealFile(Real_Bottom_path, &realBottomPC);
	///*/


	// 초기 정렬 행렬
	Eigen::Affine3f TOP_initial = Eigen::Affine3f::Identity();
	TOP_initial.translation() << 1.3, -165.0, 1.0;
	Eigen::Matrix4f T_ideal_top;
	T_ideal_top << -1, 0, 0, 7, 0, -1, 0, 537, 0, 0, 1, 0, 0, 0, 0, 1;

	//Eigen::Affine3f SIDE_initial = Eigen::Affine3f::Identity();
	//SIDE_initial.translation() << 1, -44, -0.35;

	//Eigen::Affine3f BOTTOM_initial = Eigen::Affine3f::Identity();
	//BOTTOM_initial.translation() << -1.3, -163.5, 0;


	// ICP
	ICPRegistration(realTopCloud, idealTopCloud, realTopPC, TOP_initial, T_ideal_top, "Real_To_Ideal_TOP_PC.txt");
	//ICPRegistration(realSideCloud, idealSideCloud, realSidePC, SIDE_initial, "../Data/Real_To_Ideal_SIDE_PC.txt");
	//ICPRegistration(realBottomCloud, idealBottomCloud, realBottomPC, BOTTOM_initial, "../Data/Real_To_Ideal_BOTTOM_PC.txt");




	return 0;
}

void savePointCloudTotxt(const vector<Point3f>& points, const string& filename)
{
	ofstream file(filename);
	if (!file.is_open())
	{
		return;
	}
	for (const auto& p : points) {
		file << p.x << " " << p.y << " " << p.z << endl;
	}
	file.close();
}


void ICPRegistration(
	pcl::PointCloud<pcl::PointXYZ>::Ptr notchRealCloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr notchIdealCloud,
	const vector<Point3f>& realPC,
	const Eigen::Affine3f& T_initial,
	const Eigen::Matrix4f& T_ideal,
	const string& outputFileName)
{
	// 초기 정렬 적용
	pcl::transformPointCloud(*notchRealCloud, *notchRealCloud, T_initial);


	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(notchRealCloud);
	icp.setInputTarget(notchIdealCloud);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	
	if (!icp.hasConverged())
	{
		cout << "failed" << endl;
		return;
	}

	cout << "ICP Transformation Matrix : " << endl << icp.getFinalTransformation() << endl;


	Eigen::Matrix4f RT = icp.getFinalTransformation();
	Eigen::Matrix4f T_final = RT * T_initial.matrix();
	Eigen::Matrix4f T_merged = T_ideal * T_final; // transformation matrix to merged 3D(real data -> ideal data -> origin frame)


	cout << "merged 3D Transformation Matrix(Top) : " << endl << T_merged << endl;


	vector<Point3f> transformedPC;
	transformedPC.reserve(realPC.size());

	for (const auto& pt : realPC)
	{
		Eigen::Vector4f pt_h(pt.x, pt.y, pt.z, 1.0f);
		Eigen::Vector4f pt_transformed = T_final * pt_h;
		transformedPC.push_back(Point3f(pt_transformed[0], pt_transformed[1], pt_transformed[2]));
	}

	// saved read -> ideal Point Cloud
	savePointCloudTotxt(transformedPC, outputFileName);
	cout << "Real to Ideal Transformed Point Cloud saved : " << outputFileName << endl;
}