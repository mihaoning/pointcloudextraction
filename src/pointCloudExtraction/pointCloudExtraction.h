#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <vector>

using namespace std;

namespace pointCloudExtraction
{
	

	bool RestorePointCloud(const float* pDepthImage, unsigned int uDepthWidth, unsigned int uDepthHeight,
		const vector<vector<float>>& cameraIntrinsic,
		const uint8_t* pColorImage, unsigned int uColorWidth, unsigned int uColorHeight, unsigned int uChannelNum,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr outPointCloud);

	bool write_point_cloud(std::string output_dir, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, int point_count);

	/*void VoxelGridFilter(const std::vector<Point>& pointCloud, const std::vector<Point>& outPointCloud);

	void ClusterRemoval(const std::vector<Point>& pointCloud, const std::vector<Point>& outPointCloud);

	glm::mat4x4 PointCloudRegister(const std::vector<Point>& pointCloudP, 
		const std::vector<Point>& pointCloudQ, 
		unsigned int uMaxIterationNum, 
		float fDistanceThreshold, 
		float fTransformThreshold);*/

}