#include <iostream>
#include <fstream>
#include <sstream>
#include<vector>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "FaceRecon.h"
using namespace std;

inline float ConvertDepth(float fDepth)
{
	return (fDepth * 100.0f);
}

bool pointCloudExtraction::RestorePointCloud(const float* pDepthImage, unsigned int uDepthWidth, unsigned int uDepthHeight,
	const vector<vector<float>>& cameraIntrinsic,
	const uint8_t* pColorImage, unsigned int uColorWidth, unsigned int uColorHeight, unsigned int uChannelNum,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outPointCloud)
{
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr outPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    
	for (unsigned int uRowIndex = 0; uRowIndex < uDepthHeight; ++uRowIndex)
	{
		for (unsigned int uColIndex = 0; uColIndex < uDepthWidth; ++uColIndex)
		{
			float fDepth = ConvertDepth(pDepthImage[uRowIndex * uDepthWidth + uColIndex]);
            if (fDepth > 100)continue;
			const float x = (uColIndex - cameraIntrinsic[0][2]) * fDepth / cameraIntrinsic[0][0];
			const float y = (uRowIndex - cameraIntrinsic[1][2]) * fDepth / cameraIntrinsic[1][1];
            if (isnan(x) || isnan(y) || isnan(fDepth))continue;
			/*glm::vec2 uv;
			uv.x = (float)uColIndex / (uDepthWidth);
			uv.y = (float)uRowIndex / (uDepthHeight);
			glm::ivec2 pixelPos;*/
            int posx = (float)uColIndex;// / (uDepthWidth - 1) * (uColorWidth));
            int posy = (float)uRowIndex;// / (uDepthHeight - 1) * (uColorHeight));
			const unsigned int uColorPixelIndex = (posy) * uColorWidth + posx;
			/*const uint8_t r = (pColorImage[(uColorPixelIndex ) * uChannelNum] + pColorImage[(uColorPixelIndex + 1) * uChannelNum] + pColorImage[(uColorPixelIndex + uColorWidth) * uChannelNum] + pColorImage[(uColorPixelIndex + uColorWidth + 1) * uChannelNum])/4;
			const uint8_t g = (pColorImage[(uColorPixelIndex)*uChannelNum + 1] + pColorImage[(uColorPixelIndex + 1) * uChannelNum + 1] + pColorImage[(uColorPixelIndex + uColorWidth) * uChannelNum + 1] + pColorImage[(uColorPixelIndex + uColorWidth + 1) * uChannelNum + 1]) / 4;
			const uint8_t b = (pColorImage[(uColorPixelIndex)*uChannelNum + 2] + pColorImage[(uColorPixelIndex + 1) * uChannelNum + 2] + pColorImage[(uColorPixelIndex + uColorWidth) * uChannelNum + 2] + pColorImage[(uColorPixelIndex + uColorWidth + 1) * uChannelNum + 2]) / 4;
            */
            const uint8_t r = pColorImage[(uColorPixelIndex)*uChannelNum];
            const uint8_t g = pColorImage[uColorPixelIndex * uChannelNum + 1];
            const uint8_t b = pColorImage[uColorPixelIndex * uChannelNum + 2];

			/*Point p;
			p.pos = glm::vec3(x, y, -fDepth);
			p.color = glm::vec3(r, g, b);*/
            outPointCloud->push_back(pcl::PointXYZRGB(x, y, -fDepth,r,g,b));
			//outPointCloud.push_back(p);
		}
	}

	return true;
}

bool pointCloudExtraction::write_point_cloud(std::string output_dir, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, int point_count)
{
    int width = 480;
    int height = 640;


    std::cout << "begin to save" << std::endl;
    // save to the ply file
    std::ofstream ofs(output_dir); // text mode first

    //if (ofs) std::cout << " new file created" << std::endl;
    if (ofs.is_open())
    {
        std::cout << "file.is_open()" << std::endl;
    }
    else
        return false;

    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex"
        << " " << point_count << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property float nx" << std::endl;
    ofs << "property float ny" << std::endl;
    ofs << "property float nz" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int i = 0; i < point_count; i++)
    {

        ss << (float)pointCloud->points[i].x << " " << (float)pointCloud->points[i].y << " "
            << (float)pointCloud->points[i].z << " " << (float)pointCloud->points[i].r << " " << (float)pointCloud->points[i].g << " " << (float)pointCloud->points[i].b << std::endl;
    }

    std::ofstream ofs_text(output_dir, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
    ofs_text.close();
    return true;
}



//void pointCloudExtraction::VoxelGridFilter(const std::vector<Point>& pointCloud, const std::vector<Point>& outPointCloud)
//{
//
//}
//
//void pointCloudExtraction::ClusterRemoval(const std::vector<Point>& pointCloud, const std::vector<Point>& outPointCloud)
//{
//
//}
//
//glm::mat4x4 pointCloudExtraction::PointCloudRegister(const std::vector<Point>& pointCloudP, const std::vector<Point>& pointCloudQ, unsigned int uMaxIterationNum, float fDistanceThreshold, float fTransformThreshold)
//{
//	return glm::mat4x4();
//}
