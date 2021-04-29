#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION    
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize.h"
#include <vector>
#include <algorithm>
//#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp>
//#include <glm/gtc/type_ptr.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <learnopengl/filesystem.h>
#include <learnopengl/shader_m.h>
#include <learnopengl/camera.h>

#include <iostream>
#include <fstream>

#include "pointCloudExtraction.h"
#include "traverseFiles.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1920;

// camera
//Camera camera(glm::vec3(0.0f, 0.0f, 0.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

bool LoadCameraIntrinsicFromFile(const char* szFileName, vector<vector<float>>& cameraIntrinsic)
{
	std::ifstream ifs(FileSystem::getPath(szFileName));
	if (!ifs)
	{
		std::cout << "Failed to open camera intrinsic file." << std::endl;
		return false;
	}

	ifs >> cameraIntrinsic[0][0];
	ifs >> cameraIntrinsic[0][1];
	ifs >> cameraIntrinsic[0][2];
	ifs >> cameraIntrinsic[1][0];
	ifs >> cameraIntrinsic[1][1];
	ifs >> cameraIntrinsic[1][2];
	ifs >> cameraIntrinsic[2][0];
	ifs >> cameraIntrinsic[2][1];
	ifs >> cameraIntrinsic[2][2];

	return true;
}

bool GeneratePointCloudFromImage(const char* szDepthImageName, const char* szColorImageName, const vector<vector<float>>& cameraIntrinsic, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outPointCloud)
{
	// load depth image
	//int nDepthWidth, nDepthHeight, nDepthChannels;

	//stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
	//uint16_t *depthData = stbi_load_16(FileSystem::getPath(szDepthImageName).c_str(), &nDepthWidth, &nDepthHeight, &nDepthChannels, 0);
	//if (!depthData)
	//{
	//	std::cout << "Failed to load depth image." << std::endl;
	//	return false;
	//}

	bool bFlipVertical = true;

	const unsigned int nDepthWidth = 480, nDepthHeight = 640;
	float* depthData = new float[nDepthWidth * nDepthHeight * sizeof(float)];
	std::ifstream depthifs(FileSystem::getPath(szDepthImageName), std::ios::binary | std::ios::in);
	depthifs.read((char*)depthData, nDepthWidth * nDepthHeight * sizeof(float));
    std::cout << sizeof(depthData)/sizeof(float)<<std::endl;
	if (bFlipVertical)
	{
		float* temp = new float[nDepthWidth * nDepthHeight * sizeof(float)];
		for (unsigned int i = 0; i < nDepthHeight; i++)
		{
			for (unsigned int j = 0; j < nDepthWidth; j++)
			{
				temp[i * nDepthWidth + j] = depthData[(nDepthHeight - 1 - i) * nDepthWidth + j];
			}
		}
		memcpy(depthData, temp, nDepthWidth * nDepthHeight * sizeof(float));
        std::cout << sizeof(*depthData) / sizeof(float)<<std::endl;
		delete[] temp;
	}
	

	// load color image
	int nColorWidth, nColorHeight, nColorChannels;
	stbi_set_flip_vertically_on_load(bFlipVertical); // tell stb_image.h to flip loaded texture's on the y-axis.
	uint8_t *colorData = stbi_load(FileSystem::getPath(szColorImageName).c_str(), &nColorWidth, &nColorHeight, &nColorChannels, 0);
	int ow = nColorWidth / 2;
	int oh = nColorHeight / 2;
	auto* odata = (unsigned char*)malloc(ow * oh * nColorChannels);
	string path = szColorImageName;
	string::size_type iPos = path.find_last_of('\\') + 1;
	string filename = path.substr(iPos, path.length() - iPos);
	string outputPath = "";
	outputPath += filename;
	stbir_resize(colorData, nColorWidth, nColorHeight, 0, odata, ow, oh, 0, STBIR_TYPE_UINT8, nColorChannels, STBIR_ALPHA_CHANNEL_NONE, 0,
		STBIR_EDGE_CLAMP, STBIR_EDGE_CLAMP,
		STBIR_FILTER_BOX, STBIR_FILTER_BOX,
		STBIR_COLORSPACE_SRGB, nullptr
	);
	cout << outputPath;
	stbi_flip_vertically_on_write(true);
	stbi_write_jpg(outputPath.c_str(), ow, oh, nColorChannels, odata, 100);

	if (!colorData)
	{
		std::cout << "Failed to load color image." << std::endl;
		//stbi_image_free(depthData);
		return false;
	}
	//std::cout << nDepthHeight << " " << nDepthWidth << " " << ow << " " << oh << endl;
	bool bSucc = pointCloudExtraction::RestorePointCloud(depthData, nDepthWidth, nDepthHeight, cameraIntrinsic, odata, ow, oh, nColorChannels, outPointCloud);
	
	delete[] depthData;
	//stbi_image_free(depthData);
	stbi_image_free(colorData);

	return bSucc;
}



int main()
{
	const std::string strCameraIntrinsicFile = "resources/camera_intrinsic.txt";
	const std::string strImagePath = "\\video_case_2\\*";
	const std::string strImageFile = "resources\\video_case_2";
    std::string strOutFile = "./out/";

	vector<string> files;
	files = getFiles(strImagePath);
	/*for (string file : files) {
		cout << file << endl;
	}*/
	vector<vector<float>> cameraIntrinsic(3,vector<float>(3));
	if (!LoadCameraIntrinsicFromFile(strCameraIntrinsicFile.c_str(), cameraIntrinsic))
	{
		return -1;
	}
	string strColorImageFile;
	string strDepthImageFile;
	for (int i = 0; i < 2; i += 2) {
		//std::vector<pointCloudExtraction::Point> pointCloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr finalPointCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		strColorImageFile.assign(strImageFile).append("\\").append(files[i]);
		strDepthImageFile.assign(strImageFile).append("\\").append(files[i+1]);
		if (!GeneratePointCloudFromImage(strDepthImageFile.c_str(), strColorImageFile.c_str(), cameraIntrinsic, pointCloud))
		{
			return -1;
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(pointCloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(2.0);
		sor.filter(*cloud_filtered);

		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
		ne.setInputCloud(cloud_filtered);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
		ne.setSearchMethod(tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius 3cm
		//ne.setRadiusSearch(0.03);
		ne.setKSearch(10);
		// Compute the features
		ne.compute(*cloud_normals);
		pcl::concatenateFields(*cloud_filtered, *cloud_normals, *finalPointCloud);

		
		string name = files[i].substr(0, files[i].rfind("."));
		string strColorPLYFile = strOutFile + name + ".ply";
		cout << strColorPLYFile << endl;
		pcl::io::savePLYFile(strColorPLYFile, *finalPointCloud);
		/*if (!pointCloudExtraction::write_point_cloud(strColorPLYFile, pointCloud, pointCloud->points.size()))
		{
			return -1;
		}*/
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	
	
	
	
   // camera.MovementSpeed = 100.0f;

    // glfw: initialize and configure
    // ------------------------------
//    glfwInit();
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//
//#ifdef __APPLE__
//    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
//#endif
//
//    // glfw window creation
//    // --------------------
//    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
//    if (window == NULL)
//    {
//        std::cout << "Failed to create GLFW window" << std::endl;
//        glfwTerminate();
//        return -1;
//    }
//    glfwMakeContextCurrent(window);
//    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
//    glfwSetCursorPosCallback(window, mouse_callback);
//    glfwSetScrollCallback(window, scroll_callback);
//
//    // tell GLFW to capture our mouse
//    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
//
//    // glad: load all OpenGL function pointers
//    // ---------------------------------------
//    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
//    {
//        std::cout << "Failed to initialize GLAD" << std::endl;
//        return -1;
//    }
//
//    // configure global opengl state
//    // -----------------------------
//    glEnable(GL_DEPTH_TEST);
//
//    // build and compile our shader zprogram
//    // ------------------------------------
//    Shader ourShader("camera.vs", "camera.fs");
//
//    // set up vertex data (and buffer(s)) and configure vertex attributes
//    // ------------------------------------------------------------------ 
//    unsigned int VBO, VAO;
//    glGenVertexArrays(1, &VAO);
//    glGenBuffers(1, &VBO);
//
//    glBindVertexArray(VAO);
//
//    glBindBuffer(GL_ARRAY_BUFFER, VBO);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(pointCloudExtraction::Point) * pointCloud.size(), pointCloud.data(), GL_STATIC_DRAW);
//
//    // position attribute
//    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pointCloudExtraction::Point), (void*)0);
//    glEnableVertexAttribArray(0);
//    // color attribute
//    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(pointCloudExtraction::Point), (void*)(sizeof(glm::vec3)));
//    glEnableVertexAttribArray(1);
//
//	glPointSize(2.0f);
//
//    // render loop
//    // -----------
//    while (!glfwWindowShouldClose(window))
//    {
//        // per-frame time logic
//        // --------------------
//        float currentFrame = glfwGetTime();
//        deltaTime = currentFrame - lastFrame;
//        lastFrame = currentFrame;
//
//        // input
//        // -----
//        processInput(window);
//
//        // render
//        // ------
//        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
//
//        // activate shader
//        ourShader.use();
//
//        // pass projection matrix to shader (note that in this case it could change every frame)
//        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 1000.0f);
//        ourShader.setMat4("projection", projection);
//
//        // camera/view transformation
//        glm::mat4 view = camera.GetViewMatrix();
//        ourShader.setMat4("view", view);
//
//        // render point cloud
//        glBindVertexArray(VAO);
//        glDrawArrays(GL_POINTS, 0, pointCloud.size());
//
//        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
//        // -------------------------------------------------------------------------------
//        glfwSwapBuffers(window);
//        glfwPollEvents();
//    }
//
//    // optional: de-allocate all resources once they've outlived their purpose:
//    // ------------------------------------------------------------------------
//    glDeleteVertexArrays(1, &VAO);
//    glDeleteBuffers(1, &VBO);
//
//    // glfw: terminate, clearing all previously allocated GLFW resources.
//    // ------------------------------------------------------------------
//    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
//void processInput(GLFWwindow *window)
//{
//    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
//        glfwSetWindowShouldClose(window, true);
//
//    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
//        camera.ProcessKeyboard(FORWARD, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
//        camera.ProcessKeyboard(BACKWARD, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
//        camera.ProcessKeyboard(LEFT, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
//        camera.ProcessKeyboard(RIGHT, deltaTime);
//}
//
//// glfw: whenever the window size changed (by OS or user resize) this callback function executes
//// ---------------------------------------------------------------------------------------------
//void framebuffer_size_callback(GLFWwindow* window, int width, int height)
//{
//    // make sure the viewport matches the new window dimensions; note that width and 
//    // height will be significantly larger than specified on retina displays.
//    glViewport(0, 0, width, height);
//}
//
//
//// glfw: whenever the mouse moves, this callback is called
//// -------------------------------------------------------
//void mouse_callback(GLFWwindow* window, double xpos, double ypos)
//{
//    if (firstMouse)
//    {
//        lastX = xpos;
//        lastY = ypos;
//        firstMouse = false;
//    }
//
//    float xoffset = xpos - lastX;
//    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
//
//    lastX = xpos;
//    lastY = ypos;
//
//    camera.ProcessMouseMovement(xoffset, yoffset);
//}
//
//// glfw: whenever the mouse scroll wheel scrolls, this callback is called
//// ----------------------------------------------------------------------
//void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
//{
//    camera.ProcessMouseScroll(yoffset);
//}