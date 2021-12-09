#pragma once


#include <string>
#include <opencv2/opencv.hpp>

#include "CPointCloudWnd.h"




#define IMAGE_NUM 6
#define MATRIX_ROWS 3
#define MATRIX_COLS 3



class CPanoramaStitching
{
public:
	CPanoramaStitching();
	~CPanoramaStitching();

	//typedef pcl::PointXYZRGB PointType;

public:
	std::string m_strImgPath = "";
	int m_FILE_INDEX[7] = { 6, 4, 5, 7, 3, 2, 1 };

	pcl::PointCloud<PointType>::Ptr m_pcloud_stitch;

	// camera parameters
	double m_intrinsics18[3 * 3] = {610.7208, 0, 0, 0, 610.8133, 0, 645.3167, 483.2261, 1};
	double m_intrinsics20[3 * 3] = {596.5926, 0, 0, 0, 596.9962, 0, 629.46, 458.5713, 1};
	double m_distorts18[5] = {-0.3094, 0.0992, -0.0143, 0.000014154, -0.00034282};
	double m_distorts20[5] = {-0.2827, 0.0781, -0.0093, -0.0011, 0.00076695};


	int m_nOpenAngleSelect = 0;
	float m_fOpeningAngle = 0.0f;         // 阵列的张开角度
	float m_fRadius18 = 0.0f;            // 球体半径
	float m_fRadius20 = 0.0f;

	cv::Mat camera_matrixa18;
	cv::Mat camera_matrixa20;
	cv::Mat distCoeffs18;
	cv::Mat distCoeffs20;

	std::vector<std::vector<std::vector<double>>> vecRotation20;
	std::vector<std::vector<std::vector<double>>> vecRotation30;
	std::vector<std::vector<std::vector<double>>> vecRotation40;
	std::vector<std::vector<std::vector<double>>> vecRotation49;
	std::vector<std::vector<std::vector<double>>> vecRotation55;
	std::vector<std::vector<std::vector<double>>> *pvecRotation = nullptr;


	void loadCameraParam();
	void loadRotateMatrix(int angle);
	void rotateMatrixInitial20();
	void rotateMatrixInitial30();
	void rotateMatrixInitial40();
	void rotateMatrixInitial49();
	void rotateMatrixInitial55();
	void ImageStitch(const char* imgpath);
	void ImageStitchUseRotateMatrix(const char* imgpath, int angle);
	void ImageStitchUseRotateMatrix2(const char* imgpath, int angle);
	void SaveCloudFile(const char* cloudname);
	void SaveCloudFile(const char* cloudname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void SetOpeningAngle(float opening_angle);

private:


	int OpencvImgReadUndistort(int index, const char* imgname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	int GenerateCloudFromImage(const char* imgname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void PointProject(PointType input, PointType *output, float radius);
	void SphereProject(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr projected_cloud, float radius);




protected:
	   





};

