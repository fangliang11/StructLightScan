#include <iostream> //标准输入/输出
#include <fstream>
#include <memory>
#include <algorithm>
#include <math.h>
#include <vector>

#include <boost/thread/thread.hpp>                     //多线程
#include <pcl/common/io.h>
#include <pcl/common/common_headers.h>
//#include <pcl/range_image/range_image.h>               //深度图有关头文件
#include <pcl/io/pcd_io.h>                             //pcd文件输入/输出
//#include <pcl/visualization/range_image_visualizer.h>  //深度图可视化
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>                         //命令行参数解析
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/sample_consensus/sac_model_sphere.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>



typedef pcl::PointXYZRGB PointType;



int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.5, 0.5, 1.0);  //设置背景
	pcl::PointXYZ o;   //点云数据
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);  // 增加球体
	std::cout << "i only run once" << std::endl;
}

//显示字符
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}


void PointProject(PointType input, PointType *output, float radius)
{
	float factor_k = radius / std::sqrt(std::pow(input.x, 2) + std::pow(input.y, 2) + std::pow(input.z, 2));
	output->x = input.x * factor_k;
	output->y = input.y * factor_k;
	output->z = input.z * factor_k;
	//output->z = std::sqrt(std::pow(radius, 2) - std::pow(input.x, 2) - std::pow(input.y, 2));

}


void SphereProject(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr projected_cloud, float radius)
{
	for (int index = 0; index < cloud->width; ++index) {
		PointType projected_point;

		PointProject(cloud->at(index), &projected_point, radius);
		projected_cloud->at(index).x = projected_point.x;
		projected_cloud->at(index).y = projected_point.y;
		projected_cloud->at(index).z = projected_point.z;
		//printf("output:x=%f, y=%f, z=%f\n", cloud->at(index).x, cloud->at(index).y, cloud->at(index).z);

	}
}



/*
// 主函数
int main(int argc, char** argv)
{
	// 打开图像文件, 读取像素值
	int img_width = 1280;
	int img_height = 960;
	std::unique_ptr<unsigned char[]> pbuffer(new unsigned char[img_width * img_height]());
	std::string imgname = "E:\\检测与装备事业部\\4_国防科大柔性阵列\\to_fang\\personal_define\\6.bmp";
	std::ifstream inFile;
	inFile.open(imgname, std::ios::in | std::ios::binary);
	if (!inFile.is_open()) {
		printf("cannot open image\n");
		return -1;
	}
	inFile.seekg(1078, inFile.beg);   // 输入文件偏移1078字节：54 + 1024
	inFile.read((char*)&pbuffer[0], img_width * img_height);
	if (pbuffer == nullptr) {
		printf("read pixel value error: %d\n", GetLastError());
		return -1;
	}

	//读取pcd文件。如果没有指定文件，则创建样本云点
	pcl::PointCloud<PointType>::Ptr cloud;
	cloud.reset(new pcl::PointCloud<PointType>);
	pcl::PointXYZ center(img_width / 2 + 0.5f, img_height / 2 + 0.5f, 0);
	for (int j = 0; j < img_height; ++j) {
		for (int i = 0; i < img_width; ++i) {
			PointType point;
			point.x = (float)i - center.x;
			point.y = (float)j - center.y;
			point.z = 0.0f - center.z;
			point.r = pbuffer[j * img_width + i];
			point.g = pbuffer[j * img_width + i];
			point.b = pbuffer[j * img_width + i];
			cloud->push_back(point);
		}
	}
	cloud->width = (int)cloud->points.size();
	cloud->height = 1;

	float radius = 1000.0f;
	// 旋转平移
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.0, 0.0, -radius;
	pcl::transformPointCloud(*cloud, *cloud, transform);
	   	 

	//pcl::PointCloud<PointType>::Ptr projected_cloud;
	//projected_cloud.reset(new pcl::PointCloud<PointType>);
	//pcl::copyPointCloud(*cloud, *projected_cloud);

	SphereProject(cloud, cloud, radius);

	std::cout << "point 10 position x|y|z = " << int(cloud->points[9].x)
		<< "  " << int(cloud->points[9].y)
		<< "  " << int(cloud->points[9].z) << std::endl;

	std::cout << "point 10 color r|g|b = " << int(cloud->points[9].r)
		<< "  " << int(cloud->points[9].g)
		<< "  " << int(cloud->points[9].b) << std::endl;

	// 平移回来
	Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
	transform2.translation() << 0.0, 0.0, radius;
	pcl::transformPointCloud(*cloud, *cloud, transform2);


	//// initilaze 可视化
	//pcl::visualization::CloudViewer viewer("Cloud Viewer");
	////blocks until the cloud is actually rendered
	//viewer.showCloud(cloud);
	////use the following functions to get access to the underlying more advanced/powerful
	////PCLVisualizer
	////This will only get called once
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);

	////This will get called once per visualization iteration
	//viewer.runOnVisualizationThread(viewerPsycho);
	//while (!viewer.wasStopped()){
	//	//you can also do cool processing here
	//	//FIXME: Note that this is running in a separate thread from viewerPsycho
	//	//and you should guard against race conditions yourself...
	//	user_data++;
	//}

	////双视口显示
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("test Viewer"));
	//viewer->initCameraParameters();
	//int v1(0), v2(0);
	////原始点云窗口
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->setCameraPosition(1.3*radius, 1.3*radius, 1.3*radius, 0, 0, 0, v1);
	//viewer->setBackgroundColor(0, 0, 0, v1);
	//viewer->addText("original", 10, 10, "v1 text", v1);
	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud1", v1);
	//viewer->addCoordinateSystem(1.5*radius);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	////滤波窗口
	//viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	////viewer->setCameraPosition(100.0, 100.0, 100.0, 0, 0, 0, v2);
	//viewer->setBackgroundColor(0, 0, 0, v2);
	//viewer->addText("cloud_projected", 10, 10, "v2 text", v2);
	//viewer->addPointCloud<pcl::PointXYZRGB>(projected_cloud, "sample cloud2", v2);
	//viewer->addCoordinateSystem(1.5*radius);
	//viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.01, 0.5, 0.8, 0.2, "sphere");   // 原点
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);  //刷新
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	// 保存点云
	std::string strout = "E:\\检测与装备事业部\\4_国防科大柔性阵列\\to_fang\\personal_define\\6.pcd";
	pcl::PCDWriter writer;
	if (!cloud->empty()) {
		writer.write(strout, *cloud);
		//pcl::io::savePCDFileBinaryCompressed(strout, cloud);
		//pcl::io::savePCDFileASCII(strout, *cloud);
	}
	printf("finished\n");

	return 0;
}
*/






int img_width = 1280;
int img_height = 960;
double camera_width = img_width * 0.00375;    //  mm
double camera_height = img_height * 0.00375;
double dx = camera_width / 1280.0;
double dy = camera_height / 960.0;
double cx = (camera_width + 0.00375) / 2.0;
double cy = (camera_height + 0.00375) / 2.0;

//  相机矩阵 [{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}]
cv::Mat camera_matrixa18 = (cv::Mat_<double>(3, 3) << 610.7208, 0.0, 645.3167, 0, 610.8133, 483.2261, 0, 0, 1);
cv::Mat camera_matrixa20 = (cv::Mat_<double>(3, 3) << 596.5926, 0.0, 629.46, 0, 596.9962, 458.5713, 0, 0, 1);

//  畸变系数向量 (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]]),默认值为0
cv::Mat distCoeffs18 = (cv::Mat_<double >(5, 1) << -0.3094, 0.0992, 0.000014154, -0.00034282, -0.0143);
cv::Mat distCoeffs20 = (cv::Mat_<double >(5, 1) << -0.2827, 0.0781, -0.0011, 0.00076695, -0.0093);



#define PI_3 M_PI/3.0
//float round_angle = 2 * M_PI / 6.0;		                    // 周围相机在圆周上的分布角度
float opening_angle_arc = 40.0;                             // 阵列的张开角度
float opening_angle = opening_angle_arc * M_PI / 180;
float factor_center_round = 596.7 / 610.72;                 // 中心相机视角/周围相机视角
float radius18 = 610.72;                       // 球体半径
float radius20 = 596.7;

#define IMAGE_NUM 6
#define MATRIX_ROWS 3
#define MATRIX_COLS 3
std::vector<std::vector<std::vector<double>>> vecRotation40;


void matrixInitial(int angle)
{
	vecRotation40.resize(IMAGE_NUM);
	for (int index = 0; index < IMAGE_NUM; ++index) {
		vecRotation40[index].resize(MATRIX_ROWS);
		for (int row = 0; row < MATRIX_ROWS; ++row) {
			vecRotation40[index][row].resize(MATRIX_COLS);
		}
	}
	vecRotation40[0][0][0] = 1.0248;
	vecRotation40[0][0][1] = 0.0302;
	vecRotation40[0][0][2] = -0.0264;
	vecRotation40[0][1][0] = 0.0026;
	vecRotation40[0][1][1] = 0.8190;
	vecRotation40[0][1][2] = 0.6585;
	vecRotation40[0][2][0] = 0.0308;
	vecRotation40[0][2][1] = -0.6639;
	vecRotation40[0][2][2] = 0.7456;

	vecRotation40[1][0][0] = 0.8516;
	vecRotation40[1][0][1] = -0.0967;
	vecRotation40[1][0][2] = 0.5573;
	vecRotation40[1][1][0] = -0.0842;
	vecRotation40[1][1][1] = 0.9832;
	vecRotation40[1][1][2] = 0.3032;
	vecRotation40[1][2][0] = -0.5660;
	vecRotation40[1][2][1] = -0.2814;
	vecRotation40[1][2][2] = 0.7788;

	vecRotation40[2][0][0] = 0.8439;
	vecRotation40[2][0][1] = 0.1004;
	vecRotation40[2][0][2] = 0.5470;
	vecRotation40[2][1][0] = 0.0847;
	vecRotation40[2][1][1] = 0.9598;
	vecRotation40[2][1][2] = -0.3075;
	vecRotation40[2][2][0] = -0.5488;
	vecRotation40[2][2][1] = 0.3033;
	vecRotation40[2][2][2] = 0.7785;

	vecRotation40[3][0][0] = 1.0085;
	vecRotation40[3][0][1] = -0.0095;
	vecRotation40[3][0][2] = -0.0121;
	vecRotation40[3][1][0] = -0.0037;
	vecRotation40[3][1][1] = 0.7633;
	vecRotation40[3][1][2] = -0.6064;
	vecRotation40[3][2][0] = 0.0072;
	vecRotation40[3][2][1] = 0.6002;
	vecRotation40[3][2][2] = 0.7937;

	vecRotation40[4][0][0] = 0.8657;
	vecRotation40[4][0][1] = -0.1067;
	vecRotation40[4][0][2] = -0.4934;
	vecRotation40[4][1][0] = -0.0922;
	vecRotation40[4][1][1] = 0.9378;
	vecRotation40[4][1][2] = -0.3038;
	vecRotation40[4][2][0] = 0.4893;
	vecRotation40[4][2][1] = 0.2879;
	vecRotation40[4][2][2] = 0.8167;

	vecRotation40[5][0][0] = 0.7778;
	vecRotation40[5][0][1] = 0.0375;
	vecRotation40[5][0][2] = -0.6001;
	vecRotation40[5][1][0] = 0.0478;
	vecRotation40[5][1][1] = 0.9974;
	vecRotation40[5][1][2] = 0.2111;
	vecRotation40[5][2][0] = 0.5651;
	vecRotation40[5][2][1] = -0.2066;
	vecRotation40[5][2][2] = 0.7817;
}


int readImg(const char* imgname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	// 打开图像文件, 读取像素值
	std::unique_ptr<unsigned char[]> pbuffer(new unsigned char[img_width * img_height]());
	//std::string imgname = "E:\\检测与装备事业部\\4_国防科大柔性阵列\\to_fang\\60\\6.bmp";
	std::ifstream inFile;
	inFile.open(imgname, std::ios::in | std::ios::binary);
	if (!inFile.is_open()) {
		printf("cannot open image %s\n", imgname);
		return -1;
	}
	inFile.seekg(1078, inFile.beg);   // 输入文件偏移1078字节：54 + 1024
	inFile.read((char*)&pbuffer[0], img_width * img_height);
	if (pbuffer == nullptr) {
		printf("read pixel value error: %d\n", GetLastError());
		return -1;
	}

	// 要将图像坐标转换为相机坐标
	pcl::PointXYZ center(img_width / 2 + 0.5f, img_height / 2 + 0.5f, 0);
	//pcl::PointXYZ center(cx, cy, 0.0);
	for (int j = 0; j < img_height; ++j) {
		for (int i = 0; i < img_width; ++i) {
			pcl::PointXYZRGB point;
			point.x = (float)(i + 1) - center.x;
			point.y = (float)(j + 1) - center.y;
			point.z = 0.0f;
			point.r = pbuffer[j * img_width + i];
			point.g = pbuffer[j * img_width + i];
			point.b = pbuffer[j * img_width + i];
			cloud->push_back(point);
		}
	}
	cloud->width = (int)cloud->points.size();
	cloud->height = 1;
	return 0;
}


int opencvReadImg(int index, const char* imgname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	cv::Mat src = cv::imread(imgname, CV_8UC1);
	if (src.empty()) {
		printf("image path is empty\n");
		return -1;
	}
	cv::Mat undistor;
	if(index == 0)
		cv::undistort(src, undistor, camera_matrixa20, distCoeffs20);
	else
		cv::undistort(src, undistor, camera_matrixa18, distCoeffs18);

	int width = undistor.cols;
	int height = undistor.rows;

	// 要将图像坐标转换为相机坐标
	pcl::PointXYZ center(width / 2 + 0.5f, height / 2 + 0.5f, 0);
	for (int j = 0; j < height; ++j) {
		uchar* pdata = undistor.ptr<uchar>(height - j - 1);
		for (int i = 0; i < width; ++i) {
			pcl::PointXYZRGB point;
			point.y = (float)(i + 1) - center.x;
			point.x = -(float)(j + 1) + center.y;
			if (index == 0)
				point.z = radius20;
			else
				point.z = radius18;

			if (index == 0) {
				if (pdata[i] < 255)
					pdata[i] = pdata[i] * 0.7;
			}
			if (index == 6) {
				if (pdata[i] < 255)
					pdata[i] = pdata[i] * 0.9;
			}
			point.r = pdata[i];
			point.g = pdata[i];
			point.b = pdata[i];
			cloud->push_back(point);
		}
	}
	cloud->width = (int)cloud->points.size();
	cloud->height = 1;

	//printf("imgwidth = %d, imgheight = %d, cloud_width = %d\n", width, height, cloud->width);

	//cv::namedWindow("undis");
	//cv::imshow("undis", undistor);
	//cv::waitKey(0);

	return 0;
}


int main(int argc, char** argv)
{
	//std::string strpath = "E:\\检测与装备事业部\\4_国防科大柔性阵列\\to_fang\\personal_define\\";
	std::string strpath = "E:\\检测与装备事业部\\4_国防科大柔性阵列\\ceshi\\40\\";
	int file_INDEX[7] = {6, 4, 5, 7, 3, 2, 1 };

	//  旋转矩阵
	matrixInitial(40);

	pcl::PointCloud<PointType>::Ptr stitch_cloud;
	stitch_cloud.reset(new pcl::PointCloud<PointType>);	
	float angleX, angleY, angleZ, x, y, z;
	for (int index = 0; index <= 6; ++index) {
		pcl::PointCloud<PointType>::Ptr cloud;
		cloud.reset(new pcl::PointCloud<PointType>);
		//  1. 读图，矫正畸变，转换为点云坐标
		std::string strimg = strpath + std::to_string(file_INDEX[index]) + ".bmp";
		opencvReadImg(index, strimg.c_str(), cloud);
		printf("%d before rotate: x = %f, y = %f, z = %f\n", index, cloud->points.at(0).x, cloud->points.at(0).y, cloud->points.at(0).z);
		// 所有点云均首先变换至1#初始位置
		if (index != 0) {
			Eigen::Affine3f initial_transform = Eigen::Affine3f::Identity();
			initial_transform.rotate(Eigen::AngleAxisf(-(M_PI_2 + PI_3 * (index - 1)), Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud, *cloud, initial_transform);
		}
		//  2.投影至球面
		SphereProject(cloud, cloud, radius18);
		//  3.乘以旋转矩阵
		for (int i = 0; i < cloud->points.size(); ++i) {
			float nx = cloud->points.at(i).x;
			float ny = cloud->points.at(i).y;
			float nz = cloud->points.at(i).z;
			if (index == 0) {
				cloud->points[i].x = nx * factor_center_round;
				cloud->points[i].y = -ny * factor_center_round;
				cloud->points[i].z = nz * factor_center_round;
			}
			else {
				cloud->points[i].x =
					vecRotation40[index - 1][0][0] * nx +
					vecRotation40[index - 1][0][1] * ny +
					vecRotation40[index - 1][0][2] * nz;
				cloud->points[i].y = -(
					vecRotation40[index - 1][1][0] * nx +
					vecRotation40[index - 1][1][1] * ny +
					vecRotation40[index - 1][1][2] * nz);
				cloud->points[i].z =
					vecRotation40[index - 1][2][0] * nx +
					vecRotation40[index - 1][2][1] * ny +
					vecRotation40[index - 1][2][2] * nz;
			}
		}

		*stitch_cloud += *cloud;
		printf("load image %d finished\n", index);
	}

	// 显示窗
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(stitch_cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(stitch_cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.5*radius18);
	viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.01, 0.5, 0.8, 0.2, "sphere");   // 原点
	viewer->initCameraParameters();
	viewer->setCameraPosition(1.3*radius18, 1.3*radius18, 1.3*radius18, 0, 0, 0);
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);  //刷新
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	return 0;
}



int charmain(int argc, char** argv)
{
	std::string strpath = "E:\\检测与装备事业部\\4_国防科大柔性阵列\\to_fang\\personal_define\\";
	//读取pcd文件。如果没有指定文件，则创建样本云点
	pcl::PointCloud<PointType>::Ptr cloud0, cloud1, cloud2, cloud3, cloud4, cloud5, cloud6;
	cloud0.reset(new pcl::PointCloud<PointType>);
	cloud1.reset(new pcl::PointCloud<PointType>);
	cloud2.reset(new pcl::PointCloud<PointType>);
	cloud3.reset(new pcl::PointCloud<PointType>);
	cloud4.reset(new pcl::PointCloud<PointType>);
	cloud5.reset(new pcl::PointCloud<PointType>);
	cloud6.reset(new pcl::PointCloud<PointType>);

	pcl::PointCloud<PointType>::Ptr stitch_cloud;
	stitch_cloud.reset(new pcl::PointCloud<PointType>);

	//pcl::PointCloud<PointType>::Ptr projected_cloud;
	//projected_cloud.reset(new pcl::PointCloud<PointType>);


	float round_angle = 2 * M_PI / 6.0;		//周围相机在圆周上的分布角度
	float opening_angle = -M_PI / 3.0;      //阵列的张开角度
	float radius = 1000.0f;
	for (int index = 0; index <= 6; ++index) {
		std::string filename = std::to_string(index);
		filename = strpath + filename + ".pcd";

		// 读取点云并进行旋转平移: 先旋转，后平移
		// 旋转的顺序决定了最终位置
		pcl::PCLPointCloud2 cloudhead;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		int pcd_version;
		int data_type;
		unsigned int data_idx;
		int offset = 0;
		pcl::PCDReader rd;
		rd.readHeader(filename, cloudhead, origin, orientation, pcd_version, data_type, data_idx);
		switch (index)
		{
		case 0: {
			rd.read<PointType>(filename, *cloud0);
			float angleX = -M_PI_2;
			float angleY = 0.0;
			float angleZ = M_PI;
			Eigen::Affine3f angle_transform = Eigen::Affine3f::Identity();
			angle_transform.rotate(Eigen::AngleAxisf(angleX, Eigen::Vector3f::UnitX()));
			angle_transform.rotate(Eigen::AngleAxisf(angleY, Eigen::Vector3f::UnitY()));
			angle_transform.rotate(Eigen::AngleAxisf(angleZ, Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud0, *cloud0, angle_transform);
			float x = 0.0;
			float y = -radius;
			float z = 0.0;
			Eigen::Affine3f pos_transform = Eigen::Affine3f::Identity();
			pos_transform.translation() << x, y, z;
			pcl::transformPointCloud(*cloud0, *cloud0, pos_transform);
			//SphereProject(cloud0, cloud0, radius);
			break;
		}
		case 1: {
			rd.read<PointType>(filename, *cloud1);
			float angleX = opening_angle;
			float angleY = -round_angle * (index - 1);
			float angleZ = 0.0;
			Eigen::Affine3f angleX_transform = Eigen::Affine3f::Identity();
			angleX_transform.rotate(Eigen::AngleAxisf(angleX, Eigen::Vector3f::UnitX()));
			pcl::transformPointCloud(*cloud1, *cloud1, angleX_transform);
			Eigen::Affine3f angleY_transform = Eigen::Affine3f::Identity();
			angleY_transform.rotate(Eigen::AngleAxisf(angleY, Eigen::Vector3f::UnitY()));
			pcl::transformPointCloud(*cloud1, *cloud1, angleY_transform);
			Eigen::Affine3f angleZ_transform = Eigen::Affine3f::Identity();
			angleZ_transform.rotate(Eigen::AngleAxisf(angleZ, Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud1, *cloud1, angleZ_transform);
			float x = 0.0;
			float y = radius * sin(opening_angle);
			float z = -radius;
			Eigen::Affine3f pos_transform = Eigen::Affine3f::Identity();
			pos_transform.translation() << x, y, z;
			pcl::transformPointCloud(*cloud1, *cloud1, pos_transform);
			//SphereProject(cloud1, cloud1, radius);
			break;
		}
		case 2: {
			rd.read<PointType>(filename, *cloud2);
			float angleX = opening_angle;
			float angleY = -round_angle * (index - 1);
			float angleZ = 0.0;
			Eigen::Affine3f angleX_transform = Eigen::Affine3f::Identity();
			angleX_transform.rotate(Eigen::AngleAxisf(angleX, Eigen::Vector3f::UnitX()));
			pcl::transformPointCloud(*cloud2, *cloud2, angleX_transform);
			Eigen::Affine3f angleY_transform = Eigen::Affine3f::Identity();
			angleY_transform.rotate(Eigen::AngleAxisf(angleY, Eigen::Vector3f::UnitY()));
			pcl::transformPointCloud(*cloud2, *cloud2, angleY_transform);
			Eigen::Affine3f angleZ_transform = Eigen::Affine3f::Identity();
			angleZ_transform.rotate(Eigen::AngleAxisf(angleZ, Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud2, *cloud2, angleZ_transform);
			float x = radius * sin(round_angle);
			float y = radius * sin(opening_angle);
			float z =-radius * cos(round_angle);
			Eigen::Affine3f pos_transform = Eigen::Affine3f::Identity();
			pos_transform.translation() << x, y, z;
			pcl::transformPointCloud(*cloud2, *cloud2, pos_transform);
			//SphereProject(cloud2, cloud2, radius);
			break;
		}
		case 3: {
			rd.read<PointType>(filename, *cloud3);
			float angleX = opening_angle;
			float angleY = -round_angle * (index - 1);
			float angleZ = 0.0;
			Eigen::Affine3f angleX_transform = Eigen::Affine3f::Identity();
			angleX_transform.rotate(Eigen::AngleAxisf(angleX, Eigen::Vector3f::UnitX()));
			pcl::transformPointCloud(*cloud3, *cloud3, angleX_transform);
			Eigen::Affine3f angleY_transform = Eigen::Affine3f::Identity();
			angleY_transform.rotate(Eigen::AngleAxisf(angleY, Eigen::Vector3f::UnitY()));
			pcl::transformPointCloud(*cloud3, *cloud3, angleY_transform);
			Eigen::Affine3f angleZ_transform = Eigen::Affine3f::Identity();
			angleZ_transform.rotate(Eigen::AngleAxisf(angleZ, Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud3, *cloud3, angleZ_transform);
			float x = radius * sin(round_angle);
			float y = radius * sin(opening_angle);
			float z = radius * cos(round_angle);
			Eigen::Affine3f pos_transform = Eigen::Affine3f::Identity();
			pos_transform.translation() << x, y, z;
			pcl::transformPointCloud(*cloud3, *cloud3, pos_transform);
			//SphereProject(cloud3, cloud3, radius);
			break;
		}
		case 4: {
			rd.read<PointType>(filename, *cloud4);
			float angleX = opening_angle;
			float angleY = -round_angle * (index - 1);
			float angleZ = 0.0;
			Eigen::Affine3f angleX_transform = Eigen::Affine3f::Identity();
			angleX_transform.rotate(Eigen::AngleAxisf(angleX, Eigen::Vector3f::UnitX()));
			pcl::transformPointCloud(*cloud4, *cloud4, angleX_transform);
			Eigen::Affine3f angleY_transform = Eigen::Affine3f::Identity();
			angleY_transform.rotate(Eigen::AngleAxisf(angleY, Eigen::Vector3f::UnitY()));
			pcl::transformPointCloud(*cloud4, *cloud4, angleY_transform);
			Eigen::Affine3f angleZ_transform = Eigen::Affine3f::Identity();
			angleZ_transform.rotate(Eigen::AngleAxisf(angleZ, Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud4, *cloud4, angleZ_transform);
			float x = 0.0;
			float y = radius * sin(opening_angle);
			float z = radius;
			Eigen::Affine3f pos_transform = Eigen::Affine3f::Identity();
			pos_transform.translation() << x, y, z;
			pcl::transformPointCloud(*cloud4, *cloud4, pos_transform);
			//SphereProject(cloud4, cloud4, radius);
			break;
		}
		case 5: {
			rd.read<PointType>(filename, *cloud5);
			float angleX = opening_angle;
			float angleY = -round_angle * (index - 1);
			float angleZ = 0.0;
			Eigen::Affine3f angleX_transform = Eigen::Affine3f::Identity();
			angleX_transform.rotate(Eigen::AngleAxisf(angleX, Eigen::Vector3f::UnitX()));
			pcl::transformPointCloud(*cloud5, *cloud5, angleX_transform);
			Eigen::Affine3f angleY_transform = Eigen::Affine3f::Identity();
			angleY_transform.rotate(Eigen::AngleAxisf(angleY, Eigen::Vector3f::UnitY()));
			pcl::transformPointCloud(*cloud5, *cloud5, angleY_transform);
			Eigen::Affine3f angleZ_transform = Eigen::Affine3f::Identity();
			angleZ_transform.rotate(Eigen::AngleAxisf(angleZ, Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud5, *cloud5, angleZ_transform);
			float x = -radius * sin(round_angle);
			float y = radius * sin(opening_angle);
			float z = radius * cos(round_angle);
			Eigen::Affine3f pos_transform = Eigen::Affine3f::Identity();
			pos_transform.translation() << x, y, z;
			pcl::transformPointCloud(*cloud5, *cloud5, pos_transform);
			//SphereProject(cloud5, cloud5, radius);
			break;
		}
		case 6: {
			rd.read<PointType>(filename, *cloud6);
			float angleX = opening_angle;
			float angleY = -round_angle * (index - 1);
			float angleZ = 0.0;
			Eigen::Affine3f angleX_transform = Eigen::Affine3f::Identity();
			angleX_transform.rotate(Eigen::AngleAxisf(angleX, Eigen::Vector3f::UnitX()));
			pcl::transformPointCloud(*cloud6, *cloud6, angleX_transform);
			Eigen::Affine3f angleY_transform = Eigen::Affine3f::Identity();
			angleY_transform.rotate(Eigen::AngleAxisf(angleY, Eigen::Vector3f::UnitY()));
			pcl::transformPointCloud(*cloud6, *cloud6, angleY_transform);
			Eigen::Affine3f angleZ_transform = Eigen::Affine3f::Identity();
			angleZ_transform.rotate(Eigen::AngleAxisf(angleZ, Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud6, *cloud6, angleZ_transform);
			float x = -radius * sin(round_angle);
			float y = radius * sin(opening_angle);
			float z = -radius * cos(round_angle);
			Eigen::Affine3f pos_transform = Eigen::Affine3f::Identity();
			pos_transform.translation() << x, y, z;
			pcl::transformPointCloud(*cloud6, *cloud6, pos_transform);
			//SphereProject(cloud6, cloud6, radius);
			break;
		}
		}
		printf("load pcd file %d finished\n", index);
	}

	*stitch_cloud += *cloud0;
	*stitch_cloud += *cloud1;
	*stitch_cloud += *cloud2;
	*stitch_cloud += *cloud3;
	*stitch_cloud += *cloud4;
	*stitch_cloud += *cloud5;
	*stitch_cloud += *cloud6;


	// 显示窗
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(stitch_cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(stitch_cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.5*radius);
	viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.01, 0.5, 0.8, 0.2, "sphere");   // 原点
	viewer->initCameraParameters();
	viewer->setCameraPosition(1.3*radius, 1.3*radius, 1.3*radius, 0, 0, 0);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);  //刷新
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	////双视口显示
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("test Viewer"));
	//viewer->initCameraParameters();
	//int v1(0);
	////原始点云窗口
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->setCameraPosition(1.3*radius, 1.3*radius, 1.3*radius, 0, 0, 0, v1);
	//viewer->setBackgroundColor(0, 0, 0, v1);
	//viewer->addText("original", 10, 10, "v1 text", v1);
	//viewer->addPointCloud<pcl::PointXYZRGB>(stitch_cloud, "sample cloud1", v1);
	//viewer->addCoordinateSystem(1.5*radius);
	//viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.01, 0.5, 0.8, 0.2, "sphere");   // 原点
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);  //刷新
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	return 0;
}



