// ImportPCD.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <memory>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <vtkSmartPointer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkLight.h>
//#include <vtkImageViewer2.h>
//#include <vtkJPEGReader.h>
//#include <vtkImageActor.h>

//  PCL
#undef min 
#undef max 
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>



int main()
{
    std::cout << "Hello World!\n";

	std::string file_name = "C:\\Users\\fang\\Desktop\\柔电院全景\\source.pcd";
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::Normal> ncloud;


	pcl::PCLPointCloud2 cloud2;
	Eigen::Vector4f origin;
	Eigen::Quaternionf orientation;
	int pcd_version;
	int data_type;
	unsigned int data_idx;
	int offset = 0;
	pcl::PCDReader rd;
	rd.readHeader(file_name, cloud2, origin, orientation, pcd_version,
		data_type, data_idx);
	rd.read<pcl::PointXYZRGB>(file_name, cloud);
	rd.read<pcl::Normal>(file_name, ncloud);
	unsigned int pointsize = cloud.points.size();
	std::cout << "point size = " << pointsize << std::endl;
	for (unsigned int i = 0; i < pointsize; i++) {
		cloud.points[i].r = ncloud.points[i].normal_x;
		cloud.points[i].g = ncloud.points[i].normal_y;
		cloud.points[i].b = ncloud.points[i].normal_z;
	}

	std::cout << "point 10 color r|g|b = " << int(cloud.points[9].r)
		<< "  " << int(cloud.points[9].g)
		<< "  " << int(cloud.points[9].b) << std::endl;
	std::string strout("C:\\Users\\fang\\Desktop\\柔电院全景\\source_binary.pcd");
	pcl::PCDWriter writer;
	if (!cloud.empty()) {
		//writer.write(strout, *cloud);
		pcl::io::savePCDFileBinaryCompressed(strout, cloud);
		//pcl::io::savePCDFileASCII(strout, cloud);
	}


	std::cout << "finished\n";
}
