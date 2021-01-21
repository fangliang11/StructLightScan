#pragma once

//  Qt
#include <QWidget>
#include <QFileDialog>
#include <QOpenGLWidget>
#include <QDebug>
#include <QStyleOption>
#include <QPainter>
#include <QVTKOpenGLNativeWidget.h>

//  std
#include <memory>
#include <thread>

//  vtk
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <vtkSmartPointer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkPolyVertex.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
//#include <vtkImageViewer2.h>
//#include <vtkJPEGReader.h>
//#include <vtkImageActor.h>

//  PCL
#undef min 
#undef max 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/segmentation/sac_segmentation.h>


class CPointCloudWnd : public QWidget
{
	Q_OBJECT
public:
	explicit CPointCloudWnd(QVTKOpenGLNativeWidget *wnd, QWidget *parent = Q_NULLPTR);
	~CPointCloudWnd();

private:
	QVTKOpenGLNativeWidget *ui;
	std::thread *workThread;
	void WorkingOnPointCloud();

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_monoCloud;                   //pcl单色点云数据指针
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_colorCloud;
	std::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;       //pcl可视化对象，应使用共享智能指针否则窗口会独立
	vtkSmartPointer<vtkRenderer> m_ren;                                //vtk渲染对象：用于控制一个对象的渲染进程
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renWnd;            //vtk渲染的窗口句柄
	vtkSmartPointer<vtkRenderWindowInteractor> m_iren;                 //vtk交互的对象:鼠标、键盘


	unsigned red;
	unsigned green;
	unsigned blue;
	unsigned size;

	void initialVtkWidget();
	void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);


	void displayOnVTK1();
	void displaySphere();

	void RebuildTest();
	void Draw();


signals:
	void signalOpenPCL();
	void signalSelect();
	void signalDelete();
	void signalAdd();
	void signalClear();
	void signalSurfaceRebuild();

private slots:
	void onOpenPCL();
	void onSelect();
	void onDelete();
	void onAdd();
	void onClear();
	void onSurfaceRebuild();

};
