#pragma once

#include <QtWidgets/QMainWindow>
#include <QFileDialog>
#include <QVTKOpenGLNativeWidget.h>

#include <memory>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
#include <vtkSmartPointer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
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

#include "ui_CloudPointDisplayQt.h"


class vtkImageViewer2;
class vtkRenderer;


class CloudPointDisplayQt : public QMainWindow
{
    Q_OBJECT

public:
    CloudPointDisplayQt(QWidget *parent = Q_NULLPTR);


private:
	void initialVtkWidget();

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;                       //pcl点云数据指针
	std::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;       //pcl可视化对象，应使用共享智能指针否则窗口会独立
	vtkSmartPointer<vtkRenderer> m_ren;                                //vtk渲染对象：用于控制一个对象的渲染进程
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renWnd;            //vtk渲染的窗口句柄
	vtkSmartPointer<vtkRenderWindowInteractor> m_iren;                 //vtk交互的对象:鼠标、键盘


private:
    Ui::CloudPointDisplayQtClass ui;

private slots:
	void onOpen();

};
