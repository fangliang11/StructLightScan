#pragma once

#include <QtWidgets/QMainWindow>
#include <QFileDialog>
#include <QDebug>
#include <QVTKOpenGLNativeWidget.h>
//#include <QVTKOpenGLWidget.h>

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

#include "ui_CloudPointDisplayQt.h"


class vtkImageViewer2;
class vtkRenderer;


class CloudPointDisplayQt : public QMainWindow
{
    Q_OBJECT

public:
    CloudPointDisplayQt(QWidget *parent = Q_NULLPTR);
	~CloudPointDisplayQt();

private:
	void initialVtkWidget();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;                       //pcl��������ָ��
	std::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;       //pcl���ӻ�����Ӧʹ�ù�������ָ����򴰿ڻ����
	vtkSmartPointer<vtkRenderer> m_ren;                                //vtk��Ⱦ�������ڿ���һ���������Ⱦ����
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renWnd;            //vtk��Ⱦ�Ĵ��ھ��
	vtkSmartPointer<vtkRenderWindowInteractor> m_iren;                 //vtk�����Ķ���:��ꡢ����
	vtkSmartPointer<vtkLight> m_light;                                 //vtk����

private:
    Ui::CloudPointDisplayQtClass ui;

private slots:
	void onOpen();
	void onDelete();

};
