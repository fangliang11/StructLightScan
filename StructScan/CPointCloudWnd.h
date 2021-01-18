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


class CPointCloudWnd : public QWidget
{
	Q_OBJECT
public:
	explicit CPointCloudWnd(QVTKOpenGLNativeWidget *wnd, QWidget *parent = Q_NULLPTR);
	~CPointCloudWnd();

private:
	QVTKOpenGLNativeWidget *ui;

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;                       //pcl��������ָ��
	std::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;       //pcl���ӻ�����Ӧʹ�ù�������ָ����򴰿ڻ����
	vtkSmartPointer<vtkRenderer> m_ren;                                //vtk��Ⱦ�������ڿ���һ���������Ⱦ����
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renWnd;            //vtk��Ⱦ�Ĵ��ھ��
	vtkSmartPointer<vtkRenderWindowInteractor> m_iren;                 //vtk�����Ķ���:��ꡢ����


	void initialVtkWidget();

signals:
	void signalOpenPCL();

private slots:
	void onOpenPCL();

};
