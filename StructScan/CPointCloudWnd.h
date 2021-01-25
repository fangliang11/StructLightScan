#pragma once

//  Qt
#include <QWidget>
#include <QGLWidget>
#include <QFileDialog>
#include <QOpenGLWidget>
#include <QDebug>
#include <QStyleOption>
#include <QPainter>
#include <QMouseEvent>
#include <QKeyEvent>
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
#include <vtkEventQtSlotConnect.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkPolyVertex.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkPicker.h>
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
	void DestroyThisWnd();

private:
	QVTKOpenGLNativeWidget *ui;
	std::thread *workThread;
	volatile bool loopFlag;
	void WorkingOnPointCloud();

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_monoCloud;                   //pcl��ɫ��������ָ��
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_colorCloud;
	std::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;       //pcl���ӻ�����Ӧʹ�ù�������ָ����򴰿ڻ����
	vtkSmartPointer<vtkRenderer> m_ren;                                //vtk��Ⱦ�������ڿ���һ���������Ⱦ����
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renWnd;            //vtk��Ⱦ�Ĵ��ھ��
	vtkSmartPointer<vtkRenderWindowInteractor> m_iren;                 //vtk�����Ķ���:��ꡢ����
	//vtkSmartPointer<vtkEventQtSlotConnect> m_vtkEventConnection;     //vtk��qt�¼�����

	//void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
	//void resizeEvent(QResizeEvent *event) Q_DECL_OVERRIDE;
	void initialVtkWidget();
	void Draw();
	bool ResponseSignals(int code);
	void displaySelectPCD();
	void displaySphere();
	void displayPCDfile();
	void RebuildTest();
	void filteredCloud();



	int m_actionCode;
	//  ���յ���ָ��
	enum {
		ACTION_NONE = 0,
		ACTION_OPEN,
		ACTION_SELECT,
		ACTION_DELETE,
		ACTION_ADD,
		ACTION_CLEAR,
		ACTION_REBUILD
	};
signals:
	void signalUpdateCloudWnd();
	void signalOpenPCL();
	void signalSelect();
	void signalDelete();
	void signalAdd();
	void signalClear();
	void signalSurfaceRebuild();

private slots:
	void onUpdateCloudWnd();
	void onVtkOpenGLNativeWidgetMouseEvent(QMouseEvent *event);
	void onOpenPCL();
	void onSelect();
	void onDelete();
	void onAdd();
	void onClear();
	void onSurfaceRebuild();

};
