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
#include <string>
#include <vector>

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
#include <vtkCamera.h>
#include <vtkLight.h>
#include <vtkCommand.h>
#include <vtkPicker.h>
#include <vtkAxesActor.h>
//#include <vtkImageViewer2.h>
//#include <vtkJPEGReader.h>
//#include <vtkImageActor.h>

//  PCL
#undef min 
#undef max 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/area_picking_event.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
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
#include <pcl/surface/processing.h>
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
	void UpdateThisWnd();
	void UpdatePCLViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn);

private:
	int m_actionCode;
	//  接收到的指令
	enum {
		ACTION_NONE = 0,
		ACTION_OPEN,
		ACTION_SELECT,
		ACTION_DELETE,
		ACTION_ADD,
		ACTION_FILTER,
		ACTION_REMOVE_OUTLIER,
		ACTION_SMOOTH,
		ACTION_MESH,
		ACTION_REBUILD
	};

	QVTKOpenGLNativeWidget *ui;
	std::thread *workThread;
	volatile bool loopFlag;
	void WorkingOnPointCloud();

	pcl::PolygonMesh m_mesh;                                           //存储最终三角化的网格模型
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_sourceCloud;                 //pcl单色点云数据指针
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_displayCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_filteredCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_removeOutlieredCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_smoothedCloud;
	std::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;       //pcl可视化对象，应使用共享智能指针否则窗口会独立
	vtkSmartPointer<vtkRenderer> m_ren;                                //vtk渲染对象：用于控制一个对象的渲染进程
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renWnd;            //vtk渲染的窗口句柄
	vtkSmartPointer<vtkRenderWindowInteractor> m_iren;                 //vtk交互的对象:鼠标、键盘
	//vtkSmartPointer<vtkEventQtSlotConnect> m_vtkEventConnection;     //vtk与qt事件连接
	vtkSmartPointer<vtkLight> light_front;
	vtkSmartPointer<vtkLight> light_back;
	vtkSmartPointer<vtkCamera> camera;

	std::string m_pcdPath;
	bool b_rubber_band_selection_mode;
	bool m_bFilterEnable;
	int m_nFilterMethod;
	float m_fFilterParam1;
	float m_fFilterParam2;
	float m_fFilterParam3;
	bool m_bremoveOutlierEnable;
	float m_fremoveOutlierParam1;
	float m_fremoveOutlierParam2;
	bool m_bsmoothEnable;
	float m_fsmoothParam1;
	float m_fsmoothParam2;
	int m_nRebuildMethod;
	int m_nmeshSearchK;
	float m_fmeshSearchRadius;
	int m_nmeshMaxNeighbors;


	void initialVtkWidget();
	bool ResponseSignals(int code);
	void AddCoordinateSystem();
	void savePointCloudFile();
	void importCloudPoint();
	void displaySelectPCD();
	void displaySphereVTK();
	void displayPCDfile(std::string file_name);
	void clearDisplayCloud();
	void filteredCloud(int method, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudIn, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudOut,
		float param1, float param2, float param3);
	void removeOutlierPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudIn,
		pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudOut,
		float param1, float param2);
	void smoothCloudPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudIn,
		pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudOut,
		float param1, float param2);
	void calculatePointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn,
		pcl::PointCloud<pcl::PointNormal>::Ptr &cloudNormal, int searchK);
	void greedyProjectionTriangula(pcl::PointCloud<pcl::PointNormal>::Ptr &cloudNormal,
		pcl::PolygonMesh &mesh, float searchRadius, int maxNeighbors);
	void poissonRebuild(pcl::PointCloud<pcl::PointNormal>::Ptr &cloudNormal, pcl::PolygonMesh &mesh);
	void buildMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PolygonMesh &mesh);
	void saveMeshToPLY(const char* filename, pcl::PolygonMesh mesh);
	void userSelect();
	static void areaPickCallback(const pcl::visualization::AreaPickingEvent &event, void* args);
	static void pointPickCallback(const pcl::visualization::PointPickingEvent &event, void *args);

public:
	void setProperties(int property, double value, const std::string &id);
	void getCloudPointNumbers(unsigned &number);
	void setCloudPointSize(int size);
	void getCloudPointSize(double &size);
	void setCloudPointColor(int index);
	void displayCoordinate(bool state);
	void filterEnable(bool state);
	void filterMethod(int index);
	void setFilterParams(float param1, float param2, float param3);
	void getFilterParams(float &param1, float &param2, float &param3);
	void removeOutlierEnable(bool state);
	void setRemoveOutlierParams(float param1, float param2);
	void getRemoveOutlierParams(float &param1, float &param2);
	void smoothEnable(bool state);
	void setSmoothParams(float param1, float param2);
	void getSmoothParams(float &param1, float &param2);
	void rebuildMethod(int index);
	void setRebuildParams(int param1, float param2, int param3);
	void getRebuildParams(int &param1, float &param2, int &param3);
	void meshDisplayModel(int index);
	void setMeshColor(int index);

signals:
	void signalUpdateCloudWnd();
	void signalOpenPCL();
	void signalSelect();
	void signalDelete();
	void signalAdd();
	void signalFilter();
	void signalMesh();
	void signalSurfaceRebuild();

private slots:
	void onUpdateCloudWnd();
	void onVtkOpenGLNativeWidgetMouseEvent(QMouseEvent *event);
	void onOpenPCL();
	void onSelect();
	void onDelete();
	void onAdd();
	void onFilter();
	void onMesh();
	void onSurfaceRebuild();

};
