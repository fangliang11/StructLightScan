#include "CloudPointDisplayQt.h"

CloudPointDisplayQt::CloudPointDisplayQt(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

	initialVtkWidget();

	//connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));	
	//connect(ui.actionDelete, SIGNAL(trigggered()), this, SLOT(onDelete()));
}


CloudPointDisplayQt::~CloudPointDisplayQt()
{

}


void CloudPointDisplayQt::initialVtkWidget()
{
	//创建vtk渲染对象控制和渲染窗口
	m_ren = vtkSmartPointer<vtkRenderer>::New();
	m_light = vtkSmartPointer<vtkLight>::New();
	m_light->SetColor(1, 1, 1);
	//m_light->SetPosition(100, 100, 100);
	//m_light->SetFocalPoint(m_ren->GetActiveCamera()->GetFocalPoint());

	m_ren->AddLight(m_light); 
	m_renWnd = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	m_renWnd->AddRenderer(m_ren);

	m_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	//绑定pcl可视化对象到vtk渲染窗口
	m_viewer.reset(new pcl::visualization::PCLVisualizer(m_ren, m_renWnd, "viewer", false));
	m_viewer->addPointCloud(m_cloud, "cloud");
	m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "sample cloud");
	m_viewer->addCoordinateSystem(1);

	//绑定渲染窗口，绑定事件交互
	ui.vtkOpenGLWidget->SetRenderWindow(m_viewer->getRenderWindow());
	m_viewer->setupInteractor(ui.vtkOpenGLWidget->GetInteractor(),
		ui.vtkOpenGLWidget->GetRenderWindow());
	m_viewer->setBackgroundColor(0.01, 0.5, 0.6);

	/*{
		m_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
		vtkNew<vtkGenericOpenGLRenderWindow> window;
		window->AddRenderer(m_viewer->getRendererCollection()->GetFirstRenderer());
		ui.vtkOpenGLWidget->SetRenderWindow(window.Get());
		m_viewer->setupInteractor(ui.vtkOpenGLWidget->GetInteractor(),
		ui.vtkOpenGLWidget->GetRenderWindow());
	}*/

	ui.vtkOpenGLWidget->update();
}


void CloudPointDisplayQt::onOpen()
{
	QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".",
		"Open PCD files(*.pcd)");
	if (!fileName.isEmpty()){
		std::string file_name = fileName.toStdString();

		//sensor_msgs::PointCloud2 cloud2;

		pcl::PCLPointCloud2 cloud2;

		//pcl::PointCloud<Eigen::MatrixXf> cloud2;

		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		int pcd_version;
		int data_type;
		unsigned int data_idx;
		int offset = 0;
		pcl::PCDReader rd;
		rd.readHeader(file_name, cloud2, origin, orientation, pcd_version,
			data_type, data_idx);
		if (data_type == 0){
			pcl::io::loadPCDFile(fileName.toStdString(), *m_cloud);
		}
		else if (data_type == 2){
			pcl::PCDReader reader;
			reader.read<pcl::PointXYZRGB>(fileName.toStdString(), *m_cloud);
		}
		//m_viewer->addPointCloud(m_cloud, "cloud");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(m_cloud,
			232,232,232);
		m_viewer->updatePointCloud(m_cloud, single_color, "cloud");

		m_viewer->resetCamera();
		//ui.vtkOpenGLWidget->update();
		m_renWnd->Render();
	}
}


void CloudPointDisplayQt::onDelete()
{
	bool state = false;
	//state = m_viewer->removePointCloud("cloud");

	m_cloud->clear();
	m_viewer->updatePointCloud(m_cloud, "cloud");


	m_viewer->resetCamera();
	m_renWnd->Render();

}