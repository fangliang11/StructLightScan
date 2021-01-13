#include "CloudPointDisplayQt.h"

CloudPointDisplayQt::CloudPointDisplayQt(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

	initialVtkWidget();

	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));	
}



void CloudPointDisplayQt::initialVtkWidget()
{
	//����vtk��Ⱦ������ƺ���Ⱦ����
	m_ren = vtkSmartPointer<vtkRenderer>::New();
	m_renWnd = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	m_renWnd->AddRenderer(m_ren);

	m_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	//��pcl���ӻ�����vtk��Ⱦ����
	m_viewer.reset(new pcl::visualization::PCLVisualizer(m_ren, m_renWnd, "viewer", false));
	m_viewer->addPointCloud(m_cloud, "cloud");

	//����Ⱦ���ڣ����¼�����
	ui.vtkOpenGLWidget->SetRenderWindow(m_viewer->getRenderWindow());
	m_viewer->setupInteractor(ui.vtkOpenGLWidget->GetInteractor(), ui.vtkOpenGLWidget->GetRenderWindow());

	ui.vtkOpenGLWidget->update();
}


void CloudPointDisplayQt::onOpen()
{
	QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.pcd)");
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
		rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);
		if (data_type == 0){
			pcl::io::loadPCDFile(fileName.toStdString(), *m_cloud);
		}
		else if (data_type == 2){
			pcl::PCDReader reader;
			reader.read<pcl::PointXYZ>(fileName.toStdString(), *m_cloud);
		}
		m_viewer->updatePointCloud(m_cloud, "cloud");
		m_viewer->addCoordinateSystem();
		m_viewer->setBackgroundColor(0, 0, 0);
		m_viewer->resetCamera();
		//ui.vtkOpenGLWidget->update();
		m_renWnd->Render();
	}
}