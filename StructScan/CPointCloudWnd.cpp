
#include "CPointCloudWnd.h"

CPointCloudWnd::CPointCloudWnd(QVTKOpenGLNativeWidget *wnd, QWidget *parent)
	: ui(wnd), QWidget(parent), workThread(nullptr)
	, loopFlag(false), m_actionCode(0)
{

	initialVtkWidget();

	connect(ui, &QVTKOpenGLNativeWidget::mouseEvent, this, &CPointCloudWnd::onVtkOpenGLNativeWidgetMouseEvent);
	connect(this, &CPointCloudWnd::signalUpdateCloudWnd, this, &CPointCloudWnd::onUpdateCloudWnd);
	connect(this, &CPointCloudWnd::signalOpenPCL, this, &CPointCloudWnd::onOpenPCL);
	connect(this, &CPointCloudWnd::signalSelect, this, &CPointCloudWnd::onSelect);
	connect(this, &CPointCloudWnd::signalDelete, this, &CPointCloudWnd::onDelete);
	connect(this, &CPointCloudWnd::signalAdd, this, &CPointCloudWnd::onAdd);
	connect(this, &CPointCloudWnd::signalClear, this, &CPointCloudWnd::onClear);
	connect(this, &CPointCloudWnd::signalSurfaceRebuild, this, &CPointCloudWnd::onSurfaceRebuild);
}

CPointCloudWnd::~CPointCloudWnd()
{

}


void CPointCloudWnd::DestroyThisWnd()
{
	loopFlag = false;
}


void CPointCloudWnd::initialVtkWidget()
{
	//����vtk��Ⱦ������ƺ���Ⱦ����
	m_ren = vtkSmartPointer<vtkRenderer>::New();
	m_renWnd = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	m_renWnd->AddRenderer(m_ren);
	m_iren = vtkSmartPointer< vtkRenderWindowInteractor>::New();
	m_iren->SetRenderWindow(m_renWnd);
	//m_vtkEventConnection = vtkSmartPointer<vtkEventQtSlotConnect>::New();

	m_monoCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	m_colorCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	// ��pcl���ӻ����� VTK ��Ⱦ����
	m_viewer.reset(new pcl::visualization::PCLVisualizer(m_ren, m_renWnd, "CloudPoint", false));
	m_viewer->setupInteractor(m_iren, m_renWnd);
	//pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB> color_cloud;
	//m_viewer->addPointCloud(m_colorCloud, "cloud");
	//m_viewer->addPointCloud(m_monoCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(m_monoCloud, 0, 255, 255), "cloud");
	//m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");

	//��vtk��Ⱦ������ ui �ؼ�
	ui->SetRenderWindow(m_renWnd);
	ui->update();
	//�����߳�
	if (!workThread) {
		workThread = new std::thread(&CPointCloudWnd::WorkingOnPointCloud, this);
		loopFlag = true;
		//if (workThread->joinable())
		//	workThread->detach();
	}
}

// �̺߳���
void CPointCloudWnd::WorkingOnPointCloud()
{
	qDebug() << " enter WorkingThread";

	while (loopFlag) {
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
		//���к�ʱ���������ڴ�ѭ��
		if (ResponseSignals(m_actionCode)) {
			Draw();
		}
	}
}


void CPointCloudWnd::Draw()
{
	emit signalUpdateCloudWnd();
}

//�����̣߳��ۺ�����Ӧ
bool CPointCloudWnd::ResponseSignals(int code)
{
	bool state = false;
	switch (code)
	{
	case ACTION_NONE:
		state = false;
		break;
	case ACTION_OPEN:
		//displayOnVTK1();
		displayPCDfile();
		//displaySphere();
		state = true;
		break;
	case ACTION_SELECT:
		break;
	case ACTION_DELETE:
		break;
	case ACTION_ADD:
		break;
	case ACTION_CLEAR:
		break;
	case ACTION_REBUILD:
		RebuildTest();
		state = true;
		break;
	case 7:
		break;
	default:
		break;
	}
	m_actionCode = ACTION_NONE;

	return state;
}


void CPointCloudWnd::displaySelectPCD()
{
	QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.pcd)");
	if (!fileName.isEmpty()) {
		std::string file_name = fileName.toStdString();

		m_monoCloud->clear();
		m_colorCloud->clear();
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
		if (data_type == 0) {
			//�ж��Ƿ������ɫ����
			if (cloud2.fields.size() > 3)
				pcl::io::loadPCDFile(file_name, *m_colorCloud);
			else
				pcl::io::loadPCDFile(file_name, *m_monoCloud);
		}
		else if (data_type == 2) {
			pcl::PCDReader reader;
			if (cloud2.fields.size() > 3)
				reader.read<pcl::PointXYZRGBA>(file_name, *m_colorCloud);
			else
				reader.read<pcl::PointXYZ>(file_name, *m_monoCloud);
		}
		if (cloud2.fields.size() > 3) {
			m_viewer->addPointCloud(m_colorCloud, "cloud");
			m_viewer->updatePointCloud(m_colorCloud, "cloud");
		}
		else {
			m_viewer->addPointCloud(m_monoCloud, "cloud");
			m_viewer->updatePointCloud(m_monoCloud, "cloud");
		}
		m_viewer->addCoordinateSystem(1);
		m_viewer->addSphere(pcl::PointXYZ(0,0,0), 0.1, 0.5, 0.3, 0.0, "sphere");

		m_viewer->setBackgroundColor(0, 0, 0);
		//m_viewer->setShowFPS(false);

		m_viewer->resetCamera();
		//m_viewer->resetCameraViewpoint();
		m_renWnd->Render();
	}
}


void CPointCloudWnd::displaySphere()
{
	// Setup sphere
	vtkSmartPointer<vtkSphereSource> sphereSource =	vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->Update();
	vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
	vtkSmartPointer<vtkActor> sphereActor =	vtkSmartPointer<vtkActor>::New();
	sphereActor->SetMapper(sphereMapper);
	sphereActor->SetScale(0.2);

	m_ren->AddActor(sphereActor);
	//m_ren->ResetCamera();
	//m_renWnd->Render();
}


void CPointCloudWnd::displayPCDfile()
{
	std::string file_name = "table_scene_lms400_downsampled.pcd";

	m_monoCloud->clear();
	m_colorCloud->clear();
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
	if (data_type == 0) {
		//�ж��Ƿ������ɫ����
		if (cloud2.fields.size() > 3)
			pcl::io::loadPCDFile(file_name, *m_colorCloud);
		else
			pcl::io::loadPCDFile(file_name, *m_monoCloud);
	}
	else if (data_type == 2) {
		pcl::PCDReader reader;
		if (cloud2.fields.size() > 3)
			reader.read<pcl::PointXYZRGBA>(file_name, *m_colorCloud);
		else
			reader.read<pcl::PointXYZ>(file_name, *m_monoCloud);
	}
	if (cloud2.fields.size() > 3) {
		m_viewer->addPointCloud(m_colorCloud, "cloud");
		m_viewer->updatePointCloud(m_colorCloud, "cloud");
	}
	else {
		m_viewer->addPointCloud(m_monoCloud, "cloud");
		m_viewer->updatePointCloud(m_monoCloud, "cloud");
	}
	m_viewer->addCoordinateSystem(1);
	m_viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.1, 0.5, 0.3, 0.0, "sphere");

	m_viewer->setBackgroundColor(0, 0, 0);
	//m_viewer->setShowFPS(false);

	//m_viewer->resetCamera();
	//m_viewer->resetCameraViewpoint();
	//m_renWnd->Render();

}


void CPointCloudWnd::RebuildTest()
{
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���÷��߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree->setInputCloud(m_monoCloud);//��cloud����tree����
	n.setInputCloud(m_monoCloud);//Ϊ���߹��ƶ��������������
	n.setSearchMethod(tree);//������������
	n.setKSearch(20);//����k�������ص�������Χ
	n.compute(*normals);//���Ʒ���
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);//
	pcl::concatenateFields(*m_monoCloud, *normals, *cloud_with_normals);//�����ֶΣ�cloud_with_normals�洢�������
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//��������������
	tree2->setInputCloud(cloud_with_normals);//����������ƹ���tree

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//�������ǻ�����
	pcl::PolygonMesh triangles;//�洢�������ǻ�������ģ��

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);         //���������뾶radius����ȷ�����ǻ�ʱkһ�ڽ�����뾶��

	// Set typical values for the parameters
	gp3.setMu(2.5);                     //���������㵽����������ĳ˻�ϵ�� mu �����ÿ�������������������룬����ʹ���㷨����Ӧ�����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100);//����������������������������Ŀ100 ��
	gp3.setMaximumSurfaceAngle(M_PI / 4);  //45 degrees����������ʱ�����Ƕ� eps_angle ����ĳ�㷨������ڲ�����ķ���ƫ��Ƕȳ��������Ƕ�ʱ������ʱ�Ͳ����Ǹõ㡣
	gp3.setMinimumAngle(M_PI / 18);        //10 degrees���������ǻ��������ε���С�ǣ����� minimum_angle Ϊ��С�ǵ�ֵ��
	gp3.setMaximumAngle(2 * M_PI / 3);       //120 degrees���������ǻ��������ε����ǣ����� maximum_angle Ϊ���ǵ�ֵ��
	gp3.setNormalConsistency(false);     //����һ����־ consistent ������֤���߳���һ�£��������Ϊ true ���ʹ���㷨���ַ��߷���һ�£����Ϊ false �㷨�򲻻���з���һ���Լ�顣

	// Get result
	gp3.setInputCloud(cloud_with_normals);//�����������Ϊ�������
	gp3.setSearchMethod(tree2);           //����������ʽtree2
	gp3.reconstruct(triangles);           //�ؽ���ȡ���ǻ�
   // std::cout << triangles;
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();//����ؽ���ÿ��� ID, Parts �� 0 ��ʼ��ţ� a-1 ��ʾδ���ӵĵ㡣
	/*
	����ؽ���ÿ���״̬��ȡֵΪ FREE �� FRINGE �� BOUNDARY �� COMPLETED �� NONE ������
	���� NONE ��ʾδ���壬
	FREE ��ʾ�õ�û���� �� �ǻ���������ڣ�Ϊ���ɵ㣬
	COMPLETED ��ʾ�õ������ǻ���������ڣ��������������˵㣬
	BOUNDARY ��ʾ�õ������ǻ�������˱�Ե��
	FRINGE ��ʾ�õ��� �� �ǻ���������ڣ������ӻ�����ص��ߡ�
	*/
	std::vector<int> states = gp3.getPointStates();

	m_viewer->addPolygonMesh(triangles, "mesh");
	m_viewer->addCoordinateSystem(1.0);
	//m_viewer->initCameraParameters();
	//m_viewer->resetCamera();
	//m_viewer->resetCameraViewpoint();
	//m_viewer->setPosition(100, 100);
	//m_viewer->spin();      //Calls the interactor and runs an internal loop
	m_viewer->updatePolygonMesh(triangles, "mesh");
	//m_renWnd->Render();

}


void CPointCloudWnd::filteredCloud()
{


}
//------------------------------------------------- SLOT -------------------------------//
void CPointCloudWnd::onUpdateCloudWnd()
{
	m_viewer->resetCamera();
	//m_viewer->resetCameraViewpoint();
	m_renWnd->Render();

}

// ����¼�
void CPointCloudWnd::onVtkOpenGLNativeWidgetMouseEvent(QMouseEvent *event)
{
	// ����
	if (event->button() == Qt::LeftButton) {
		// ����¼����ͣ���갴�¡��ͷš��ƶ���˫���ȣ�
		if (event->type() == QEvent::MouseButtonRelease) {
			// PointerPicker
			auto picker = m_renWnd->GetInteractor()->GetPicker();

			// ��ȡ��������
			int *tmp = m_renWnd->GetInteractor()->GetEventPosition();
			std::vector<int> pixel_point{ tmp[0], tmp[1] };
			qDebug() << "picking screen coordinate: " << pixel_point[0] << ", " << pixel_point[1];

			// ��ȡ��ǰ�¼�������renderer�� �п���Ϊnullptr
			vtkRenderer* renderer = m_renWnd->GetInteractor()->FindPokedRenderer(pixel_point[0], pixel_point[1]);

			// ��ȡVTK��������
			picker->Pick(tmp[0], tmp[1], 0, renderer);
			// picker->Pick(tmp[0], tmp[1], 0, m_renderWindow->GetRenderers()->GetFirstRenderer());
			double picked[3];
			picker->GetPickPosition(picked);
			qDebug() << "picked VTKrenderer coordinate: " << picked[0] << picked[1] << picked[2];

		}

		if (event->type() == QEvent::MouseButtonDblClick) {
			qDebug() << "mouse double clicked";
		}
	}
}


void CPointCloudWnd::onOpenPCL()
{
	m_actionCode = ACTION_OPEN;
}


void CPointCloudWnd::onSelect()
{
	m_actionCode = ACTION_SELECT;
}


void CPointCloudWnd::onDelete()
{
	m_actionCode = ACTION_DELETE;
}


void CPointCloudWnd::onAdd()
{
	m_actionCode = ACTION_ADD;
}


void CPointCloudWnd::onClear()
{
	m_actionCode = ACTION_CLEAR;
}


void CPointCloudWnd::onSurfaceRebuild()
{
	m_actionCode = ACTION_REBUILD;
}


