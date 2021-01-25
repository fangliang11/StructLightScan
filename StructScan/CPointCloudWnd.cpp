
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
	//创建vtk渲染对象控制和渲染窗口
	m_ren = vtkSmartPointer<vtkRenderer>::New();
	m_renWnd = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	m_renWnd->AddRenderer(m_ren);
	m_iren = vtkSmartPointer< vtkRenderWindowInteractor>::New();
	m_iren->SetRenderWindow(m_renWnd);
	//m_vtkEventConnection = vtkSmartPointer<vtkEventQtSlotConnect>::New();

	m_monoCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	m_colorCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	// 绑定pcl可视化对象到 VTK 渲染窗口
	m_viewer.reset(new pcl::visualization::PCLVisualizer(m_ren, m_renWnd, "CloudPoint", false));
	m_viewer->setupInteractor(m_iren, m_renWnd);
	//pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB> color_cloud;
	//m_viewer->addPointCloud(m_colorCloud, "cloud");
	//m_viewer->addPointCloud(m_monoCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(m_monoCloud, 0, 255, 255), "cloud");
	//m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");

	//绑定vtk渲染窗口至 ui 控件
	ui->SetRenderWindow(m_renWnd);
	ui->update();
	//启动线程
	if (!workThread) {
		workThread = new std::thread(&CPointCloudWnd::WorkingOnPointCloud, this);
		loopFlag = true;
		//if (workThread->joinable())
		//	workThread->detach();
	}
}

// 线程函数
void CPointCloudWnd::WorkingOnPointCloud()
{
	qDebug() << " enter WorkingThread";

	while (loopFlag) {
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
		//所有耗时操作都至于此循环
		if (ResponseSignals(m_actionCode)) {
			Draw();
		}
	}
}


void CPointCloudWnd::Draw()
{
	emit signalUpdateCloudWnd();
}

//工作线程：槽函数响应
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
			//判断是否带有颜色数据
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
		//判断是否带有颜色数据
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
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//设置法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree->setInputCloud(m_monoCloud);//用cloud构造tree对象
	n.setInputCloud(m_monoCloud);//为法线估计对象设置输入点云
	n.setSearchMethod(tree);//设置搜索方法
	n.setKSearch(20);//设置k邻域搜素的搜索范围
	n.compute(*normals);//估计法线
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);//
	pcl::concatenateFields(*m_monoCloud, *normals, *cloud_with_normals);//连接字段，cloud_with_normals存储有向点云
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
	tree2->setInputCloud(cloud_with_normals);//利用有向点云构造tree

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
	pcl::PolygonMesh triangles;//存储最终三角化的网络模型

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);         //设置搜索半径radius，来确定三角化时k一邻近的球半径。

	// Set typical values for the parameters
	gp3.setMu(2.5);                     //设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);//设置样本点最多可以搜索的邻域数目100 。
	gp3.setMaximumSurfaceAngle(M_PI / 4);  //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
	gp3.setMinimumAngle(M_PI / 18);        //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
	gp3.setMaximumAngle(2 * M_PI / 3);       //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
	gp3.setNormalConsistency(false);     //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。

	// Get result
	gp3.setInputCloud(cloud_with_normals);//设置输入点云为有向点云
	gp3.setSearchMethod(tree2);           //设置搜索方式tree2
	gp3.reconstruct(triangles);           //重建提取三角化
   // std::cout << triangles;
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();//获得重建后每点的 ID, Parts 从 0 开始编号， a-1 表示未连接的点。
	/*
	获得重建后每点的状态，取值为 FREE 、 FRINGE 、 BOUNDARY 、 COMPLETED 、 NONE 常量，
	其中 NONE 表示未定义，
	FREE 表示该点没有在 三 角化后的拓扑内，为自由点，
	COMPLETED 表示该点在三角化后的拓扑内，并且邻域都有拓扑点，
	BOUNDARY 表示该点在三角化后的拓扑边缘，
	FRINGE 表示该点在 三 角化后的拓扑内，其连接会产生重叠边。
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

// 鼠标事件
void CPointCloudWnd::onVtkOpenGLNativeWidgetMouseEvent(QMouseEvent *event)
{
	// 鼠标键
	if (event->button() == Qt::LeftButton) {
		// 鼠标事件类型（鼠标按下、释放、移动、双击等）
		if (event->type() == QEvent::MouseButtonRelease) {
			// PointerPicker
			auto picker = m_renWnd->GetInteractor()->GetPicker();

			// 获取像素坐标
			int *tmp = m_renWnd->GetInteractor()->GetEventPosition();
			std::vector<int> pixel_point{ tmp[0], tmp[1] };
			qDebug() << "picking screen coordinate: " << pixel_point[0] << ", " << pixel_point[1];

			// 获取当前事件发生的renderer， 有可能为nullptr
			vtkRenderer* renderer = m_renWnd->GetInteractor()->FindPokedRenderer(pixel_point[0], pixel_point[1]);

			// 获取VTK世界坐标
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


