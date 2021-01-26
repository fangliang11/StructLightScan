
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
	connect(this, &CPointCloudWnd::signalFilter, this, &CPointCloudWnd::onFilter);
	connect(this, &CPointCloudWnd::signalMesh, this, &CPointCloudWnd::onMesh);
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
	vtkSmartPointer<vtkLight> light = vtkSmartPointer<vtkLight>::New();
	light->SetColor(1, 1, 1);
	//light->SetPosition(100, 100, 100);
	//light->SetFocalPoint(m_ren->GetActiveCamera()->GetFocalPoint());
	m_ren->AddLight(light);
	vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
	camera->SetPosition(20, 20, 20);
	m_ren->SetActiveCamera(camera);
	m_renWnd = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	m_renWnd->AddRenderer(m_ren);
	m_iren = vtkSmartPointer< vtkRenderWindowInteractor>::New();
	m_iren->SetRenderWindow(m_renWnd);
	//m_vtkEventConnection = vtkSmartPointer<vtkEventQtSlotConnect>::New();

	m_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	// 绑定pcl可视化对象到 VTK 渲染窗口
	m_viewer.reset(new pcl::visualization::PCLVisualizer(m_ren, m_renWnd, "CloudPoint", false));
	m_viewer->setupInteractor(m_iren, m_renWnd);
	m_viewer->setBackgroundColor(0.01, 0.5, 0.6);
	m_viewer->addCoordinateSystem(1);
	m_viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.05, 0.3, 0.3, 0.0, "sphere");
	m_viewer->addPointCloud(m_cloud, "cloud");
	m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "cloud");


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
		displayPCDfile(m_pcdPath);
		state = true;
		break;
	case ACTION_SELECT:
		break;
	case ACTION_DELETE:
		deleteCloud();
		state = true;
		break;
	case ACTION_ADD:
		break;
	case ACTION_FILTER:
		filteredCloud(1);
		state = true;
		break;
	case ACTION_MESH:
		buildMesh();
		state = true;
		break;
	case ACTION_REBUILD:
		RebuildTest();
		state = true;
		break;
	default:
		state = false;
		break;
	}
	m_actionCode = ACTION_NONE;

	return state;
}


void CPointCloudWnd::displaySelectPCD()
{
	QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".",
		"Open PCD files(*.pcd)");
	if (!fileName.isEmpty()) {
		std::string file_name = fileName.toStdString();

		m_cloud->clear();
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
		if (data_type == 0)
			pcl::io::loadPCDFile(file_name, *m_cloud);
		else if (data_type == 2) {
			pcl::PCDReader reader;
			reader.read<pcl::PointXYZ>(file_name, *m_cloud);
		}

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(m_cloud,
			232, 232, 232);
		m_viewer->updatePointCloud(m_cloud, single_color, "cloud");

		////判断是否带有颜色数据
		//if (cloud2.fields.size() > 3)
		//	m_viewer->updatePointCloud(m_cloud, "cloud");
		//else {
		//}
		//m_viewer->addCoordinateSystem(1);
		//m_viewer->addSphere(pcl::PointXYZ(0,0,0), 0.1, 0.3, 0.3, 0.0, "sphere");

		//m_viewer->setBackgroundColor(0, 0, 0);
		//m_viewer->setShowFPS(false);

		//m_viewer->resetCamera();
		//m_viewer->resetCameraViewpoint();
		//m_renWnd->Render();
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


void CPointCloudWnd::displayPCDfile(std::string file_name)
{
	//std::string file_name = "table_scene_lms400_downsampled.pcd";
	//std::string file_name = "../TestFiles/rabbit.pcd";
	//std::string file_name = "./TestFiles/rabbit.pcd";

	m_cloud->clear();
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
	if (data_type == 0)
		pcl::io::loadPCDFile(file_name, *m_cloud);
	else if (data_type == 2) {
		pcl::PCDReader reader;
		reader.read<pcl::PointXYZ>(file_name, *m_cloud);
	}

	// 按照z字段进行深度渲染，不同深度不同颜色
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(m_cloud, "z");
	m_viewer->updatePointCloud(m_cloud, fildColor, "cloud");
}


void CPointCloudWnd::deleteCloud()
{
	// 删除当前窗口中的点云对象, 会自动调用窗口刷新
	
	//m_viewer->removePointCloud("cloud");
	m_cloud->clear();

	m_viewer->updatePointCloud(m_cloud, "cloud");

}


void CPointCloudWnd::RebuildTest()
{
	//// Normal estimation*
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//设置法线估计对象
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	//tree->setInputCloud(m_cloud);//用cloud构造tree对象
	//n.setInputCloud(m_cloud);//为法线估计对象设置输入点云
	//n.setSearchMethod(tree);//设置搜索方法
	//n.setKSearch(20);//设置k邻域搜素的搜索范围
	//n.compute(*normals);//估计法线
	////* normals should not contain the point normals + surface curvatures

	//// Concatenate the XYZ and normal fields*
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);//
	//pcl::concatenateFields(*m_cloud, *normals, *cloud_with_normals);//连接字段，cloud_with_normals存储有向点云
	////* cloud_with_normals = cloud + normals

	//// Create search tree*
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
	//tree2->setInputCloud(cloud_with_normals);//利用有向点云构造tree

	//// Initialize objects
	//pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
	//pcl::PolygonMesh triangles;//存储最终三角化的网络模型

	//// Set the maximum distance between connected points (maximum edge length)
	//gp3.setSearchRadius(0.025);         //设置搜索半径radius，来确定三角化时k一邻近的球半径。

	//// Set typical values for the parameters
	//gp3.setMu(2.5);                     //设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
	//gp3.setMaximumNearestNeighbors(100);//设置样本点最多可以搜索的邻域数目100 。
	//gp3.setMaximumSurfaceAngle(M_PI / 4);  //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
	//gp3.setMinimumAngle(M_PI / 18);        //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
	//gp3.setMaximumAngle(2 * M_PI / 3);       //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
	//gp3.setNormalConsistency(false);     //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。

	//// Get result
	//gp3.setInputCloud(cloud_with_normals);//设置输入点云为有向点云
	//gp3.setSearchMethod(tree2);           //设置搜索方式tree2
	//gp3.reconstruct(triangles);           //重建提取三角化
 //  // std::cout << triangles;
	//// Additional vertex information
	//std::vector<int> parts = gp3.getPartIDs();//获得重建后每点的 ID, Parts 从 0 开始编号， a-1 表示未连接的点。
	///*
	//获得重建后每点的状态，取值为 FREE 、 FRINGE 、 BOUNDARY 、 COMPLETED 、 NONE 常量，
	//其中 NONE 表示未定义，
	//FREE 表示该点没有在 三 角化后的拓扑内，为自由点，
	//COMPLETED 表示该点在三角化后的拓扑内，并且邻域都有拓扑点，
	//BOUNDARY 表示该点在三角化后的拓扑边缘，
	//FRINGE 表示该点在 三 角化后的拓扑内，其连接会产生重叠边。
	//*/
	//std::vector<int> states = gp3.getPointStates();

	//m_viewer->addPolygonMesh(triangles, "mesh");
	//m_viewer->addCoordinateSystem(1.0);
	////m_viewer->initCameraParameters();
	////m_viewer->resetCamera();
	////m_viewer->resetCameraViewpoint();
	////m_viewer->setPosition(100, 100);
	////m_viewer->spin();      //Calls the interactor and runs an internal loop
	//m_viewer->updatePolygonMesh(triangles, "mesh");
	////m_renWnd->Render();

}


void CPointCloudWnd::filteredCloud(int filtercode)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());

	if (filtercode == 0) {
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  //创建滤波器
		outrem.setInputCloud(m_cloud);    //设置输入点云
		outrem.setRadiusSearch(0.8);     //设置半径为0.8的范围内找临近点
		outrem.setMinNeighborsInRadius(2); //设置查询点的邻域点集数小于2的删除
		outrem.filter(*filtered);     //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
	}
	else if (filtercode == 1) {
		//创建条件限定的下的滤波器
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new 
			pcl::ConditionAnd<pcl::PointXYZ>()); 
		//为条件定义对象添加比较算子 ,   //添加在Z字段上大于0的比较算子
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new	
			pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
		//添加在Z字段上小于0.8的比较算子
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
		  // 创建滤波器并用条件定义对象初始化
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setCondition(range_cond);
		condrem.setInputCloud(m_cloud);                   //输入点云
		condrem.setKeepOrganized(true);               //设置保持点云的结构
		// 执行滤波
		condrem.filter(*filtered);  //大于0.0小于0.8这两个条件用于建立滤波器
	}

	m_cloud->clear();
	m_viewer->updatePointCloud(m_cloud, "cloud");
	//点云复制
	pcl::copyPointCloud(*filtered, *m_cloud);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(m_cloud,
	//	232, 232, 232);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(m_cloud, "z");
	m_viewer->updatePointCloud(m_cloud, fildColor, "cloud");
}


void CPointCloudWnd::buildMesh()
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new 
		pcl::PointCloud<pcl::PointNormal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(m_cloud);
	n.setInputCloud(m_cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	pcl::concatenateFields(*m_cloud, *normals, *cloud_with_normals);
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh mesh; //存储最终三角化的网格模型
	gp3.setSearchRadius(2);//这个参数需要更改
	gp3.setMu(2.5);//设置样本点搜索其邻近点的最远距离为2.5 
	gp3.setMaximumNearestNeighbors(100);//设置样本点搜索的邻域个数为100
	gp3.setMaximumSurfaceAngle(M_PI / 3);//设置某点法线方向偏离样本点法线方向的最大角度为45度
	gp3.setMinimumAngle(M_PI / 180);//设置三角化后得到的三角形内角最小角度为10度
	gp3.setMaximumAngle(2 * M_PI / 3);
	gp3.setNormalConsistency(false);//设置该参数保证法线朝向一致
	gp3.setInputCloud(cloud_with_normals);//设置输入点云为有向点云
	gp3.setSearchMethod(tree2);//设置搜素方式为tree2
	gp3.reconstruct(mesh);//重建提取三角化
	
	m_cloud->clear();
	m_viewer->updatePointCloud(m_cloud, "cloud");
	m_viewer->addPolygonMesh(mesh, "W");
}

//------------------------------------------------- SLOT -------------------------------//

// 窗口刷新
void CPointCloudWnd::onUpdateCloudWnd()
{
	//m_viewer->resetCamera();
	//m_viewer->resetCameraViewpoint();

	//m_ren->GetActiveCamera()->SetPosition(20, 20, 20);
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
			//qDebug() << "picking screen coordinate: " << pixel_point[0] << ", " << pixel_point[1];

			// 获取当前事件发生的renderer， 有可能为nullptr
			vtkRenderer* renderer = m_renWnd->GetInteractor()->FindPokedRenderer(pixel_point[0], pixel_point[1]);

			// 获取VTK世界坐标
			picker->Pick(tmp[0], tmp[1], 0, renderer);
			// picker->Pick(tmp[0], tmp[1], 0, m_renderWindow->GetRenderers()->GetFirstRenderer());
			double picked[3];
			picker->GetPickPosition(picked);
			//qDebug() << "picked VTKrenderer coordinate: " << picked[0] << picked[1] << picked[2];

		}

		if (event->type() == QEvent::MouseButtonDblClick) {
			qDebug() << "mouse double clicked";
		}
	}
}


void CPointCloudWnd::onOpenPCL()
{
	QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".",
		"Open PCD files(*.pcd)");
	if (!fileName.isEmpty()) {
		m_pcdPath = fileName.toStdString();

		m_actionCode = ACTION_OPEN;
	}
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


void CPointCloudWnd::onFilter()
{
	m_actionCode = ACTION_FILTER;
}


void CPointCloudWnd::onMesh()
{
	m_actionCode = ACTION_MESH;
	qDebug() << "active mesh";
}


void CPointCloudWnd::onSurfaceRebuild()
{
	m_actionCode = ACTION_REBUILD;
}


