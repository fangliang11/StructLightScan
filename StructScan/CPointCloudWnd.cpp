
#include "CPointCloudWnd.h"


CPointCloudWnd::CPointCloudWnd(QVTKOpenGLNativeWidget *wnd, QWidget *parent)
	: ui(wnd), QWidget(parent), workThread(nullptr)
	, loopFlag(false), m_actionCode(0)
	, b_rubber_band_selection_mode(false)
{
	initialVtkWidget();

	m_bFilterEnable = false;
	m_nFilterMethod = 0;
	m_fFilterParam1 = 0.01f;
	m_fFilterParam2 = 0.01f;
	m_fFilterParam3 = 0.01f;;
	m_bremoveOutlierEnable = false;
	m_fremoveOutlierParam1 = 50.0f;
	m_fremoveOutlierParam2 = 1.0f;
	m_bsmoothEnable = false;
	m_fsmoothParam1 = 0.05f;
	m_fsmoothParam2 = 1.0f;


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


void CPointCloudWnd::UpdateThisWnd()
{
	emit signalUpdateCloudWnd();
}


void CPointCloudWnd::UpdatePCLViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn)
{
	assert(m_viewer);
	assert(m_displayCloud);

	m_displayCloud->clear();
	m_viewer->updatePointCloud(m_displayCloud, "cloud");
	pcl::copyPointCloud(*cloudIn, *m_displayCloud);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color(m_displayCloud, "z");
	m_viewer->updatePointCloud(m_displayCloud, color, "cloud");
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
	m_renCamera = vtkSmartPointer<vtkCamera>::New();
	m_renCamera->SetFocalPoint(0, 0, 0);
	m_ren->SetActiveCamera(m_renCamera);
	m_renWnd = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	m_renWnd->AddRenderer(m_ren);
	
	m_iren = vtkSmartPointer< vtkRenderWindowInteractor>::New();
	m_iren->SetRenderWindow(m_renWnd);
	//m_vtkEventConnection = vtkSmartPointer<vtkEventQtSlotConnect>::New();

	//  PCL
	m_sourceCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	m_displayCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	m_filteredCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	m_removeOutlieredCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	m_smoothedCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	// 绑定pcl可视化对象到 VTK 渲染窗口
	m_viewer.reset(new pcl::visualization::PCLVisualizer(m_ren, m_renWnd, "CloudPoint", false));
	m_viewer->setupInteractor(m_iren, m_renWnd);
	m_viewer->setBackgroundColor(0.01, 0.5, 0.6);
	//m_viewer->addCoordinateSystem(1);
	m_viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.05, 0.3, 0.3, 0.0, "sphere");
	m_viewer->addPointCloud(m_displayCloud, "cloud");
	m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "cloud");
	m_viewer->registerAreaPickingCallback(areaPickCallback, this);


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
		//所有耗时操作都置于此循环
		if (ResponseSignals(m_actionCode)) {
			UpdateThisWnd();
		}
	}
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
		//displaySphere();
		state = true;
		break;
	case ACTION_SELECT:
		userSelect();
		break;
	case ACTION_DELETE:
		clearDisplayCloud();
		state = true;
		break;
	case ACTION_ADD:
		//savePointCloudFile();
		UpdateThisWnd();
		state = true;
		break;
	case ACTION_FILTER: {
		pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud;
		if (m_bremoveOutlierEnable && !m_bsmoothEnable)
			inCloud = m_removeOutlieredCloud;
		else if (!m_bremoveOutlierEnable && m_bsmoothEnable)
			inCloud = m_smoothedCloud;
		else if (m_bremoveOutlierEnable && m_bsmoothEnable)
			inCloud = m_displayCloud;
		else
			inCloud = m_sourceCloud;
		filteredCloud(m_nFilterMethod, &inCloud, &m_filteredCloud, m_fFilterParam1, m_fFilterParam2, m_fFilterParam3);
		UpdatePCLViewer(m_filteredCloud);
		state = true;
		break;
	}
	case ACTION_REMOVE_OUTLIER: {
		pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud;
		if (m_bFilterEnable && !m_bsmoothEnable)
			inCloud = m_filteredCloud;
		else if (!m_bFilterEnable && m_bsmoothEnable)
			inCloud = m_smoothedCloud;
		else if (m_bFilterEnable && m_bsmoothEnable)
			inCloud = m_displayCloud;
		else
			inCloud = m_sourceCloud;
		removeOutlierPoints(&inCloud, &m_removeOutlieredCloud, m_fremoveOutlierParam1, m_fremoveOutlierParam2);
		UpdatePCLViewer(m_removeOutlieredCloud);
		state = true;
		break;
	}
	case ACTION_SMOOTH: {
		pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud;
		if (m_bFilterEnable && !m_bremoveOutlierEnable)
			inCloud = m_filteredCloud;
		else if (!m_bFilterEnable && m_bremoveOutlierEnable)
			inCloud = m_removeOutlieredCloud;
		else if (m_bFilterEnable && m_bremoveOutlierEnable)
			inCloud = m_displayCloud;
		else
			inCloud = m_sourceCloud;
		smoothCloudPoints(&inCloud, &m_smoothedCloud, m_fsmoothParam1, m_fsmoothParam2);
		UpdatePCLViewer(m_smoothedCloud);
		state = true;
		break;
	}
	case ACTION_MESH:
		buildMesh(m_sourceCloud);
		//SmoothPointcloud();
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
	//UpdataThisWnd();

	return state;
}


void CPointCloudWnd::AddCoordinateSystem()
{
	// 屏幕坐标系 > 世界坐标系
	double dispCoord[2] = { 10, 30 };
	vtkSmartPointer<vtkCoordinate> pCoorPress = vtkSmartPointer<vtkCoordinate>::New();
	pCoorPress->SetCoordinateSystemToDisplay();
	pCoorPress->SetValue(dispCoord);
	double *worldCoord = pCoorPress->GetComputedWorldValue(m_ren);
	qDebug() << worldCoord[0] << worldCoord[1] << worldCoord[2];

	// 添加坐标系
	vtkSmartPointer<vtkAxesActor> actor2 = vtkSmartPointer<vtkAxesActor>::New();
	actor2->SetPosition(worldCoord[0], worldCoord[1], worldCoord[2]);
	actor2->SetTotalLength(0.5,0.5,0.5);
	actor2->SetShaftType(0);
	actor2->SetAxisLabels(0);
	actor2->SetCylinderRadius(0.02);
	m_ren->AddActor(actor2);
}


void CPointCloudWnd::savePointCloudFile()
{
	std::string filename("test_save.pcd");
	pcl::PCDWriter writer;
	if(!m_displayCloud->empty())
		writer.write(filename, *m_displayCloud);
}


void CPointCloudWnd::saveMeshFile()
{
	std::string filename("test_save_mesh.ply");

	//pcl::io::savePLYFile(filename, mesh);

}


void CPointCloudWnd::displaySelectPCD()
{
	QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".",
		"Open PCD files(*.pcd)");
	if (!fileName.isEmpty()) {
		std::string file_name = fileName.toStdString();

		m_sourceCloud->clear();
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
			pcl::io::loadPCDFile(file_name, *m_sourceCloud);
		else if (data_type == 2) {
			pcl::PCDReader reader;
			reader.read<pcl::PointXYZ>(file_name, *m_sourceCloud);
		}

		pcl::copyPointCloud(*m_sourceCloud, *m_displayCloud);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(m_displayCloud,
			232, 232, 232);
		m_viewer->updatePointCloud(m_displayCloud, single_color, "cloud");

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


void CPointCloudWnd::displaySphereVTK()
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

	m_sourceCloud->clear();
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
		pcl::io::loadPCDFile(file_name, *m_sourceCloud);
	else if (data_type == 2) {
		pcl::PCDReader reader;
		reader.read<pcl::PointXYZ>(file_name, *m_sourceCloud);
	}
	
	pcl::copyPointCloud(*m_sourceCloud, *m_displayCloud);
	// 按照z字段进行深度渲染，不同深度不同颜色
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> filledColor(m_displayCloud, "z");
	m_viewer->updatePointCloud(m_displayCloud, filledColor, "cloud");

}


void CPointCloudWnd::clearDisplayCloud()
{
	// 删除当前窗口中的点云对象, 会自动调用窗口刷新
	
	//m_viewer->removePointCloud("cloud");
	m_displayCloud->clear();

	m_viewer->updatePointCloud(m_displayCloud, "cloud");
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

// 点云滤波
void CPointCloudWnd::filteredCloud(int method,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudIn,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudOut,
	float param1, float param2, float param3)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
	qDebug() << "before filtered: size " << (*cloudIn)->size();

	if (method == 0) {
		pcl::VoxelGrid<pcl::PointXYZ> sor;       //体素化网格滤波，实现下采样
		sor.setInputCloud(*cloudIn);
		sor.setLeafSize(param1, param1, param1);    //设置体素大小:单位（米）
		sor.filter(**cloudOut);
	}
	else if (method == 1) {
		pcl::UniformSampling<pcl::PointXYZ> unfm;   // 均匀采样滤波
		unfm.setInputCloud(*cloudIn);
		unfm.setRadiusSearch(param1);                //设置搜索半径
		unfm.filter(**cloudOut);
	}
	else if (method == 2) {
		// 增采样
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;  // 定义最小二乘实现的对象mls
		mls.setInputCloud(*cloudIn);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
		mls.setSearchMethod(kdtree);
		mls.setSearchRadius(param1);                                  // 设置 k临近 半径
		// Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
		mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
		mls.setUpsamplingRadius(param2);
		mls.setUpsamplingStepSize(param3);
		mls.setPolynomialFit(false);                                 // 设置为false可以 加速 smooth
		mls.process(**cloudOut);
	}
	else if (method == 3) {
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downSampled(new
		//	pcl::PointCloud<pcl::PointXYZ>());
		//pcl::VoxelGrid<pcl::PointXYZ> downSampled;  //下采样滤波器
		//downSampled.setInputCloud(m_cloud);            //设置需要过滤的点云给滤波对象
		//downSampled.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
		//downSampled.filter(*cloud_downSampled);           //执行滤波处理，存储输出

		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed(new
		//	pcl::PointCloud<pcl::PointXYZ>());
		//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //离群剔除滤波器
		//sor.setInputCloud(cloud_downSampled);                //设置待滤波的点云
		//sor.setMeanK(50);            //设置在进行统计时考虑的临近点个数
		//sor.setStddevMulThresh(1.0); //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
		//sor.filter(*cloud_removed);  //滤波结果存储到cloud_filtered

		//pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new
		//	pcl::search::KdTree<pcl::PointXYZ>); // mls平滑点云
		//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;  // 定义最小二乘实现的对象mls
		//mls.setSearchMethod(treeSampling);    // 设置KD-Tree作为搜索方法
		//mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
		//mls.setInputCloud(cloud_removed);        //设置待处理点云
		//mls.setPolynomialOrder(2);             // 拟合2阶多项式拟合
		//mls.setPolynomialFit(false);  // 设置为false可以 加速 smooth
		//mls.setSearchRadius(0.05); // 单位m.设置用于拟合的K近邻半径
		//mls.process(*filtered);        //输出
	}

	qDebug() << "success filtered, size: " << (*cloudOut)->size();

	//m_sourceCloud->clear();
	//m_viewer->updatePointCloud(m_sourceCloud, "cloud");
	//pcl::copyPointCloud(*filtered, *m_sourceCloud);
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(m_sourceCloud, "z");
	//m_viewer->updatePointCloud(m_sourceCloud, fildColor, "cloud");
}

// 移除离群点
void CPointCloudWnd::removeOutlierPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudIn,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudOut,
	float param1, float param2)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //离群剔除滤波器
	sor.setInputCloud(*cloudIn);                //设置待滤波的点云
	sor.setMeanK((int)param1);            //设置在进行统计时考虑的临近点个数
	sor.setStddevMulThresh((double)param2); //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
	sor.filter(**cloudOut);  //滤波结果存储到cloud_filtered
}

// 点云平滑:最小二乘法
void CPointCloudWnd::smoothCloudPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudIn,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudOut,
	float param1, float param2)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new
		pcl::search::KdTree<pcl::PointXYZ>); // mls平滑点云
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;  // 定义最小二乘实现的对象mls
	mls.setSearchMethod(treeSampling);    // 设置KD-Tree作为搜索方法
	mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
	mls.setInputCloud(*cloudIn);        //设置待处理点云
	mls.setPolynomialOrder(2);             // 拟合2阶多项式拟合
	mls.setPolynomialFit(false);  // 设置为false可以 加速 smooth
	mls.setSearchRadius((double)param1); // 单位m.设置用于拟合的K近邻半径
	mls.process(**cloudOut);        //输出
}

// 网格化：贪心三角化算法
void CPointCloudWnd::buildMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new 
		pcl::PointCloud<pcl::PointNormal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);// 定义输出的点云法线
	// 创建用于最近邻搜索的KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	// K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
	n.setKSearch(20);   // 使用当前点周围最近的20个点
	//n.setRadiusSearch(0.05);//对于每一个点都用半径为5cm的近邻搜索方式
	n.compute(*normals);
	// 将点云位姿、颜色、法线信息连接到一起
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//定义搜索树对象
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	// 定义三角化对象
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//pcl::PolygonMesh mesh; //存储最终三角化的网格模型
	//设置搜索时的半径，也就是KNN的球半径, 这个参数需要更改
	gp3.setSearchRadius(2);
	//设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	gp3.setMu(2.5);
	//设置样本点最多可搜索的邻域个数，典型值是50-100
	gp3.setMaximumNearestNeighbors(100);
	// 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	// 设置三角化后得到的三角形内角的最小的角度为10°
	gp3.setMinimumAngle(M_PI / 18);
	// 设置三角化后得到的三角形内角的最大角度为120°
	gp3.setMaximumAngle(2 * M_PI / 3);
	//设置该参数保证法线朝向一致
	gp3.setNormalConsistency(false);
	//设置输入点云为有向点云
	gp3.setInputCloud(cloud_with_normals);
	//设置搜素方式为tree2
	gp3.setSearchMethod(tree2);
	//重建提取三角化
	gp3.reconstruct(m_mesh);
	
	cloud->clear();
	m_viewer->updatePointCloud(m_sourceCloud, "cloud");
	m_viewer->addPolygonMesh(m_mesh, "mesh");
	//设置网格模型显示模式
	//viewer->setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示  
	//viewer->setRepresentationToPointsForAllActors(); //网格模型以点形式显示  
	//m_viewer->setRepresentationToWireframeForAllActors();  //网格模型以线框图模式显示

	pcl::io::savePLYFile("test_save_mesh.ply", m_mesh);
}

// 框选删除点云: 激活 rubber_band_selection_mode 
void CPointCloudWnd::userSelect()
{
	HKL	hcurkl;
	hcurkl = GetKeyboardLayout(0);
	//这里会切换为美式键盘的英文输入法
	LoadKeyboardLayout(L"0x0409", KLF_ACTIVATE);

	::keybd_event(0x58, 0, 0, 0);
	Sleep(20);
	::keybd_event(0x58, 0, KEYEVENTF_KEYUP, 0);  // x

	b_rubber_band_selection_mode = !b_rubber_band_selection_mode;

}

int num = 0;
void CPointCloudWnd::areaPickCallback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	CPointCloudWnd *pThis = (CPointCloudWnd *)args;
	if (!pThis->b_rubber_band_selection_mode)
		return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	std::vector< int > indices;
	if (event.getPointsIndices(indices) == -1)
		return;

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	std::vector<int>::iterator it;
	for (it = indices.begin(); it != indices.end(); it++) {
		//clicked_cloud->points.push_back(data->clicked_points_3d->points.at(*it));
		inliers->indices.push_back(*it);
	}
	extract.setInputCloud(pThis->m_displayCloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*clicked_cloud);

	pThis->m_displayCloud->clear();
	pThis->m_viewer->updatePointCloud(pThis->m_displayCloud, "cloud");
	pcl::copyPointCloud(*clicked_cloud, *pThis->m_displayCloud);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(pThis->m_displayCloud, "z");
	pThis->m_viewer->updatePointCloud(pThis->m_displayCloud, fildColor, "cloud");
	//pThis->m_viewer->spinOnce(100);
	pThis->UpdateThisWnd();


	//std::stringstream ss;
	//std::string cloudName;
	//ss << num++;
	//ss >> cloudName;
	//cloudName += "_cloudName";
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_cloud, 255, 0, 0);
	//data->viewer->addPointCloud(clicked_cloud, red, cloudName);
	//data->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
	//	5, cloudName);
}

// 属性设置：点云
void CPointCloudWnd::setProperties(int property, double value, const std::string &id)
{
	assert(m_viewer);
	m_viewer->setPointCloudRenderingProperties(property, value, id);
	//m_viewer->getPointCloudRenderingProperties();
	//assert(m_cloud);
	//// 按照z字段进行深度渲染，不同深度不同颜色
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> filledColor(m_cloud, "z");
	//m_viewer->updatePointCloud(m_cloud, filledColor, "cloud");


}

void CPointCloudWnd::getCloudPointNumbers(unsigned &number)
{
	assert(m_displayCloud);
	number = m_displayCloud->size();
}

void CPointCloudWnd::setCloudPointSize(int size)
{
	assert(m_viewer);
	m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "cloud");

	UpdateThisWnd();
}

void CPointCloudWnd::getCloudPointSize(double &size)
{
	assert(m_viewer);
	m_viewer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "cloud");
}

void CPointCloudWnd::setCloudPointColor(int index)
{
	assert(m_viewer);

	switch (index)
	{
	case 0: {
		// 按照z字段进行深度渲染，不同深度不同颜色
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> filledColor(m_displayCloud, "z");
		m_viewer->updatePointCloud(m_displayCloud, filledColor, "cloud");
		break;
	}
	case 1: {
		// Gray
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(m_displayCloud, 128, 128, 128);
		m_viewer->updatePointCloud(m_displayCloud, single_color, "cloud");
		break;
	}
	case 2: {
		// Blue
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(m_displayCloud, 32, 32, 232);
		m_viewer->updatePointCloud(m_displayCloud, single_color, "cloud");
		break;
	}
	case 3: {
		// White
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(m_displayCloud, 232, 232, 232);
		m_viewer->updatePointCloud(m_displayCloud, single_color, "cloud");
		break;
	}
	default:
		break;
	}

	UpdateThisWnd();
}

void CPointCloudWnd::displayCoordinate(bool state)
{
	assert(m_viewer);
	if (state)
		m_viewer->addCoordinateSystem(1);
	else
		m_viewer->removeCoordinateSystem();

	UpdateThisWnd();
}

void CPointCloudWnd::filterEnable(bool state)
{
	assert(m_viewer);

	m_bFilterEnable = state;
	if (state)
		m_actionCode = ACTION_FILTER;
	else {
		if (m_bremoveOutlierEnable && !m_bsmoothEnable)
			UpdatePCLViewer(m_removeOutlieredCloud);
		else if (!m_bremoveOutlierEnable && m_bsmoothEnable)
			UpdatePCLViewer(m_smoothedCloud);
		else if (m_bremoveOutlierEnable && m_bsmoothEnable)
			UpdatePCLViewer(m_displayCloud);
		else
			UpdatePCLViewer(m_sourceCloud);
	}

	UpdateThisWnd();
}

void CPointCloudWnd::filterMethod(int index)
{
	assert(m_sourceCloud);
	m_nFilterMethod = index;
}

void CPointCloudWnd::setFilterParams(float param1, float param2, float param3)
{
	assert(m_sourceCloud);
	m_fFilterParam1 = param1;
	m_fFilterParam2 = param2;
	m_fFilterParam3 = param3;
}

void CPointCloudWnd::getFilterParams(float &param1, float &param2, float &param3)
{
	assert(m_sourceCloud);
	param1 = m_fFilterParam1;
	param2 = m_fFilterParam2;
	param3 = m_fFilterParam3;
}

void CPointCloudWnd::removeOutlierEnable(bool state)
{
	assert(m_viewer);

	m_bremoveOutlierEnable = state;
	if (state)
		m_actionCode = ACTION_REMOVE_OUTLIER;
	else {
		if (m_bFilterEnable && !m_bsmoothEnable)
			UpdatePCLViewer(m_filteredCloud);
		else if (!m_bFilterEnable && m_bsmoothEnable)
			UpdatePCLViewer(m_smoothedCloud);
		else if (m_bFilterEnable && m_bsmoothEnable)
			UpdatePCLViewer(m_displayCloud);
		else
			UpdatePCLViewer(m_sourceCloud);
	}

	UpdateThisWnd();
}

void CPointCloudWnd::setRemoveOutlierParams(float param1, float param2)
{
	assert(m_sourceCloud);
	m_fremoveOutlierParam1 = param1;
	m_fremoveOutlierParam2 = param2;
}

void CPointCloudWnd::getRemoveOutlierParams(float &param1, float &param2)
{
	assert(m_sourceCloud);
	param1 = m_fremoveOutlierParam1;
	param2 = m_fremoveOutlierParam2;
}

void CPointCloudWnd::smoothEnable(bool state)
{
	assert(m_viewer); 

	m_bsmoothEnable = state;
	if (state)
		m_actionCode = ACTION_SMOOTH;
	else {
		if (m_bFilterEnable && !m_bremoveOutlierEnable)
			UpdatePCLViewer(m_filteredCloud);
		else if (!m_bFilterEnable && m_bremoveOutlierEnable)
			UpdatePCLViewer(m_removeOutlieredCloud);
		else if (m_bFilterEnable && m_bremoveOutlierEnable)
			UpdatePCLViewer(m_displayCloud);
		else
			UpdatePCLViewer(m_sourceCloud);
	}

	UpdateThisWnd();
}

void CPointCloudWnd::setSmoothParams(float param1, float param2)
{
	assert(m_sourceCloud);
	m_fsmoothParam1 = param1;
	m_fsmoothParam2 = param2;
}

void CPointCloudWnd::getSmoothParams(float &param1, float &param2)
{
	assert(m_sourceCloud);
	param1 = m_fsmoothParam1;
	param2 = m_fsmoothParam2;
}





//------------------------------------------------- SLOT -------------------------------//

// 窗口刷新
void CPointCloudWnd::onUpdateCloudWnd()
{
	if (m_sourceCloud->size() > 0) {
		Eigen::Vector4f centroid;  //质心 
		pcl::compute3DCentroid(*m_sourceCloud, centroid); // 计算质心
		m_renCamera->SetFocalPoint(centroid.x(), centroid.y(), centroid.z());
	}


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
