
#include "CPointCloudWnd.h"

CPointCloudWnd::CPointCloudWnd(QVTKOpenGLNativeWidget *wnd, QWidget *parent)
	: ui(wnd), QWidget(parent)
{
	//ui.setupUi(this);

	initialVtkWidget();

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


void CPointCloudWnd::initialVtkWidget()
{
	//启动线程
	if (!workThread) {
		workThread = new std::thread(&CPointCloudWnd::WorkingOnPointCloud, this);
		if (workThread->joinable())
			workThread->detach();
	}


	//创建vtk渲染对象控制和渲染窗口
	m_ren = vtkSmartPointer<vtkRenderer>::New();
	m_renWnd = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	m_renWnd->AddRenderer(m_ren);

	m_monoCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	m_colorCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//绑定pcl可视化对象到vtk渲染窗口
	m_viewer.reset(new pcl::visualization::PCLVisualizer(m_ren, m_renWnd, "CloudPoint", false));
	//pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB> color_cloud;
	//添加点云
	//m_viewer->addPointCloud(m_colorCloud, "cloud");
	//m_viewer->addPointCloud(m_monoCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(m_monoCloud, 0, 255, 255), "cloud");
	//m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");


	//绑定渲染窗口，绑定事件交互
	ui->SetRenderWindow(m_viewer->getRenderWindow());
	m_viewer->setupInteractor(ui->GetInteractor(), ui->GetRenderWindow());

	ui->update();
}


void CPointCloudWnd::showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
//	if (pointCloud->points.size() == 0){
//		warningWindow(STR_3D_IMAGE_ERROR_TITLE);
//		return;
//	}
//	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New(); //key code
//	vtkIdType size = pointCloud->points.size();
//	int n = 0;
//	//可以通过openmp优化
//	for (vtkIdType rowId = 0; rowId < size; rowId++){
//		auto dp = pointCloud->points.at(rowId);
//		if (dp.z == 0){
//			continue;
//		}
//		points->InsertNextPoint(dp.x * 1.0 / _iShowScale, dp.y * 1.0 / _iShowScale, dp.z * 1.0 / _iShowScale); //key code
//		n++;
//	}
//	//todo确认是否需要，也许可以去掉，通过polydata直接显示
//	vtkSmartPointer<vtkPolyVertex> polyvertex = vtkPolyVertex::New();
//	polyvertex->GetPointIds()->SetNumberOfIds(n);
//	int i = 0;
//	//建立拓扑关系
//	for (i = 0; i < n; i++){
//		polyvertex->GetPointIds()->SetId(i, i); //todo
//	}
//	vtkSmartPointer<vtkUnstructuredGrid> grid = vtkUnstructuredGrid::New();
//	grid->SetPoints(points);
//	grid->InsertNextCell(polyvertex->GetCellType(), polyvertex->GetPointIds());
//
//	vtkSmartPointer<vtkDataSetMapper> mapper = vtkDataSetMapper::New();
//	mapper->SetInputData(grid);
//	//actor显示外包围边框
//#if 0
////    vtkSmartPointer<vtkOutlineFilter>outLineData = vtkSmartPointer<vtkOutlineFilter>::New();
////    outLineData->SetInputData(grid);
////
////    vtkSmartPointer<vtkPolyDataMapper> mapOutline = vtkSmartPointer<vtkPolyDataMapper>::New();
////    mapOutline->SetInputConnection(outLineData->GetOutputPort());
////
////    vtkSmartPointer<vtkActor>outline = vtkSmartPointer<vtkActor>::New();
////    outline->SetMapper(mapOutline);
////    outline->GetProperty()->SetColor(0, 0, 0);
////    _renderer->AddActor(outline);
//#endif
//	_renderer->RemoveActor(_tmpmodelpointCloudActor);
//	_tmpmodelpointCloudActor = vtkActor::New();
//	_tmpmodelpointCloudActor->SetMapper(mapper);
//	_tmpmodelpointCloudActor->GetProperty()->SetColor(0.9, 0.2, 0.9);
//	_renderer->AddActor(_tmpmodelpointCloudActor);
//	_qvtkWidget->GetRenderWindow()->Render();
}


void CPointCloudWnd::displayOnVTK1()
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
				pcl::io::loadPCDFile(fileName.toStdString(), *m_colorCloud);
			else
				pcl::io::loadPCDFile(fileName.toStdString(), *m_monoCloud);
		}
		else if (data_type == 2) {
			pcl::PCDReader reader;
			if (cloud2.fields.size() > 3)
				reader.read<pcl::PointXYZRGBA>(fileName.toStdString(), *m_colorCloud);
			else
				reader.read<pcl::PointXYZ>(fileName.toStdString(), *m_monoCloud);
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
	m_ren->ResetCamera();
	m_renWnd->Render();
}

// 线程函数
void CPointCloudWnd::WorkingOnPointCloud()
{


	while (1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(30));



	}




}
//------------------------------------------------- SLOT -------------------------------//
void CPointCloudWnd::onOpenPCL()
{
	displayOnVTK1();

	//displaySphere();


}


void CPointCloudWnd::onSelect()
{

}


void CPointCloudWnd::onDelete()
{

}


void CPointCloudWnd::onAdd()
{

}


void CPointCloudWnd::onClear()
{

}


void CPointCloudWnd::onSurfaceRebuild()
{
	////建立直通滤波器，消除杂散的NaN点
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
	//cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected;
	//cloud_projected.reset(new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(m_monoCloud);   //设置输入点云
	//pass.setFilterFieldName("z");//设置分隔字段为z坐标
	//pass.setFilterLimits(0, 1.1);//设置分割阈值范围，z轴上不在该范围的点过滤掉
	//pass.filter(*cloud_filtered);
	//std::cerr << "PointCloud after filtering has: "
	//	<< cloud_filtered->points.size() << " data points." << std::endl;

	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//inliers指针存储点云分割后的结果

	//// Create the segmentation object
	//pcl::SACSegmentation<pcl::PointXYZ> seg;//创建分割对象
	//// 可选设置
	//seg.setOptimizeCoefficients(true);     //设置优化系数，该参数为可选设置
	//// 必须设置
	//seg.setModelType(pcl::SACMODEL_PLANE); //设置分割模型为SACMODEL_PLANE 平面模型
	//seg.setMethodType(pcl::SAC_RANSAC);    //设置采样一致性估计方法模型为SAC_RANSAC
	//seg.setDistanceThreshold(0.01);        //设置距离阈值为0.01， 即与估计平面模型的距离小于0.01m的点都为内点inliers

	//seg.setInputCloud(cloud_filtered);     //设置输入点云为滤波后的点云
	//seg.segment(*inliers, *coefficients);  //分割结果：平面模型
	//std::cerr << "PointCloud after segmentation has: "
	//	<< inliers->indices.size() << " inliers." << std::endl;

	//// Project the model inliers
	//pcl::ProjectInliers<pcl::PointXYZ> proj; //创建点云投影滤波对象
	//proj.setModelType(pcl::SACMODEL_PLANE); //设置投影模型为SACMODEL_PLANE
	//proj.setInputCloud(cloud_filtered);     //设置输入点云为滤波后的点云
	//proj.setModelCoefficients(coefficients);//将估计得到的平面模型coefficients参数设置为投影平面模型系数
	//proj.filter(*cloud_projected);          //将滤波后的点云投影到平面模型中得到投影后的点云cloud_projected
	//std::cerr << "PointCloud after projection has: "
	//	<< cloud_projected->points.size() << " data points." << std::endl;
	//   

	////m_viewer->addPointCloud(cloud_projected, "cloud");
	//m_viewer->addPointCloud(cloud_filtered, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_filtered, 0, 0, 255), "cloud");
	//m_viewer->updatePointCloud(cloud_filtered, "cloud");

	////m_viewer->addPolygonMesh(mesh, "my");
	////m_viewer->addPointCloudNormals(m_monoCloud, mls_points, 100, 0.02f, "cloud", 0);
	//m_renWnd->Render();


	RebuildTest();
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
	m_viewer->resetCamera();
	//m_viewer->resetCameraViewpoint();
	//m_viewer->setPosition(100, 100);
	//m_viewer->spin();      //Calls the interactor and runs an internal loop
	m_viewer->updatePolygonMesh(triangles, "mesh");
	m_renWnd->Render();

}


void CPointCloudWnd::Draw()
{


}