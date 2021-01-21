
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
	//�����߳�
	if (!workThread) {
		workThread = new std::thread(&CPointCloudWnd::WorkingOnPointCloud, this);
		if (workThread->joinable())
			workThread->detach();
	}


	//����vtk��Ⱦ������ƺ���Ⱦ����
	m_ren = vtkSmartPointer<vtkRenderer>::New();
	m_renWnd = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	m_renWnd->AddRenderer(m_ren);

	m_monoCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	m_colorCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//��pcl���ӻ�����vtk��Ⱦ����
	m_viewer.reset(new pcl::visualization::PCLVisualizer(m_ren, m_renWnd, "CloudPoint", false));
	//pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB> color_cloud;
	//��ӵ���
	//m_viewer->addPointCloud(m_colorCloud, "cloud");
	//m_viewer->addPointCloud(m_monoCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(m_monoCloud, 0, 255, 255), "cloud");
	//m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");


	//����Ⱦ���ڣ����¼�����
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
//	//����ͨ��openmp�Ż�
//	for (vtkIdType rowId = 0; rowId < size; rowId++){
//		auto dp = pointCloud->points.at(rowId);
//		if (dp.z == 0){
//			continue;
//		}
//		points->InsertNextPoint(dp.x * 1.0 / _iShowScale, dp.y * 1.0 / _iShowScale, dp.z * 1.0 / _iShowScale); //key code
//		n++;
//	}
//	//todoȷ���Ƿ���Ҫ��Ҳ�����ȥ����ͨ��polydataֱ����ʾ
//	vtkSmartPointer<vtkPolyVertex> polyvertex = vtkPolyVertex::New();
//	polyvertex->GetPointIds()->SetNumberOfIds(n);
//	int i = 0;
//	//�������˹�ϵ
//	for (i = 0; i < n; i++){
//		polyvertex->GetPointIds()->SetId(i, i); //todo
//	}
//	vtkSmartPointer<vtkUnstructuredGrid> grid = vtkUnstructuredGrid::New();
//	grid->SetPoints(points);
//	grid->InsertNextCell(polyvertex->GetCellType(), polyvertex->GetPointIds());
//
//	vtkSmartPointer<vtkDataSetMapper> mapper = vtkDataSetMapper::New();
//	mapper->SetInputData(grid);
//	//actor��ʾ���Χ�߿�
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
			//�ж��Ƿ������ɫ����
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

// �̺߳���
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
	////����ֱͨ�˲�����������ɢ��NaN��
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
	//cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected;
	//cloud_projected.reset(new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(m_monoCloud);   //�����������
	//pass.setFilterFieldName("z");//���÷ָ��ֶ�Ϊz����
	//pass.setFilterLimits(0, 1.1);//���÷ָ���ֵ��Χ��z���ϲ��ڸ÷�Χ�ĵ���˵�
	//pass.filter(*cloud_filtered);
	//std::cerr << "PointCloud after filtering has: "
	//	<< cloud_filtered->points.size() << " data points." << std::endl;

	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//inliersָ��洢���Ʒָ��Ľ��

	//// Create the segmentation object
	//pcl::SACSegmentation<pcl::PointXYZ> seg;//�����ָ����
	//// ��ѡ����
	//seg.setOptimizeCoefficients(true);     //�����Ż�ϵ�����ò���Ϊ��ѡ����
	//// ��������
	//seg.setModelType(pcl::SACMODEL_PLANE); //���÷ָ�ģ��ΪSACMODEL_PLANE ƽ��ģ��
	//seg.setMethodType(pcl::SAC_RANSAC);    //���ò���һ���Թ��Ʒ���ģ��ΪSAC_RANSAC
	//seg.setDistanceThreshold(0.01);        //���þ�����ֵΪ0.01�� �������ƽ��ģ�͵ľ���С��0.01m�ĵ㶼Ϊ�ڵ�inliers

	//seg.setInputCloud(cloud_filtered);     //�����������Ϊ�˲���ĵ���
	//seg.segment(*inliers, *coefficients);  //�ָ�����ƽ��ģ��
	//std::cerr << "PointCloud after segmentation has: "
	//	<< inliers->indices.size() << " inliers." << std::endl;

	//// Project the model inliers
	//pcl::ProjectInliers<pcl::PointXYZ> proj; //��������ͶӰ�˲�����
	//proj.setModelType(pcl::SACMODEL_PLANE); //����ͶӰģ��ΪSACMODEL_PLANE
	//proj.setInputCloud(cloud_filtered);     //�����������Ϊ�˲���ĵ���
	//proj.setModelCoefficients(coefficients);//�����Ƶõ���ƽ��ģ��coefficients��������ΪͶӰƽ��ģ��ϵ��
	//proj.filter(*cloud_projected);          //���˲���ĵ���ͶӰ��ƽ��ģ���еõ�ͶӰ��ĵ���cloud_projected
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