
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
	//����vtk��Ⱦ������ƺ���Ⱦ����
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
	// ��pcl���ӻ����� VTK ��Ⱦ����
	m_viewer.reset(new pcl::visualization::PCLVisualizer(m_ren, m_renWnd, "CloudPoint", false));
	m_viewer->setupInteractor(m_iren, m_renWnd);
	m_viewer->setBackgroundColor(0.01, 0.5, 0.6);
	//m_viewer->addCoordinateSystem(1);
	m_viewer->addSphere(pcl::PointXYZ(0, 0, 0), 0.05, 0.3, 0.3, 0.0, "sphere");
	m_viewer->addPointCloud(m_displayCloud, "cloud");
	m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "cloud");
	m_viewer->registerAreaPickingCallback(areaPickCallback, this);


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
			UpdateThisWnd();
		}
	}
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
	// ��Ļ����ϵ > ��������ϵ
	double dispCoord[2] = { 10, 30 };
	vtkSmartPointer<vtkCoordinate> pCoorPress = vtkSmartPointer<vtkCoordinate>::New();
	pCoorPress->SetCoordinateSystemToDisplay();
	pCoorPress->SetValue(dispCoord);
	double *worldCoord = pCoorPress->GetComputedWorldValue(m_ren);
	qDebug() << worldCoord[0] << worldCoord[1] << worldCoord[2];

	// �������ϵ
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

		////�ж��Ƿ������ɫ����
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
	// ����z�ֶν��������Ⱦ����ͬ��Ȳ�ͬ��ɫ
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> filledColor(m_displayCloud, "z");
	m_viewer->updatePointCloud(m_displayCloud, filledColor, "cloud");

}


void CPointCloudWnd::clearDisplayCloud()
{
	// ɾ����ǰ�����еĵ��ƶ���, ���Զ����ô���ˢ��
	
	//m_viewer->removePointCloud("cloud");
	m_displayCloud->clear();

	m_viewer->updatePointCloud(m_displayCloud, "cloud");
}


void CPointCloudWnd::RebuildTest()
{
	//// Normal estimation*
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���÷��߹��ƶ���
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	//tree->setInputCloud(m_cloud);//��cloud����tree����
	//n.setInputCloud(m_cloud);//Ϊ���߹��ƶ��������������
	//n.setSearchMethod(tree);//������������
	//n.setKSearch(20);//����k�������ص�������Χ
	//n.compute(*normals);//���Ʒ���
	////* normals should not contain the point normals + surface curvatures

	//// Concatenate the XYZ and normal fields*
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);//
	//pcl::concatenateFields(*m_cloud, *normals, *cloud_with_normals);//�����ֶΣ�cloud_with_normals�洢�������
	////* cloud_with_normals = cloud + normals

	//// Create search tree*
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//��������������
	//tree2->setInputCloud(cloud_with_normals);//����������ƹ���tree

	//// Initialize objects
	//pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//�������ǻ�����
	//pcl::PolygonMesh triangles;//�洢�������ǻ�������ģ��

	//// Set the maximum distance between connected points (maximum edge length)
	//gp3.setSearchRadius(0.025);         //���������뾶radius����ȷ�����ǻ�ʱkһ�ڽ�����뾶��

	//// Set typical values for the parameters
	//gp3.setMu(2.5);                     //���������㵽����������ĳ˻�ϵ�� mu �����ÿ�������������������룬����ʹ���㷨����Ӧ�����ܶȵı仯
	//gp3.setMaximumNearestNeighbors(100);//����������������������������Ŀ100 ��
	//gp3.setMaximumSurfaceAngle(M_PI / 4);  //45 degrees����������ʱ�����Ƕ� eps_angle ����ĳ�㷨������ڲ�����ķ���ƫ��Ƕȳ��������Ƕ�ʱ������ʱ�Ͳ����Ǹõ㡣
	//gp3.setMinimumAngle(M_PI / 18);        //10 degrees���������ǻ��������ε���С�ǣ����� minimum_angle Ϊ��С�ǵ�ֵ��
	//gp3.setMaximumAngle(2 * M_PI / 3);       //120 degrees���������ǻ��������ε����ǣ����� maximum_angle Ϊ���ǵ�ֵ��
	//gp3.setNormalConsistency(false);     //����һ����־ consistent ������֤���߳���һ�£��������Ϊ true ���ʹ���㷨���ַ��߷���һ�£����Ϊ false �㷨�򲻻���з���һ���Լ�顣

	//// Get result
	//gp3.setInputCloud(cloud_with_normals);//�����������Ϊ�������
	//gp3.setSearchMethod(tree2);           //����������ʽtree2
	//gp3.reconstruct(triangles);           //�ؽ���ȡ���ǻ�
 //  // std::cout << triangles;
	//// Additional vertex information
	//std::vector<int> parts = gp3.getPartIDs();//����ؽ���ÿ��� ID, Parts �� 0 ��ʼ��ţ� a-1 ��ʾδ���ӵĵ㡣
	///*
	//����ؽ���ÿ���״̬��ȡֵΪ FREE �� FRINGE �� BOUNDARY �� COMPLETED �� NONE ������
	//���� NONE ��ʾδ���壬
	//FREE ��ʾ�õ�û���� �� �ǻ���������ڣ�Ϊ���ɵ㣬
	//COMPLETED ��ʾ�õ������ǻ���������ڣ��������������˵㣬
	//BOUNDARY ��ʾ�õ������ǻ�������˱�Ե��
	//FRINGE ��ʾ�õ��� �� �ǻ���������ڣ������ӻ�����ص��ߡ�
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

// �����˲�
void CPointCloudWnd::filteredCloud(int method,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudIn,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudOut,
	float param1, float param2, float param3)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
	qDebug() << "before filtered: size " << (*cloudIn)->size();

	if (method == 0) {
		pcl::VoxelGrid<pcl::PointXYZ> sor;       //���ػ������˲���ʵ���²���
		sor.setInputCloud(*cloudIn);
		sor.setLeafSize(param1, param1, param1);    //�������ش�С:��λ���ף�
		sor.filter(**cloudOut);
	}
	else if (method == 1) {
		pcl::UniformSampling<pcl::PointXYZ> unfm;   // ���Ȳ����˲�
		unfm.setInputCloud(*cloudIn);
		unfm.setRadiusSearch(param1);                //���������뾶
		unfm.filter(**cloudOut);
	}
	else if (method == 2) {
		// ������
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;  // ������С����ʵ�ֵĶ���mls
		mls.setInputCloud(*cloudIn);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
		mls.setSearchMethod(kdtree);
		mls.setSearchRadius(param1);                                  // ���� k�ٽ� �뾶
		// Upsampling �����ķ����� DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
		mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
		mls.setUpsamplingRadius(param2);
		mls.setUpsamplingStepSize(param3);
		mls.setPolynomialFit(false);                                 // ����Ϊfalse���� ���� smooth
		mls.process(**cloudOut);
	}
	else if (method == 3) {
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downSampled(new
		//	pcl::PointCloud<pcl::PointXYZ>());
		//pcl::VoxelGrid<pcl::PointXYZ> downSampled;  //�²����˲���
		//downSampled.setInputCloud(m_cloud);            //������Ҫ���˵ĵ��Ƹ��˲�����
		//downSampled.setLeafSize(0.01f, 0.01f, 0.01f);  //�����˲�ʱ�������������Ϊ1cm��������
		//downSampled.filter(*cloud_downSampled);           //ִ���˲������洢���

		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed(new
		//	pcl::PointCloud<pcl::PointXYZ>());
		//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //��Ⱥ�޳��˲���
		//sor.setInputCloud(cloud_downSampled);                //���ô��˲��ĵ���
		//sor.setMeanK(50);            //�����ڽ���ͳ��ʱ���ǵ��ٽ������
		//sor.setStddevMulThresh(1.0); //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ���������˱�׼�Ҳ���������std_mul
		//sor.filter(*cloud_removed);  //�˲�����洢��cloud_filtered

		//pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new
		//	pcl::search::KdTree<pcl::PointXYZ>); // mlsƽ������
		//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;  // ������С����ʵ�ֵĶ���mls
		//mls.setSearchMethod(treeSampling);    // ����KD-Tree��Ϊ��������
		//mls.setComputeNormals(false);  //��������С���˼������Ƿ���Ҫ�洢����ķ���
		//mls.setInputCloud(cloud_removed);        //���ô��������
		//mls.setPolynomialOrder(2);             // ���2�׶���ʽ���
		//mls.setPolynomialFit(false);  // ����Ϊfalse���� ���� smooth
		//mls.setSearchRadius(0.05); // ��λm.����������ϵ�K���ڰ뾶
		//mls.process(*filtered);        //���
	}

	qDebug() << "success filtered, size: " << (*cloudOut)->size();

	//m_sourceCloud->clear();
	//m_viewer->updatePointCloud(m_sourceCloud, "cloud");
	//pcl::copyPointCloud(*filtered, *m_sourceCloud);
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(m_sourceCloud, "z");
	//m_viewer->updatePointCloud(m_sourceCloud, fildColor, "cloud");
}

// �Ƴ���Ⱥ��
void CPointCloudWnd::removeOutlierPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudIn,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudOut,
	float param1, float param2)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //��Ⱥ�޳��˲���
	sor.setInputCloud(*cloudIn);                //���ô��˲��ĵ���
	sor.setMeanK((int)param1);            //�����ڽ���ͳ��ʱ���ǵ��ٽ������
	sor.setStddevMulThresh((double)param2); //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ���������˱�׼�Ҳ���������std_mul
	sor.filter(**cloudOut);  //�˲�����洢��cloud_filtered
}

// ����ƽ��:��С���˷�
void CPointCloudWnd::smoothCloudPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudIn,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloudOut,
	float param1, float param2)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new
		pcl::search::KdTree<pcl::PointXYZ>); // mlsƽ������
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;  // ������С����ʵ�ֵĶ���mls
	mls.setSearchMethod(treeSampling);    // ����KD-Tree��Ϊ��������
	mls.setComputeNormals(false);  //��������С���˼������Ƿ���Ҫ�洢����ķ���
	mls.setInputCloud(*cloudIn);        //���ô��������
	mls.setPolynomialOrder(2);             // ���2�׶���ʽ���
	mls.setPolynomialFit(false);  // ����Ϊfalse���� ���� smooth
	mls.setSearchRadius((double)param1); // ��λm.����������ϵ�K���ڰ뾶
	mls.process(**cloudOut);        //���
}

// ���񻯣�̰�����ǻ��㷨
void CPointCloudWnd::buildMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new 
		pcl::PointCloud<pcl::PointNormal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);// ��������ĵ��Ʒ���
	// �������������������KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	// K����ȷ��������ʹ��k������㣬����ȷ��һ����rΪ�뾶��Բ�ڵĵ㼯��ȷ�������ԣ�����ѡ1����
	n.setKSearch(20);   // ʹ�õ�ǰ����Χ�����20����
	//n.setRadiusSearch(0.05);//����ÿһ���㶼�ð뾶Ϊ5cm�Ľ���������ʽ
	n.compute(*normals);
	// ������λ�ˡ���ɫ��������Ϣ���ӵ�һ��
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//��������������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	// �������ǻ�����
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//pcl::PolygonMesh mesh; //�洢�������ǻ�������ģ��
	//��������ʱ�İ뾶��Ҳ����KNN����뾶, ���������Ҫ����
	gp3.setSearchRadius(2);
	//������������������ڵ����Զ����Ϊ2.5��������ֵ2.5-3��������ʹ���㷨����Ӧ�����ܶȵı仯
	gp3.setMu(2.5);
	//���������������������������������ֵ��50-100
	gp3.setMaximumNearestNeighbors(100);
	// ����ĳ�㷨�߷���ƫ�������㷨�ߵ����Ƕ�45�㣬�������������ʱ�����Ǹõ�
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	// �������ǻ���õ����������ڽǵ���С�ĽǶ�Ϊ10��
	gp3.setMinimumAngle(M_PI / 18);
	// �������ǻ���õ����������ڽǵ����Ƕ�Ϊ120��
	gp3.setMaximumAngle(2 * M_PI / 3);
	//���øò�����֤���߳���һ��
	gp3.setNormalConsistency(false);
	//�����������Ϊ�������
	gp3.setInputCloud(cloud_with_normals);
	//�������ط�ʽΪtree2
	gp3.setSearchMethod(tree2);
	//�ؽ���ȡ���ǻ�
	gp3.reconstruct(m_mesh);
	
	cloud->clear();
	m_viewer->updatePointCloud(m_sourceCloud, "cloud");
	m_viewer->addPolygonMesh(m_mesh, "mesh");
	//��������ģ����ʾģʽ
	//viewer->setRepresentationToSurfaceForAllActors(); //����ģ������Ƭ��ʽ��ʾ  
	//viewer->setRepresentationToPointsForAllActors(); //����ģ���Ե���ʽ��ʾ  
	//m_viewer->setRepresentationToWireframeForAllActors();  //����ģ�����߿�ͼģʽ��ʾ

	pcl::io::savePLYFile("test_save_mesh.ply", m_mesh);
}

// ��ѡɾ������: ���� rubber_band_selection_mode 
void CPointCloudWnd::userSelect()
{
	HKL	hcurkl;
	hcurkl = GetKeyboardLayout(0);
	//������л�Ϊ��ʽ���̵�Ӣ�����뷨
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

// �������ã�����
void CPointCloudWnd::setProperties(int property, double value, const std::string &id)
{
	assert(m_viewer);
	m_viewer->setPointCloudRenderingProperties(property, value, id);
	//m_viewer->getPointCloudRenderingProperties();
	//assert(m_cloud);
	//// ����z�ֶν��������Ⱦ����ͬ��Ȳ�ͬ��ɫ
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
		// ����z�ֶν��������Ⱦ����ͬ��Ȳ�ͬ��ɫ
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

// ����ˢ��
void CPointCloudWnd::onUpdateCloudWnd()
{
	if (m_sourceCloud->size() > 0) {
		Eigen::Vector4f centroid;  //���� 
		pcl::compute3DCentroid(*m_sourceCloud, centroid); // ��������
		m_renCamera->SetFocalPoint(centroid.x(), centroid.y(), centroid.z());
	}


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
			//qDebug() << "picking screen coordinate: " << pixel_point[0] << ", " << pixel_point[1];

			// ��ȡ��ǰ�¼�������renderer�� �п���Ϊnullptr
			vtkRenderer* renderer = m_renWnd->GetInteractor()->FindPokedRenderer(pixel_point[0], pixel_point[1]);

			// ��ȡVTK��������
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
