

#include "FlexibleArray.h"


FlexibleArray::FlexibleArray(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

	m_timerUpdate = new QTimer(this);
	m_timerUpdate->stop();
	m_timerUpdate->setInterval(1000);    //  定时 1 秒

	m_pCloud = new CPointCloudWnd(ui.vtkOpenGLWidget, this);
	if (m_pCloud == nullptr)
		return;

	// 图像显示窗口
	m_pimagewnd = new CImageWnd(this);
	if (m_pimagewnd == nullptr)
		return;
	m_pimagewnd->move(305, 80);

	// 相机控制
	HWND hnd = (HWND)m_pimagewnd->winId();
	m_pcamera = new CCameraControl(hnd);
	if (m_pcamera == nullptr)
		return;
	if (m_pcamera->initialThisClass() != 0) {
		QMessageBox msg(QMessageBox::Warning, "Information", "未发现设备!", QMessageBox::Ok);

		//exit(-1);
	}



	// 相位计算
	m_pcaculate = new CPhaseCaculate();
	if (m_pcaculate == nullptr)
		return;

	// 图像拼接成点云
	m_pStitch = new CPanoramaStitching();
	if (m_pStitch == nullptr)
		return;


	//m_mdiArea = new QMdiArea(this);
	//if (m_mdiArea == nullptr) exit(1);
	//this->setCentralWidget(m_mdiArea);
	//窗口级联模式
	//m_mdiArea->cascadeSubWindows();
	   	 
	//CreateCameraDisplayWnd();
	//CreatePointCloudWnd();
	//CreateCameraDisplayWnd();

	//m_mdiArea->addSubWindow(m_pcloudwnd);
	//m_mdiArea->addSubWindow(m_pimagewnd1);
	//m_mdiArea->addSubWindow(m_pimagewnd2);

	//QHBoxLayout *pHBox = new QHBoxLayout();
	//pHBox->addWidget(m_pimagewnd1);
	//pHBox->addWidget(m_pimagewnd2);
	//centralWidget()->setLayout(pHBox);
	
	InitialStatusBar();
	InitialDockWidget(300);
	InitialProjectTree();
	InitialPropertyTree();
	InitialConnection();

	m_timerUpdate->start();
}

FlexibleArray::~FlexibleArray()
{

}


void FlexibleArray::closeEvent(QCloseEvent *event)
{
	m_pCloud->DestroyThisWnd();


}


void FlexibleArray::InitialConnection()
{
	connect(m_timerUpdate, SIGNAL(timeout()), this, SLOT(onTimerUpdata()));

	connect(this, SIGNAL(signalDisplay(pcl::PointCloud<PointType>::Ptr)), m_pCloud, SIGNAL(signalDisplay(pcl::PointCloud<PointType>::Ptr)));
	connect(this, &FlexibleArray::signalOpenPCL,         m_pCloud, &CPointCloudWnd::signalOpenPCL);
	connect(this, &FlexibleArray::signalSelect,          m_pCloud, &CPointCloudWnd::signalSelect);
	connect(this, &FlexibleArray::signalDelete,          m_pCloud, &CPointCloudWnd::signalDelete);
	connect(this, &FlexibleArray::signalAdd,             m_pCloud, &CPointCloudWnd::signalAdd);
	connect(this, &FlexibleArray::signalFilter,          m_pCloud, &CPointCloudWnd::signalFilter);
	connect(this, &FlexibleArray::signalMesh,            m_pCloud, &CPointCloudWnd::signalMesh);
	connect(this, &FlexibleArray::signalSurfaceRebuild,  m_pCloud, &CPointCloudWnd::signalSurfaceRebuild);
	connect(m_dbroot, &CDBRoot::signalRefrushImgWnd, this, &FlexibleArray::onRootClickedRefrushImgWnd);

}


void FlexibleArray::InitialStatusBar()
{
	QLabel* label1 = new QLabel(QString::fromLocal8Bit("柔性阵列成像系统演示软件"), this);
	QLabel* label2 = new QLabel(QString::fromLocal8Bit("浙江清华柔性电子技术研究院"), this);
	QLabel* label3 = new QLabel(QString::fromLocal8Bit("国防科技大学"), this);
	m_labelTime = new QLabel(QString::fromLocal8Bit(""), this);

	ui.statusBar->addWidget(label1, 1);
	ui.statusBar->addWidget(label2, 1);
	ui.statusBar->addWidget(label3, 1);
	ui.statusBar->addWidget(m_labelTime, 1);
	
}


void FlexibleArray::UpdateStatusBar()
{
	QDateTime current_date_time = QDateTime::currentDateTime();
	//QString current_date = current_date_time.toString("yyyy.MM.dd hh:mm:ss.zzz ddd");
	QString current_date = current_date_time.toString(QString::fromLocal8Bit("yyyy年MM月dd日  hh时mm分ss秒"));

	m_labelTime->setText(current_date);
}


void FlexibleArray::InitialDockWidget(int width)
{
	//设置宽度
	//ui.dockWidget_Project->setMinimumWidth(width);

	//InitialTree();

}

// 项目树
void FlexibleArray::InitialProjectTree()
{
	m_dbroot = new CDBRoot(m_pCloud, ui.treeViewProjects, ui.treeViewProperty, this);

	QStandardItemModel* model = new QStandardItemModel(ui.treeViewProjects);
	model->setHorizontalHeaderLabels(QStringList() << QStringLiteral("项目名称") << QStringLiteral("信息"));
	QStandardItem* itemProject = new QStandardItem(QIcon(QStringLiteral("res/folder_image.ico")), QStringLiteral("全景成像"));
	model->appendRow(itemProject);
	QStandardItem *childItem1 = new QStandardItem(QStringLiteral("设备"));
	QStandardItem *childItem2 = new QStandardItem(QStringLiteral("图像"));
	QStandardItem *childItem3 = new QStandardItem(QStringLiteral("点云"));
	QStandardItem *childItem4 = new QStandardItem(QStringLiteral("网格"));
	itemProject->appendRow(childItem1);
	itemProject->appendRow(childItem2);
	itemProject->appendRow(childItem3);
	itemProject->appendRow(childItem4);
	QStandardItem *image1 = new QStandardItem(QStringLiteral("图像1"));
	QStandardItem *image2 = new QStandardItem(QStringLiteral("图像2"));
	QStandardItem *image3 = new QStandardItem(QStringLiteral("图像3"));
	QStandardItem *image4 = new QStandardItem(QStringLiteral("图像4"));
	QStandardItem *image5 = new QStandardItem(QStringLiteral("图像5"));
	QStandardItem *image6 = new QStandardItem(QStringLiteral("图像6"));
	QStandardItem *image7 = new QStandardItem(QStringLiteral("图像7"));
	childItem2->appendRow(image1);
	childItem2->appendRow(image2);
	childItem2->appendRow(image3);
	childItem2->appendRow(image4);
	childItem2->appendRow(image5);
	childItem2->appendRow(image6);
	childItem2->appendRow(image7);

	//设置虚线， 给QTreeView应用model
	ui.treeViewProjects->setStyle(QStyleFactory::create("windows"));
	ui.treeViewProjects->setModel(model);
	ui.treeViewProjects->expandAll();
}

// 属性
void FlexibleArray::InitialPropertyTree()
{

}

//************************************SLOTS******************************//
void FlexibleArray::onTimerUpdata()
{
	UpdateStatusBar();

}



void FlexibleArray::systemOnline()
{

}

void FlexibleArray::systemOffline()
{

}

void FlexibleArray::onActionProjectNewClicked()
{

}

void FlexibleArray::onActionProjectSaveClicked()
{
	qDebug("Save clicked");
	if (!m_pStitch->m_pcloud_stitch->empty()) {
		QString filename = m_qstrImgPath + QString::number(m_pStitch->m_nOpenAngleSelect) + ".pcd";
		QByteArray byName = filename.toLocal8Bit();
		m_pStitch->SaveCloudFile(byName.data());

		QMessageBox::information(nullptr, "Information", "point_cloud Save Successfully!");
	}

}

void FlexibleArray::onActionProjectOpenFileClicked()
{
	//emit signalOpenPCL();

	QString imgPath = QFileDialog::getExistingDirectory(this, QStringLiteral("选择图像目录"), "./");
	if (!imgPath.isEmpty()) {
		m_qstrImgPath = imgPath + "/";
		m_dbroot->m_qstrImgPath = imgPath + "/";


		//QByteArray cdata = imgPath.toLocal8Bit();
		//m_pcdPath = std::string(cdata);
		//m_actionCode = ACTION_OPEN;
	}
}

void FlexibleArray::onActionProjectOpenCalibrationClicked()
{
	if (!m_pcalibwnd) {
		m_pcalibwnd = new CCalibrationWnd;
		if (m_pcalibwnd == NULL) return;
		//ui.mdiArea->addSubWindow(m_pcalibwnd);
		m_pcalibwnd->show();

	}
}

void FlexibleArray::onActionSetupSystemClicked()
{

}

void FlexibleArray::onActionSetupCameraClicked()
{
	//m_pcamera->openCamera();
	//m_pcamera->setCameraTriggeModel(5.0);


}

void FlexibleArray::onActionSetupProjectorClicked()
{
	//m_pcamera->closeCamera();
}

void FlexibleArray::onActionStartClicked()
{
	//m_dbroot->updateObject();
	//m_pcamera->acquireImages();

	//qDebug("Start clicked");

	QByteArray byPath = m_qstrImgPath.toLocal8Bit();

	if (m_dbroot->m_pOpeningAngle != nullptr && m_dbroot->m_pSerialIndex != nullptr) {
		int angle = *m_dbroot->m_pOpeningAngle;
		int serialindex = *m_dbroot->m_pSerialIndex;
		m_pStitch->ImageStitchUseRotateMatrix(byPath.data(), serialindex, angle);
		qDebug("angle = %d, serialindex = %d", angle, serialindex);

		emit signalDisplay(m_pStitch->m_pcloud_stitch);
	}
}

void FlexibleArray::onActionStopClicked()
{
	//m_pcamera->stopAcquireImages();
}

void FlexibleArray::onActionPointCloudSelectClicked()
{
	//emit signalSelect();
}

void FlexibleArray::onActionPointCloudDeleteClicked()
{
	//emit signalDelete();
}

void FlexibleArray::onActionPointCloudAddClicked()
{
	//emit signalAdd();
}

void FlexibleArray::onActionPointCloudFilterClicked()
{
	//emit signalFilter();
}

void FlexibleArray::onActionPointCloudMeshClicked()
{
	//emit signalMesh();
}

void FlexibleArray::onActionViewPlotClicked()
{
	qDebug("view plot clicked");


}

void FlexibleArray::onActionView2DClicked()
{
	//emit signalSurfaceRebuild();
}

void FlexibleArray::onActionView3DClicked()
{
	//bool statue = false;
	//statue = !m_pcamera->getUserOutputStatue();

	//m_pcamera->setUserOutput(0, statue);
}

void FlexibleArray::onActionViewFrontClicked()
{

}

void FlexibleArray::onActionViewBackClicked()
{

}

void FlexibleArray::onActionViewLeftClicked()
{

}

void FlexibleArray::onActionViewRightClicked()
{

}

void FlexibleArray::onActionViewTopClicked()
{

}

void FlexibleArray::onActionViewBottomClicked()
{

}

void FlexibleArray::onRootClickedRefrushImgWnd()
{
	//qDebug() << m_dbroot->m_qstrImgPath + m_dbroot->m_qstrImgName;
	QString filename = m_dbroot->m_qstrImgPath + m_dbroot->m_qstrImgName;
	QImage img;
	img.load(filename);
	//if (img.load(filename)) {
	//	QMessageBox::critical(this, "Error", "Read Image Error!");
	//	return;
	//}
	m_pimagewnd->setFrame(QPixmap::fromImage(img));

	qDebug("%s", filename);
}
