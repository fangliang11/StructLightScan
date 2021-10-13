

#include "StructScan.h"
#include "CImageWnd.h"
#include "CCalibrationWnd.h"
#include "CPointCloudWnd.h"
#include "CDBRoot.h"
#include "CCameraControl.h"

#include "CPhaseCaculate.h"

StructScan::StructScan(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

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
	
	InitialConnection();
	InitialStatusBar();
	InitialDockWidget(300);
	InitialProjectTree();
	InitialPropertyTree();
}

StructScan::~StructScan()
{

}


void StructScan::closeEvent(QCloseEvent *event)
{
	m_pCloud->DestroyThisWnd();


}


void StructScan::InitialConnection()
{
	connect(this, &StructScan::signalOpenPCL,         m_pCloud, &CPointCloudWnd::signalOpenPCL);
	connect(this, &StructScan::signalSelect,          m_pCloud, &CPointCloudWnd::signalSelect);
	connect(this, &StructScan::signalDelete,          m_pCloud, &CPointCloudWnd::signalDelete);
	connect(this, &StructScan::signalAdd,             m_pCloud, &CPointCloudWnd::signalAdd);
	connect(this, &StructScan::signalFilter,          m_pCloud, &CPointCloudWnd::signalFilter);
	connect(this, &StructScan::signalMesh,            m_pCloud, &CPointCloudWnd::signalMesh);
	connect(this, &StructScan::signalSurfaceRebuild,  m_pCloud, &CPointCloudWnd::signalSurfaceRebuild);
}


void StructScan::InitialStatusBar()
{
	QLabel* label1 = new QLabel(tr("Label1"), this);
	QLabel* label2 = new QLabel(tr("Label2"), this);
	QLabel* label3 = new QLabel(tr("Label3"), this);
	QLabel* label4 = new QLabel(tr("Label4"), this);
	ui.statusBar->addWidget(label1, 1);
	ui.statusBar->addWidget(label2, 1);
	ui.statusBar->addWidget(label3, 1);
	ui.statusBar->addWidget(label4, 1);
	
}


void StructScan::InitialDockWidget(int width)
{
	//设置宽度
	//ui.dockWidget_Project->setMinimumWidth(width);

	//InitialTree();

}

// 项目树
void StructScan::InitialProjectTree()
{
	m_dbroot = new CDBRoot(m_pCloud, ui.treeViewProjects, ui.treeViewProperty, this);

	QStandardItemModel* model = new QStandardItemModel(ui.treeViewProjects);
	model->setHorizontalHeaderLabels(QStringList() << QStringLiteral("项目名称") << QStringLiteral("信息"));
	QStandardItem* itemProject = new QStandardItem(QIcon(QStringLiteral("res/folder_image.ico")), QStringLiteral("三维测量"));
	model->appendRow(itemProject);
	QStandardItem *childItem1 = new QStandardItem(QStringLiteral("设备"));
	//QStandardItem *childItem2 = new QStandardItem(QStringLiteral("图像"));
	QStandardItem *childItem3 = new QStandardItem(QStringLiteral("点云"));
	QStandardItem *childItem4 = new QStandardItem(QStringLiteral("网格"));
	itemProject->appendRow(childItem1);
	//itemProject->appendRow(childItem2);
	itemProject->appendRow(childItem3);
	itemProject->appendRow(childItem4);
	//QStandardItem *image1 = new QStandardItem(QStringLiteral("图像1"));
	//QStandardItem *image2 = new QStandardItem(QStringLiteral("图像2"));
	//QStandardItem *image3 = new QStandardItem(QStringLiteral("图像3"));
	//QStandardItem *image4 = new QStandardItem(QStringLiteral("图像4"));
	//QStandardItem *image5 = new QStandardItem(QStringLiteral("图像5"));
	//childItem2->appendRow(image1);
	//childItem2->appendRow(image2);
	//childItem2->appendRow(image3);
	//childItem2->appendRow(image4);
	//childItem2->appendRow(image5);

	//设置虚线， 给QTreeView应用model
	ui.treeViewProjects->setStyle(QStyleFactory::create("windows"));
	ui.treeViewProjects->setModel(model);
	ui.treeViewProjects->expandAll();
}

// 属性
void StructScan::InitialPropertyTree()
{

}

//************************************SLOTS******************************//
void StructScan::systemOnline()
{

}

void StructScan::systemOffline()
{

}

void StructScan::onActionProjectNewClicked()
{
	//QImage img("TestImage.bmp");
	//m_pimagewnd->setFrame(QPixmap::fromImage(img));

}

void StructScan::onActionProjectSaveClicked()
{

}

void StructScan::onActionProjectOpenFileClicked()
{
	emit signalOpenPCL();
}

void StructScan::onActionProjectOpenCalibrationClicked()
{
	if (!m_pcalibwnd) {
		m_pcalibwnd = new CCalibrationWnd;
		if (m_pcalibwnd == NULL) return;
		//ui.mdiArea->addSubWindow(m_pcalibwnd);
		m_pcalibwnd->show();

	}
}

void StructScan::onActionSetupSystemClicked()
{

}

void StructScan::onActionSetupCameraClicked()
{
	m_pcamera->openCamera();
	m_pcamera->setCameraTriggeModel(5.0);
}

void StructScan::onActionSetupProjectorClicked()
{
	m_pcamera->closeCamera();
}

void StructScan::onActionStartClicked()
{
	//m_dbroot->updateObject();
	m_pcamera->acquireImages();
}

void StructScan::onActionStopClicked()
{
	m_pcamera->stopAcquireImages();
}

void StructScan::onActionPointCloudSelectClicked()
{
	emit signalSelect();
}

void StructScan::onActionPointCloudDeleteClicked()
{
	emit signalDelete();
}

void StructScan::onActionPointCloudAddClicked()
{
	emit signalAdd();
}

void StructScan::onActionPointCloudFilterClicked()
{
	emit signalFilter();
}

void StructScan::onActionPointCloudMeshClicked()
{
	emit signalMesh();
}

void StructScan::onActionViewPlotClicked()
{

}

void StructScan::onActionView2DClicked()
{
	emit signalSurfaceRebuild();
}

void StructScan::onActionView3DClicked()
{
	bool statue = false;
	statue = !m_pcamera->getUserOutputStatue();

	m_pcamera->setUserOutput(0, statue);
}

void StructScan::onActionViewFrontClicked()
{

}

void StructScan::onActionViewBackClicked()
{

}

void StructScan::onActionViewLeftClicked()
{

}

void StructScan::onActionViewRightClicked()
{

}

void StructScan::onActionViewTopClicked()
{

}

void StructScan::onActionViewBottomClicked()
{

}

