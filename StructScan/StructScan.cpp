

#include "StructScan.h"
#include "CImageWnd.h"
#include "CCalibrationWnd.h"
#include "CPointCloudWnd.h"
#include "CDBRoot.h"



StructScan::StructScan(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

	m_pCloud = new CPointCloudWnd(ui.vtkOpenGLWidget, this);
	if (m_pCloud == nullptr)
		return;

	m_pimagewnd = new CImageWnd(this);
	if (m_pimagewnd == nullptr)
		return;
	m_pimagewnd->move(305, 80);




	//m_mdiArea = new QMdiArea(this);
	//if (m_mdiArea == nullptr) exit(1);
	//this->setCentralWidget(m_mdiArea);
	//���ڼ���ģʽ
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
	//���ÿ��
	//ui.dockWidget_Project->setMinimumWidth(width);

	//InitialTree();

}

// ��Ŀ��
void StructScan::InitialProjectTree()
{
	m_dbroot = new CDBRoot(m_pCloud, ui.treeViewProjects, ui.treeViewProperty, this);

	QStandardItemModel* model = new QStandardItemModel(ui.treeViewProjects);
	model->setHorizontalHeaderLabels(QStringList() << QStringLiteral("��Ŀ����") << QStringLiteral("��Ϣ"));
	QStandardItem* itemProject = new QStandardItem(QIcon(QStringLiteral("res/folder_image.ico")), QStringLiteral("��ά����"));
	model->appendRow(itemProject);
	QStandardItem *childItem1 = new QStandardItem(QStringLiteral("�豸"));
	//QStandardItem *childItem2 = new QStandardItem(QStringLiteral("ͼ��"));
	QStandardItem *childItem3 = new QStandardItem(QStringLiteral("����"));
	QStandardItem *childItem4 = new QStandardItem(QStringLiteral("����"));
	itemProject->appendRow(childItem1);
	//itemProject->appendRow(childItem2);
	itemProject->appendRow(childItem3);
	itemProject->appendRow(childItem4);
	//QStandardItem *image1 = new QStandardItem(QStringLiteral("ͼ��1"));
	//QStandardItem *image2 = new QStandardItem(QStringLiteral("ͼ��2"));
	//QStandardItem *image3 = new QStandardItem(QStringLiteral("ͼ��3"));
	//QStandardItem *image4 = new QStandardItem(QStringLiteral("ͼ��4"));
	//QStandardItem *image5 = new QStandardItem(QStringLiteral("ͼ��5"));
	//childItem2->appendRow(image1);
	//childItem2->appendRow(image2);
	//childItem2->appendRow(image3);
	//childItem2->appendRow(image4);
	//childItem2->appendRow(image5);

	//�������ߣ� ��QTreeViewӦ��model
	ui.treeViewProjects->setStyle(QStyleFactory::create("windows"));
	ui.treeViewProjects->setModel(model);
	ui.treeViewProjects->expandAll();
}

// ����
void StructScan::InitialPropertyTree()
{

}

//************************************SLOTS******************************//

void StructScan::onActionProjectNewClicked()
{
	QImage img("TestImage.bmp");
	m_pimagewnd->setFrame(QPixmap::fromImage(img));

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

}

void StructScan::onActionSetupProjectorClicked()
{

}

void StructScan::onActionStartClicked()
{
	//m_dbroot->updateObject();
}

void StructScan::onActionStopClicked()
{

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

