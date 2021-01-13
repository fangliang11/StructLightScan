

#include "StructScan.h"
#include "CImageWnd.h"
#include "CCalibrationWnd.h"
#include "CPointCloudWnd.h"



StructScan::StructScan(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

	this->setCentralWidget(ui.mdiArea);


	//m_mdiArea = new QMdiArea(this);
	//if (m_mdiArea == nullptr) exit(1);
	//this->setCentralWidget(m_mdiArea);
	//窗口级联模式
	//m_mdiArea->cascadeSubWindows();
	   	 
	CreatePointCloudWnd();
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
	
}

StructScan::~StructScan()
{

}


void StructScan::InitialConnection()
{
	connect(this, &StructScan::signalOpenPCL, m_pcloudwnd, &CPointCloudWnd::signalOpenPCL);
}

//创建新的点云显示窗口
void StructScan::CreatePointCloudWnd()
{
	m_pcloudwnd = new CPointCloudWnd();
	if (m_pcloudwnd == nullptr)
		return;

	m_pcloudwnd->setWindowTitle(QStringLiteral("点云"));
	int wid = this->width() - ui.dockProjectList->width();
	int hei = this->height() - ui.mainToolBar->height() - ui.menuBar->height() - ui.statusBar->height();
	m_pcloudwnd->setFixedWidth((int)wid * 2 / 3);
	m_pcloudwnd->setFixedHeight(hei);

	ui.mdiArea->addSubWindow(m_pcloudwnd);
	ui.mdiArea->setAttribute(Qt::WA_DeleteOnClose);
	ui.mdiArea->show();
	//m_mdiArea->activeSubWindow();

}


void StructScan::CreateCameraDisplayWnd()
{
	m_pimagewnd1 = new CImageWnd(this);
	if (m_pimagewnd1 == nullptr)
		return;
	m_pimagewnd1->setWindowTitle(QStringLiteral("相机"));

	ui.mdiArea->addSubWindow(m_pimagewnd1);
	ui.mdiArea->setAttribute(Qt::WA_DeleteOnClose);
	ui.mdiArea->show();

	m_pimagewnd2 = new CImageWnd(this);
	if (m_pimagewnd2 == nullptr)
		return;
	m_pimagewnd2->setWindowTitle(QStringLiteral("投影"));

	ui.mdiArea->addSubWindow(m_pimagewnd2);
	ui.mdiArea->setAttribute(Qt::WA_DeleteOnClose);
	ui.mdiArea->show();

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

void StructScan::InitialTree()
{
	//QStandardItemModel* model = new QStandardItemModel(ui.treeView);
	//QList<QStandardItem*> kind;
	//QStandardItem *kindname1 = new QStandardItem(QStringLiteral("项目"));
	//QStandardItem *kindname2 = new QStandardItem(QStringLiteral("结果"));
	//kind.append(kindname1);
	//kind.append(kindname2);
	//model->appendColumn(kind);

	////设置虚线， 给QTreeView应用model
	//ui.treeView->setStyle(QStyleFactory::create("windows"));
	//ui.treeView->setModel(model);
}




//************************************SLOTS******************************//

void StructScan::onActionProjectNewClicked()
{
	qDebug() << "new clicked";

	//InitialConnection();

}

void StructScan::onActionProjectSaveClicked()
{
	qDebug() << "save clicked";

	emit signalOpenPCL();

}

void StructScan::onActionProjectOpenFileClicked()
{

}

void StructScan::onActionProjectOpenCalibrationClicked()
{
	if (!m_pcalibwnd) {
		m_pcalibwnd = new CCalibrationWnd;
		if (m_pcalibwnd == NULL) return;
		ui.mdiArea->addSubWindow(m_pcalibwnd);
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

}

void StructScan::onActionStopClicked()
{

}

void StructScan::onActionPointCloudSelectClicked()
{

}

void StructScan::onActionPointCloudDeleteClicked()
{

}

void StructScan::onActionPointCloudFilterClicked()
{

}

void StructScan::onActionPointCloudGridClicked()
{

}

void StructScan::onActionViewPlotClicked()
{

}

void StructScan::onActionView2DClicked()
{

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

