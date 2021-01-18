

#include "StructScan.h"
#include "CImageWnd.h"
#include "CCalibrationWnd.h"
#include "CPointCloudWnd.h"



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


	//this->setCentralWidget(ui.mdiArea);


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
	
}

StructScan::~StructScan()
{

}


void StructScan::InitialConnection()
{
	connect(this, &StructScan::signalOpenPCL, m_pCloud, &CPointCloudWnd::signalOpenPCL);
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

void StructScan::InitialTree()
{
	//QStandardItemModel* model = new QStandardItemModel(ui.treeView);
	//QList<QStandardItem*> kind;
	//QStandardItem *kindname1 = new QStandardItem(QStringLiteral("��Ŀ"));
	//QStandardItem *kindname2 = new QStandardItem(QStringLiteral("���"));
	//kind.append(kindname1);
	//kind.append(kindname2);
	//model->appendColumn(kind);

	////�������ߣ� ��QTreeViewӦ��model
	//ui.treeView->setStyle(QStyleFactory::create("windows"));
	//ui.treeView->setModel(model);
}




//************************************SLOTS******************************//

void StructScan::onActionProjectNewClicked()
{
	qDebug() << "new clicked";

	//InitialConnection();
	QImage img("s_logo.png");
	m_pimagewnd->setFrame(QPixmap::fromImage(img));

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

