

#include "FlexibleArray.h"
#include "CImageWnd.h"
#include "CCalibrationWnd.h"
#include "CPointCloudWnd.h"
#include "CDBRoot.h"
#include "CCameraControl.h"

#include "CPhaseCaculate.h"

FlexibleArray::FlexibleArray(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

	m_pCloud = new CPointCloudWnd(ui.vtkOpenGLWidget, this);
	if (m_pCloud == nullptr)
		return;

	// ͼ����ʾ����
	m_pimagewnd = new CImageWnd(this);
	if (m_pimagewnd == nullptr)
		return;
	m_pimagewnd->move(305, 80);

	// �������
	HWND hnd = (HWND)m_pimagewnd->winId();
	m_pcamera = new CCameraControl(hnd);
	if (m_pcamera == nullptr)
		return;
	if (m_pcamera->initialThisClass() != 0) {
		QMessageBox msg(QMessageBox::Warning, "Information", "δ�����豸!", QMessageBox::Ok);

		//exit(-1);
	}



	// ��λ����
	m_pcaculate = new CPhaseCaculate();
	if (m_pcaculate == nullptr)
		return;



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
	
	InitialStatusBar();
	InitialDockWidget(300);
	InitialProjectTree();
	InitialPropertyTree();
	InitialConnection();

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
	QLabel* label1 = new QLabel(tr("Label1"), this);
	QLabel* label2 = new QLabel(tr("Label2"), this);
	QLabel* label3 = new QLabel(tr("Label3"), this);
	QLabel* label4 = new QLabel(tr("Label4"), this);
	ui.statusBar->addWidget(label1, 1);
	ui.statusBar->addWidget(label2, 1);
	ui.statusBar->addWidget(label3, 1);
	ui.statusBar->addWidget(label4, 1);
	
}


void FlexibleArray::InitialDockWidget(int width)
{
	//���ÿ��
	//ui.dockWidget_Project->setMinimumWidth(width);

	//InitialTree();

}

// ��Ŀ��
void FlexibleArray::InitialProjectTree()
{
	m_dbroot = new CDBRoot(m_pCloud, ui.treeViewProjects, ui.treeViewProperty, this);

	QStandardItemModel* model = new QStandardItemModel(ui.treeViewProjects);
	model->setHorizontalHeaderLabels(QStringList() << QStringLiteral("��Ŀ����") << QStringLiteral("��Ϣ"));
	QStandardItem* itemProject = new QStandardItem(QIcon(QStringLiteral("res/folder_image.ico")), QStringLiteral("ȫ������"));
	model->appendRow(itemProject);
	QStandardItem *childItem1 = new QStandardItem(QStringLiteral("�豸"));
	QStandardItem *childItem2 = new QStandardItem(QStringLiteral("ͼ��"));
	QStandardItem *childItem3 = new QStandardItem(QStringLiteral("����"));
	QStandardItem *childItem4 = new QStandardItem(QStringLiteral("����"));
	itemProject->appendRow(childItem1);
	itemProject->appendRow(childItem2);
	itemProject->appendRow(childItem3);
	itemProject->appendRow(childItem4);
	QStandardItem *image1 = new QStandardItem(QStringLiteral("ͼ��1"));
	QStandardItem *image2 = new QStandardItem(QStringLiteral("ͼ��2"));
	QStandardItem *image3 = new QStandardItem(QStringLiteral("ͼ��3"));
	QStandardItem *image4 = new QStandardItem(QStringLiteral("ͼ��4"));
	QStandardItem *image5 = new QStandardItem(QStringLiteral("ͼ��5"));
	QStandardItem *image6 = new QStandardItem(QStringLiteral("ͼ��6"));
	QStandardItem *image7 = new QStandardItem(QStringLiteral("ͼ��7"));
	childItem2->appendRow(image1);
	childItem2->appendRow(image2);
	childItem2->appendRow(image3);
	childItem2->appendRow(image4);
	childItem2->appendRow(image5);
	childItem2->appendRow(image6);
	childItem2->appendRow(image7);

	//�������ߣ� ��QTreeViewӦ��model
	ui.treeViewProjects->setStyle(QStyleFactory::create("windows"));
	ui.treeViewProjects->setModel(model);
	ui.treeViewProjects->expandAll();
}

// ����
void FlexibleArray::InitialPropertyTree()
{

}

//************************************SLOTS******************************//
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

}

void FlexibleArray::onActionProjectOpenFileClicked()
{
	emit signalOpenPCL();
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

	QString imgPath = QFileDialog::getExistingDirectory(this, QStringLiteral("ѡ��ͼ��Ŀ¼"), "./");
	if (!imgPath.isEmpty()) {
		m_dbroot->m_qstrImgPath = imgPath + "/";

		//QByteArray cdata = imgPath.toLocal8Bit();
		//m_pcdPath = std::string(cdata);
		//m_actionCode = ACTION_OPEN;
	}

}

void FlexibleArray::onActionSetupProjectorClicked()
{
	m_pcamera->closeCamera();
}

void FlexibleArray::onActionStartClicked()
{
	//m_dbroot->updateObject();
	m_pcamera->acquireImages();
}

void FlexibleArray::onActionStopClicked()
{
	m_pcamera->stopAcquireImages();
}

void FlexibleArray::onActionPointCloudSelectClicked()
{
	emit signalSelect();
}

void FlexibleArray::onActionPointCloudDeleteClicked()
{
	emit signalDelete();
}

void FlexibleArray::onActionPointCloudAddClicked()
{
	emit signalAdd();
}

void FlexibleArray::onActionPointCloudFilterClicked()
{
	emit signalFilter();
}

void FlexibleArray::onActionPointCloudMeshClicked()
{
	emit signalMesh();
}

void FlexibleArray::onActionViewPlotClicked()
{

}

void FlexibleArray::onActionView2DClicked()
{
	emit signalSurfaceRebuild();
}

void FlexibleArray::onActionView3DClicked()
{
	bool statue = false;
	statue = !m_pcamera->getUserOutputStatue();

	m_pcamera->setUserOutput(0, statue);
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
	m_pimagewnd->setFrame(QPixmap::fromImage(img));

}
