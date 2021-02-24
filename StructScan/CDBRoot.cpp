

#include "CDBRoot.h"

#include "CPropertyDelegate.h"
#include "CPointCloudWnd.h"


CDBRoot::CDBRoot(CPointCloudWnd *wnd, QTreeView* dbTreeWidget, QTreeView* propertiesTreeWidget, QWidget* parent)
	: QWidget(parent)
{
	// window
	assert(wnd);
	m_wnd = wnd;

	//DB Tree
	assert(dbTreeWidget);
	m_dbTreeWidget = dbTreeWidget;
	//m_dbTreeWidget->setModel(this);
	//m_dbTreeWidget->header()->hide();
	connect(m_dbTreeWidget, SIGNAL(pressed(QModelIndex)), this, SLOT(slotDBselected(QModelIndex)));


	//Properties Tree
	assert(propertiesTreeWidget);
	m_propertiesTreeWidget = propertiesTreeWidget;
	initModel();
	initDelegate();
	initView();
	initOperate();

}


CDBRoot::~CDBRoot()
{
	delete m_propertiesDelegate;
	delete m_propertiesModel;

}

// 选中DB 的某一行
void CDBRoot::slotDBselected(QModelIndex modelIndex)
{
	m_dbTreeWidget->resizeColumnToContents(modelIndex.row());
	QString selectedRowTxt = m_dbTreeWidget->model()->itemData(modelIndex).values()[0].toString();
	//qDebug() << "pressed result = " << selectedRowTxt;

	if (selectedRowTxt.compare(QStringLiteral("三维测量")) == 0) {
		updateObjectProperty(0);
	}
	else if (selectedRowTxt.compare(QStringLiteral("设备")) == 0) {
		updateObjectProperty(1);
	}
	else if (selectedRowTxt.compare(QStringLiteral("点云")) == 0) {
		updateObjectProperty(2);
	}
	else if (selectedRowTxt.compare(QStringLiteral("网格")) == 0) {
		updateObjectProperty(3);
	}


}

void CDBRoot::initModel()
{
	//QStandardItemModel类提供用于存储自定义数据的通用模型
	m_propertiesModel = new QStandardItemModel(0, 2, this);

	//const int col_count = 2;
	//m_propertiesModel->setColumnCount(col_count);
	m_propertiesModel->setHeaderData(0, Qt::Horizontal, QStringLiteral("属性"));
	m_propertiesModel->setHeaderData(1, Qt::Horizontal, QStringLiteral("值"));

	//view会根据model提供的数据来渲染
	m_propertiesTreeWidget->setModel(m_propertiesModel);
}


void CDBRoot::initDelegate()
{
	m_propertiesDelegate = new CPropertyDelegate(m_propertiesModel, m_propertiesTreeWidget);
	m_propertiesDelegate->m_wnd = this->m_wnd;
	m_propertiesTreeWidget->setItemDelegate(m_propertiesDelegate);

	//connect(m_propertiesDelegate, &CPropertyDelegate::signalObjectPropertiesChanged, this, &CDBRoot::updateObject);
	//connect(m_propertiesDelegate, &CPropertyDelegate::signalObjectAppearanceChanged, this, &CDBRoot::redrawObject);

}


void CDBRoot::initView()
{
	QTreeView* tree = m_propertiesTreeWidget;
	tree->header()->setSectionResizeMode(QHeaderView::Interactive);
	tree->setRootIsDecorated(false);
	//tree->setStyle(QStyleFactory::create("windows"));
	//tree->setStyleSheet("QTreeView::item{\
	//	margin:8px;}\
	//	QTreeView::branch{\
	//	image:none;\
	//	}");	   
}


void CDBRoot::initOperate()
{

}


void CDBRoot::updateObjectProperty(int objectCode)
{
	m_propertiesDelegate->updateModel(objectCode);


}


void CDBRoot::redrawObject()
{

}



