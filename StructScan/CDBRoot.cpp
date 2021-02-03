

#include "CDBRoot.h"

#include "CPropertyDelegate.h"


CDBRoot::CDBRoot(QTreeView* dbTreeWidget, QTreeView* propertiesTreeWidget, QWidget* parent)
	: QWidget(parent)
{
	//DB Tree
	assert(dbTreeWidget);
	m_dbTreeWidget = dbTreeWidget;
	//m_dbTreeWidget->setModel(this);
	//m_dbTreeWidget->header()->hide();


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


void CDBRoot::initModel()
{
	//QStandardItemModel类提供用于存储自定义数据的通用模型
	m_propertiesModel = new QStandardItemModel(this);

	const int col_count = 2;
	m_propertiesModel->setColumnCount(col_count);
	m_propertiesModel->setHeaderData(0, Qt::Horizontal, tr("Property"));
	m_propertiesModel->setHeaderData(1, Qt::Horizontal, tr("State/Value"));

	//const int row_count = 5;
	//m_propertiesModel->setRowCount(row_count);
	//for (int row = 0; row < row_count; row++) {
	//	for (int col = 0; col < col_count; col++) {
	//		QStandardItem *item = new QStandardItem;
	//		switch (col)
	//		{
	//		case 0:
	//			item->setData(QString("NO:%1").arg(row + 1), Qt::DisplayRole);
	//			break;
	//		case 1:
	//			item->setData(row * 3.1415926, Qt::DisplayRole);
	//			break;
	//		}
	//		//m_propertiesModel->appendRow(item);
	//		m_propertiesModel->setItem(row, col, item);
	//	}
	//}

	//view会根据model提供的数据来渲染
	m_propertiesTreeWidget->setModel(m_propertiesModel);
}


void CDBRoot::initDelegate()
{
	m_propertiesDelegate = new CPropertyDelegate(m_propertiesModel, m_propertiesTreeWidget);
	m_propertiesTreeWidget->setItemDelegate(m_propertiesDelegate);

	connect(m_propertiesDelegate, &CPropertyDelegate::signalObjectPropertiesChanged, this, &CDBRoot::updateObject);
	connect(m_propertiesDelegate, &CPropertyDelegate::signalObjectAppearanceChanged, this, &CDBRoot::redrawObject);

}


void CDBRoot::initView()
{
	QTreeView* tree = m_propertiesTreeWidget;
	tree->header()->setSectionResizeMode(QHeaderView::Interactive);
	//tree->setEnabled(false);
	tree->setFocusPolicy(Qt::NoFocus);
	tree->setRootIsDecorated(false);
	tree->setStyle(QStyleFactory::create("windows"));
	tree->setStyleSheet("QTreeView::item{border:1px solid black;}QTreeView::branch{image:none;}");
	   
}


void CDBRoot::initOperate()
{

}


void CDBRoot::updateObject()
{
	m_propertiesDelegate->updateModel();

}


void CDBRoot::redrawObject()
{

}



