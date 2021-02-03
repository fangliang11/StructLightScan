#pragma once


#include <QAbstractItemModel>
#include <QStandardItemModel>
#include <QTreeView>
#include <QHeaderView>
#include <QStyleFactory>


class CPropertyDelegate;


class CDBRoot :	public QWidget
{
	Q_OBJECT
public:
	CDBRoot(QTreeView* dbTreeWidget, QTreeView* propertiesTreeWidget, QWidget* parent = nullptr);

	~CDBRoot() override;


	void updateObject();
	void redrawObject();

private:
	void initModel();
	void initDelegate();
	void initView();
	void initOperate();

private:

	QTreeView* m_dbTreeWidget;
	QTreeView* m_propertiesTreeWidget;
	QStandardItemModel* m_propertiesModel;
	CPropertyDelegate* m_propertiesDelegate;



};

