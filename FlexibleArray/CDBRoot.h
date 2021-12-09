#pragma once


#include <QAbstractItemModel>
#include <QStandardItemModel>
#include <QTreeView>
#include <QHeaderView>
#include <QStyleFactory>


class CPropertyDelegate;
class CPointCloudWnd;


class CDBRoot :	public QWidget
{
	Q_OBJECT
public:
	CDBRoot(CPointCloudWnd *wnd, QTreeView* dbTreeWidget, QTreeView* propertiesTreeWidget, QWidget* parent = nullptr);

	~CDBRoot() override;

	QString m_qstrImgPath;
	QString m_qstrImgName;
	int *m_pOpeningAngle = nullptr;
	float *m_pSphereRadius = nullptr;
	float *m_pFactorCenterRound = nullptr;


	void updateObjectProperty(int objectCode);
	void updateObjectProperty(int objectCode, int imgIndex);
	void redrawObject();

private:
	void initModel();
	void initDelegate();
	void initView();
	void initOperate();

private:
	CPointCloudWnd* m_wnd;
	QTreeView* m_dbTreeWidget;
	QTreeView* m_propertiesTreeWidget;
	QStandardItemModel* m_propertiesModel;
	CPropertyDelegate* m_propertiesDelegate;

signals:
	void signalRefrushImgWnd();

private slots:
	void slotDBselected(QModelIndex modelIndex);

};

