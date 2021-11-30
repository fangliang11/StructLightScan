#pragma once


#include <QDebug>
#include <QStyledItemDelegate>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QAbstractItemView>
#include <QTreeView>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QList>
#include <QLabel>
#include <QPalette>


class QAbstractItemView;
class QStandardItem;
class QStandardItemModel;
class CPointCloudWnd;

class CPropertyDelegate : public QStyledItemDelegate
{
	Q_OBJECT
public:
	explicit CPropertyDelegate(QStandardItemModel* _model, QAbstractItemView* _view, QObject *parent = nullptr);
	~CPropertyDelegate();

	CPointCloudWnd* m_wnd;
	QString m_qstrImgPath;

	void updateDisplay();
	void updateModel(int objectCode);
	void updateImgIndex(int imgIndex);
	void updateItem(QStandardItem * item);

	enum PROPERTY_CODE {
		OBJECT_NO_PROPERTY = 0,
		OBJECT_NAME,
		OBJECT_INFO,
		OBJECT_VISIBILITY,
		OBJECT_CLOUD_POINTS,
		OBJECT_CLOUD_POINT_SIZE,
		OBJECT_CLOUD_DISPLAY_COLOR,
		OBJECT_CLOUD_COORDINATE,
		OBJECT_FILTER_STATE,
		OBJECT_FILTER_METHOD,
		OBJECT_FILTER_PARAM1,
		OBJECT_FILTER_PARAM2,
		OBJECT_FILTER_PARAM3,
		OBJECT_REMOVE_OUTLIER_STATE,
		OBJECT_REMOVE_OUTLIER_PARAM1,
		OBJECT_REMOVE_OUTLIER_PARAM2,
		OBJECT_SMOOTH_STATE,
		OBJECT_SMOOTH_PARAM1,
		OBJECT_SMOOTH_PARAM2,
		OBJECT_REBUILD_METHOD,
		OBJECT_REBUILD_SEARCH_K,
		OBJECT_REBUILD_SEARCH_RADIUS,
		OBJECT_REBUILD_MAX_NEIGHBORS,
		OBJECT_MESH_NAME,
		OBJECT_MESH_DISPLAY_TYPE,
		OBJECT_MESH_DISPLAY_COLOR,
		TREE_VIEW_HEADER,
	};

	enum OBJECT_CODE{NONE = 0, DEVICE, CLOUD, MESH, IMAGE};

protected:
	//渲染相关的接口
	void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
	QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const override;
	//编辑相关的接口
	QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
	void setEditorData(QWidget *editor, const QModelIndex &index) const override;
	void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;
	void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
	   
private:
	QStandardItemModel* m_model;
	QAbstractItemView* m_view;
	int m_nCurrentObject;
	int m_nCloudPointSize;
	int m_nCloudColor;
	bool m_bCloudCoordinate;
	bool m_bFilterEnable;
	int m_nFilterMethod;
	float m_fFilterParam1;
	float m_fFilterParam2;
	float m_fFilterParam3;
	bool m_bremoveOutlierEnable;
	float m_fremoveOutlierParam1;
	float m_fremoveOutlierParam2;
	bool m_bsmoothEnable;
	float m_fsmoothParam1;
	float m_fsmoothParam2;
	int m_nrebuildMethod;
	int m_nmeshSearchK;
	float m_fmeshSearchRadius;
	int m_nmeshMaxNeighbors;
	int m_nmeshDisplayModel;
	int m_nmeshDisplayColor;
	int m_nImgIndex;
	   
	void addSeparator(const QString& title);
	void appendRow(QStandardItem* leftItem, QStandardItem* rightItem, bool openPersistentEditor/*=false*/);
	void clearModel();
	void fillModel();
	void fillModelWithObject();
	void fillModelWithCloud();
	void fillModelWithFiltered();
	void fillModelWithCloudRemoveOutlier();
	void fillModelWithCloudSmooth();
	void fillModelWithRebuild();
	void fillModelWithMesh();
	void fileModelWithImageInfo(int index);

signals:
	void signalObjectPropertiesChanged() const;
	void signalObjectAppearanceChanged() const;
	void signalObjectVisibleState(bool state);
	void signalCloudCoordinateDisplayState(bool state);
	void signalFilterEnable(bool state);
	void signalRemoveOutlierEnable(bool state);
	void signalSmoothEnable(bool state);

private slots:
	void objectVisible(bool state);
	void cloudPointSizeChanged(int index);
	void cloudColorChanged(int index);
	void cloudCoordinateDisplay(bool state);
	void filterEnable(bool state);
	void filterMethodChanged(int index);
	void filterParam1Changed(double value);
	void filterParam2Changed(double value);
	void filterParam3Changed(double value);
	void removeOutlierEnable(bool state);
	void removeOutlierParam1Changed(double value);
	void removeOutlierParam2Changed(double value);
	void smoothEnable(bool state);
	void smoothParam1Changed(double value);
	void smoothParam2Changed(double value);
	void meshBuildMethod(int index);
	void meshSearchKChanged(int value);
	void meshSearchRadiusChanged(float value);
	void meshMaxNeighbors(int value);
	void meshDisplayModel(int index);
	void meshColor(int index);

};

