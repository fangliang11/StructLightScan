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
		OBJECT_REBUILD_NEAREST_K,
		OBJECT_REBUILD_SEARCH_RADIUS,
		OBJECT_REBUILD_DISTANCE_MU,
		OBJECT_REBUILD_MAX_NEAREST_NEIGHBOR,
		OBJECT_REBUILD_MAX_SURFACE_ANGLE,
		OBJECT_REBUILD_MIN_ANGLE,
		OBJECT_REBUILD_MAX_ANGLE,
		OBJECT_REBUILD_NORMAL_CONSISTENCY,
		OBJECT_MESH_NAME,
		OBJECT_MESH_DISPLAY_TYPE,
		OBJECT_MESH_DISPLAY_COLOR,
		TREE_VIEW_HEADER,
	};

	enum OBJECT_CODE{NONE = 0, DEVICE, CLOUD, MESH};

protected:
	//渲染相关的接口
	void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
	QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const override;
	//编辑相关的接口
	QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
	void setEditorData(QWidget *editor, const QModelIndex &index) const override;
	void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;
	void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const override;


public:
	CPointCloudWnd* m_wnd;

	void updateDisplay();
	void updateModel(int objectCode);
	void updateItem(QStandardItem * item);


private:
	QStandardItemModel* m_model;
	QAbstractItemView* m_view;
	int m_nCurrentObject;


	static const char* s_noneString;
	static const char* s_rgbColor;
	static const char* s_sfColor;
	static const char* s_defaultPointSizeString;
	static const char* s_defaultPolyWidthSizeString;
	   
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

};

