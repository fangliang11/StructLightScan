#pragma once


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

class QAbstractItemView;
class QStandardItem;
class QStandardItemModel;

class CPropertyDelegate : public QStyledItemDelegate
{
	Q_OBJECT
public:
	explicit CPropertyDelegate(QStandardItemModel* _model, QAbstractItemView* _view, QObject *parent = nullptr);
	~CPropertyDelegate();


	enum PROPERTY_CODE {
		OBJECT_NO_PROPERTY = 0,
		OBJECT_NAME,
		OBJECT_VISIBILITY,
		OBJECT_CLOUD_NAME,
		OBJECT_CLOUD_POINT_SIZE,
		OBJECT_CLOUD_DISPLAY_COLOR,
		OBJECT_CLOUD_COORDINATE,
		OBJECT_FILTER_METHOD,
		OBJECT_FILTER_SEARCH_RADIUS,
		OBJECT_FILTER_LEAF_SIZE,
		OBJECT_FILTER_MEAN_K,
		OBJECT_FILTER_MULTY_THRESHOLD,
		OBJECT_FILTER_POLYNOMIAL_FIT,
		OBJECT_REBUILD_METHOD,
		OBJECT_REBUILD_NEAREST_K,
		OBJECT_REBUILD_SEARCH_RADIUS,
		OBJECT_REBUILD_DISTANCE_MU,
		OBJECT_REBUILD_MAX_NEAREST_NEIGHBOR,
		OBJECT_REBUILD_MAX_SURFACE_ANGLE,
		OBJECT_REBUILD_MIN_ANGLE,
		OBJECT_REBUILD_NORMAL_CONSISTENCY,
		OBJECT_MESH_NAME,
		OBJECT_MESH_DISPLAY_TYPE,
		OBJECT_MESH_DISPLAY_COLOR,

	};

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
	void updateDisplay();
	void updateModel();
	void updateItem();

signals:
	void signalObjectPropertiesChanged() const;
	void signalObjectAppearanceChanged() const;
private slots:
	void cloudPointSizeChanged();


private:
	QStandardItemModel* m_model;
	QAbstractItemView* m_view;

	static const char* s_noneString;
	static const char* s_rgbColor;
	static const char* s_sfColor;
	static const char* s_defaultPointSizeString;
	static const char* s_defaultPolyWidthSizeString;



	void initial();
	void addSeparator(const QString& title);
	void appendRow(QStandardItem* leftItem, QStandardItem* rightItem, bool openPersistentEditor/*=false*/);
	void fillModel();
	void fillModelWithCloud();
	void fillModelWithFiltered();
	void fileModelWithRebuild();
	void fileModelWithMesh();
	void fileModelWithSelect();


};

