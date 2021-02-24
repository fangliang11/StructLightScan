
#include "CPropertyDelegate.h"
#include "CPointCloudWnd.h"

//System
#include <cassert>

// 全局变量及函数

// Default 'None' string
const char* CPropertyDelegate::s_noneString = QT_TR_NOOP("None");
// Default color sources string
const char* CPropertyDelegate::s_rgbColor = "RGB";
const char* CPropertyDelegate::s_sfColor = QT_TR_NOOP("Scalar field");
// Other strings
const char* CPropertyDelegate::s_defaultPointSizeString = QT_TR_NOOP("Default");
const char* CPropertyDelegate::s_defaultPolyWidthSizeString = QT_TR_NOOP("Default Width");
// Default separator colors
constexpr const char* SEPARATOR_STYLESHEET = "QLabel { background-color : darkGray; color : white; }";

//Shortcut to create a delegate item
static QStandardItem* ITEM(const QString& name, Qt::ItemFlag additionalFlags = Qt::NoItemFlags,
	CPropertyDelegate::PROPERTY_CODE role = CPropertyDelegate::OBJECT_NO_PROPERTY)
{
	QStandardItem* item = new QStandardItem(name);
	//flags
	item->setFlags(Qt::ItemIsEnabled | additionalFlags);
	//role (if any)
	if (role != CPropertyDelegate::OBJECT_NO_PROPERTY) {
		item->setData(role);
	}

	return item;
}

//Shortcut to create a checkable delegate item
static QStandardItem* CHECKABLE_ITEM(bool checkState, CPropertyDelegate::PROPERTY_CODE role)
{
	QStandardItem* item = ITEM("", Qt::ItemIsUserCheckable, role);
	//check state
	item->setCheckState(checkState ? Qt::Checked : Qt::Unchecked);

	return item;
}

//Shortcut to create a persistent editor item
static QStandardItem* PERSISTENT_EDITOR(CPropertyDelegate::PROPERTY_CODE role)
{
	return ITEM(QString(), Qt::ItemIsEditable, role);
}

//构造函数
CPropertyDelegate::CPropertyDelegate(QStandardItemModel* _model, QAbstractItemView* _view, QObject *parent)
	: m_model(_model), m_view(_view), QStyledItemDelegate(parent)
{
	assert(m_model && m_view);

	m_nCurrentObject = NONE;
	connect(this, &CPropertyDelegate::signalObjectVisibleState, this, &CPropertyDelegate::objectVisible);
	connect(this, &CPropertyDelegate::signalCloudCoordinateDisplayState, this, &CPropertyDelegate::cloudCoordinateDisplay);
	connect(this, &CPropertyDelegate::signalFilterEnable, this, &CPropertyDelegate::filterEnable);
	connect(this, &CPropertyDelegate::signalRemoveOutlierEnable, this, &CPropertyDelegate::removeOutlierEnable);
	connect(this, &CPropertyDelegate::signalSmoothEnable, this, &CPropertyDelegate::smoothEnable);

}

//析构函数
CPropertyDelegate::~CPropertyDelegate()
{
}

void CPropertyDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
	QStyledItemDelegate::paint(painter, option, index);
}

QSize CPropertyDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{
	return QStyledItemDelegate::sizeHint(option, index);
}

//用户编辑回调，处理 view
QWidget *CPropertyDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
	if (!m_model)
		return nullptr;
	QStandardItem* item = m_model->itemFromIndex(index);
	if (!item || !item->data().isValid())
		return nullptr;

	int itemData = item->data().toInt();
	if (item->column() == 0) {
		//on the first column, only editors spanning on 2 columns are allowed
		return nullptr;
	}

	QWidget* outputWidget = nullptr;
	switch (itemData)
	{
	case OBJECT_CLOUD_POINT_SIZE: {
		QComboBox *comboBox = new QComboBox(parent);
		//comboBox->addItem(tr(s_defaultPointSizeString));
		for (int i = 1; i <= 15; i++) {
			comboBox->addItem(QString::number(i));
		}
		//comboBox->setCurrentIndex(0);
		connect(comboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
			this, &CPropertyDelegate::cloudPointSizeChanged);
		//connect(comboBox, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),
		//	this, &CPropertyDelegate::cloudPointSizeChanged);
		//connect(comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int index) { /* ... */ });
		outputWidget = comboBox;
		break;
	}
	case OBJECT_CLOUD_DISPLAY_COLOR: {
		QComboBox *comboBox = new QComboBox(parent);
		comboBox->addItem(QString("Default"));
		comboBox->addItem(QString("Gray"));
		comboBox->addItem(QString("Blue"));
		comboBox->addItem(QString("White"));
		connect(comboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
			this, &CPropertyDelegate::cloudColorChanged);
		outputWidget = comboBox;
		break;
	}
	case OBJECT_FILTER_METHOD: {
		QComboBox *comboBox = new QComboBox(parent);
		comboBox->addItem(QStringLiteral("下采样"));
		comboBox->addItem(QStringLiteral("均匀采样"));
		comboBox->addItem(QStringLiteral("增采样"));
		connect(comboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
			this, &CPropertyDelegate::filterMethodChanged);
		outputWidget = comboBox;
		break;
	}
	case OBJECT_FILTER_PARAM1: {
		QDoubleSpinBox* spinBox = new QDoubleSpinBox(parent);
		spinBox->setDecimals(2);
		spinBox->setRange(0.0, 100.0e2);
		spinBox->setSingleStep(0.01);
		connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
			this, &CPropertyDelegate::filterParam1Changed);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_FILTER_PARAM2: {
		QDoubleSpinBox *spinBox = new QDoubleSpinBox(parent);
		spinBox->setDecimals(2);
		spinBox->setRange(0.01, 100.0e2);
		spinBox->setSingleStep(0.01);
		connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
			this, &CPropertyDelegate::filterParam2Changed);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_FILTER_PARAM3: {
		QDoubleSpinBox *spinBox = new QDoubleSpinBox(parent);
		spinBox->setDecimals(2);
		spinBox->setRange(0.01, 100.0e2);
		spinBox->setSingleStep(0.01);
		connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
			this, &CPropertyDelegate::filterParam3Changed);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REMOVE_OUTLIER_PARAM1: {
		QDoubleSpinBox *spinBox = new QDoubleSpinBox(parent);
		spinBox->setDecimals(2);
		spinBox->setRange(0.01, 100.0e2);
		spinBox->setSingleStep(0.01);
		connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
			this, &CPropertyDelegate::removeOutlierParam1Changed);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REMOVE_OUTLIER_PARAM2: {
		QDoubleSpinBox *spinBox = new QDoubleSpinBox(parent);
		spinBox->setDecimals(2);
		spinBox->setRange(0.01, 100.0e2);
		spinBox->setSingleStep(0.01);
		connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
			this, &CPropertyDelegate::removeOutlierParam2Changed);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_SMOOTH_PARAM1: {
		QDoubleSpinBox *spinBox = new QDoubleSpinBox(parent);
		spinBox->setDecimals(2);
		spinBox->setRange(0.01, 100.0e2);
		spinBox->setSingleStep(0.01);
		connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
			this, &CPropertyDelegate::smoothParam1Changed);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_SMOOTH_PARAM2: {
		QDoubleSpinBox *spinBox = new QDoubleSpinBox(parent);
		spinBox->setDecimals(2);
		spinBox->setRange(0.01, 100.0e2);
		spinBox->setSingleStep(0.01);
		connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
			this, &CPropertyDelegate::smoothParam2Changed);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REBUILD_METHOD: {
		QComboBox *comboBox = new QComboBox(parent);
		comboBox->addItem(tr("Default"));
		comboBox->addItem(QString("Method 1"));
		comboBox->addItem(QString("Method 2"));
		comboBox->addItem(QString("Method 3"));
		outputWidget = comboBox;
		break;
	}
	case OBJECT_REBUILD_NEAREST_K: {
		QSpinBox *spinBox = new QSpinBox(parent);
		spinBox->setRange(1, 1000);
		spinBox->setSingleStep(10);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REBUILD_SEARCH_RADIUS: {
		QDoubleSpinBox* spinBox = new QDoubleSpinBox(parent);
		spinBox->setDecimals(2);
		spinBox->setRange(0.0, 100.0e2);
		spinBox->setSingleStep(0.01);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REBUILD_DISTANCE_MU: {
		QDoubleSpinBox *spinBox = new QDoubleSpinBox(parent);
		spinBox->setDecimals(1);
		spinBox->setRange(0.1, 10.0);
		spinBox->setSingleStep(0.1);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REBUILD_MAX_NEAREST_NEIGHBOR: {
		QSpinBox *spinBox = new QSpinBox(parent);
		spinBox->setRange(1, 1000);
		spinBox->setSingleStep(10);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REBUILD_MAX_SURFACE_ANGLE: {
		QSpinBox *spinBox = new QSpinBox(parent);
		spinBox->setRange(0, 180);
		spinBox->setSingleStep(5);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REBUILD_MIN_ANGLE: {
		QSpinBox *spinBox = new QSpinBox(parent);
		spinBox->setRange(0, 180);
		spinBox->setSingleStep(5);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REBUILD_MAX_ANGLE: {
		QSpinBox *spinBox = new QSpinBox(parent);
		spinBox->setRange(0, 360);
		spinBox->setSingleStep(5);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REBUILD_NORMAL_CONSISTENCY: {
		QSpinBox *spinBox = new QSpinBox(parent);
		spinBox->setRange(0, 360);
		spinBox->setSingleStep(5);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_MESH_DISPLAY_TYPE: {
		QComboBox *comboBox = new QComboBox(parent);
		comboBox->addItem(tr(s_noneString));
		comboBox->addItem(QString("Surface"));
		comboBox->addItem(QString("WireFrame"));
		comboBox->addItem(QString("Point"));
		outputWidget = comboBox;
		break;
	}
	case OBJECT_MESH_DISPLAY_COLOR: {
		QComboBox *comboBox = new QComboBox(parent);
		comboBox->addItem(tr(s_noneString));
		comboBox->addItem(QString("Gray"));
		comboBox->addItem(QString("Blue"));
		comboBox->addItem(QString("White"));
		outputWidget = comboBox;
		break;
	}
	case TREE_VIEW_HEADER: {
		QLabel* headerLabel = new QLabel(parent);
		headerLabel->setStyleSheet(SEPARATOR_STYLESHEET);
		//QPalette palette;
		//palette.setColor(QPalette::Background, QColor(Qt::darkGray));
		//headerLabel->setAutoFillBackground(true);
		//headerLabel->setPalette(palette);

		//no signal connection, it's a display-only widget
		outputWidget = headerLabel;
		break;
	}
	default:break;
	}

	return outputWidget;
}

void SetDoubleSpinBoxValue(QWidget *editor, double value, bool keyboardTracking = false)
{
	QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(editor);
	if (!spinBox)
	{
		assert(false);
		return;
	}
	spinBox->setKeyboardTracking(keyboardTracking);
	spinBox->setValue(value);
}

void SetSpinBoxValue(QWidget *editor, int value, bool keyboardTracking = false)
{
	QSpinBox* spinBox = qobject_cast<QSpinBox*>(editor);
	if (!spinBox)
	{
		assert(false);
		return;
	}
	spinBox->setKeyboardTracking(keyboardTracking);
	spinBox->setValue(value);
}

void SetComboBoxIndex(QWidget *editor, int index)
{
	QComboBox* comboBox = qobject_cast<QComboBox*>(editor);
	if (!comboBox)
	{
		assert(false);
		return;
	}
	assert(index < 0 || index < comboBox->maxCount());
	comboBox->setCurrentIndex(index);
}

//用户触发编辑模式后回调，处理 view
void CPropertyDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
	if (!m_model)
		return;

	QStandardItem* item = m_model->itemFromIndex(index);
	if (!item || !item->data().isValid() || (item->column() == 0))
		return;

	switch (item->data().toInt())
	{
	case OBJECT_CLOUD_POINT_SIZE: {
		//ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_currentObject);
		//assert(cloud);
		//if (!cloud){
		//	return;
		//}
		//SetComboBoxIndex(editor, static_cast<int>(cloud->getPointSize()));
		SetComboBoxIndex(editor, static_cast<int>(0));
		break;
	}
	case OBJECT_CLOUD_DISPLAY_COLOR:
		SetComboBoxIndex(editor, static_cast<int>(0));
		break;
	case OBJECT_FILTER_METHOD:
		SetComboBoxIndex(editor, static_cast<int>(0));
		break;
	case OBJECT_FILTER_PARAM1:
		SetDoubleSpinBoxValue(editor, static_cast<double>(0.01));
		break;
	case OBJECT_FILTER_PARAM2:
		SetDoubleSpinBoxValue(editor, static_cast<double>(0.5));
		break;
	case OBJECT_FILTER_PARAM3:
		SetDoubleSpinBoxValue(editor, static_cast<double>(0.01));
		break;
	case OBJECT_REMOVE_OUTLIER_PARAM1:
		SetDoubleSpinBoxValue(editor, static_cast<double>(50.0));
		break;
	case OBJECT_REMOVE_OUTLIER_PARAM2:
		SetDoubleSpinBoxValue(editor, static_cast<double>(1.0));
		break;
	case OBJECT_SMOOTH_PARAM1:
		SetDoubleSpinBoxValue(editor, static_cast<double>(0.05));
		break;
	case OBJECT_SMOOTH_PARAM2:
		SetDoubleSpinBoxValue(editor, static_cast<double>(1.0));
		break;
	case OBJECT_REBUILD_METHOD:
		SetComboBoxIndex(editor, static_cast<int>(0));
		break;
	case OBJECT_REBUILD_NEAREST_K:
		SetSpinBoxValue(editor, static_cast<int>(20));
		break;
	case OBJECT_REBUILD_SEARCH_RADIUS:
		SetDoubleSpinBoxValue(editor, static_cast<double>(2.0));
		break;
	case OBJECT_REBUILD_DISTANCE_MU:
		SetDoubleSpinBoxValue(editor, static_cast<double>(2.5));
		break;
	case OBJECT_REBUILD_MAX_NEAREST_NEIGHBOR:
		SetSpinBoxValue(editor, static_cast<int>(100));
		break;
	case OBJECT_REBUILD_MAX_SURFACE_ANGLE:
		SetSpinBoxValue(editor, static_cast<int>(45));
		break;
	case OBJECT_REBUILD_MIN_ANGLE:
		SetSpinBoxValue(editor, static_cast<int>(10));
		break;
	case OBJECT_REBUILD_MAX_ANGLE:
		SetSpinBoxValue(editor, static_cast<int>(120));
		break;
	case OBJECT_MESH_DISPLAY_TYPE:
		SetComboBoxIndex(editor, static_cast<int>(0));
		break;
	case OBJECT_MESH_DISPLAY_COLOR:
		SetComboBoxIndex(editor, static_cast<int>(0));
		break;
	case TREE_VIEW_HEADER: {
		QLabel* label = qobject_cast<QLabel*>(editor);
		if (label) {
			label->setText(item->accessibleDescription());
		}
		break;
	}
	default:break;
	}
}

//用户操作完成后回调，处理 model
void CPropertyDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
	//if (index.isValid()){
	//	switch (index.column())
	//	{
	//	case 0:
	//		break;
	//	case 1: {
	//		QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
	//		model->setData(index, spinBox->value(), Qt::DisplayRole);
	//		break;
	//	}
	//	default:break;
	//	}
	//}
}

//用户触发编辑模式后自动回调
void CPropertyDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
	QStyledItemDelegate::updateEditorGeometry(editor, option, index);

	if (!m_model || !editor) {
		return;
	}

	QStandardItem* item = m_model->itemFromIndex(index);
	if (item &&	item->data().isValid() && item->column() == 0) {
		//if (isWideEditor(item->data().toInt())){
		//	QWidget* widget = qobject_cast<QWidget*>(editor);
		//	if (!widget){
		//		return;
		//	}
		//	//we must resize the SF edit widget so that it spans on both columns!
		//	QRect rect = m_view->visualRect(m_model->index(item->row(), 1)); //second column width
		//	widget->resize(option.rect.width() + rect.width(), widget->height());
		//}
	}
}

// 处理 checkbox
void CPropertyDelegate::updateItem(QStandardItem * item)
{
	if (item->column() == 0 || !item->data().isValid())
		return;

	bool redraw = false;
	switch (item->data().toInt())
	{
	case OBJECT_NAME:
		//m_currentObject->setName(item->text());
		//emit ccObjectPropertiesChanged(m_currentObject);
		break;
	case OBJECT_VISIBILITY:	{
		bool state = false;
		if (item->checkState() == Qt::Checked)
			state = true;
		else
			state = false;
		emit signalObjectVisibleState(state);
		break;
	}
	case OBJECT_CLOUD_COORDINATE: {
		bool state = false;
		if (item->checkState() == Qt::Checked)
			state = true;
		else
			state = false;
		emit signalCloudCoordinateDisplayState(state);
		break;
	}
	case OBJECT_FILTER_STATE: {
		bool state = false;
		if (item->checkState() == Qt::Checked)
			state = true;
		else
			state = false;
		emit signalFilterEnable(state);
		break;
	}	
	case OBJECT_REMOVE_OUTLIER_STATE: {
		bool state = false;
		if (item->checkState() == Qt::Checked)
			state = true;
		else
			state = false;
		emit signalRemoveOutlierEnable(state);
		break;
	}
	case OBJECT_SMOOTH_STATE: {
		bool state = false;
		if (item->checkState() == Qt::Checked)
			state = true;
		else
			state = false;
		emit signalSmoothEnable(state);
		break;
	}
	default:break;
	}

	if (redraw)	{
		updateDisplay();
	}
}


void CPropertyDelegate::updateDisplay()
{
	emit signalObjectPropertiesChanged();
}


void CPropertyDelegate::updateModel(int objectCode)
{
	m_nCurrentObject = objectCode;

	fillModel();
}


void CPropertyDelegate::addSeparator(const QString& title)
{
	if (m_model) {
		QStandardItem* leftItem = new QStandardItem(title);
		leftItem->setData(TREE_VIEW_HEADER);
		leftItem->setAccessibleDescription(title);
		leftItem->setBackground(QBrush(QColor(Qt::gray)));
		m_model->appendRow(leftItem);
		if (m_view != nullptr) {
			m_view->openPersistentEditor(m_model->index(m_model->rowCount() - 1, 0));
		}
	}
}


void CPropertyDelegate::appendRow(QStandardItem* leftItem, QStandardItem* rightItem, bool openPersistentEditor = false)
{
	assert(leftItem && rightItem);
	assert(m_model);

	if (m_model) {
		//append row
		QList<QStandardItem*> rowItems{ leftItem, rightItem };
		m_model->appendRow(rowItems);

		//the persistent editor (if any) is always the right one!
		if (openPersistentEditor && (m_view != nullptr)) {
			m_view->openPersistentEditor(m_model->index(m_model->rowCount() - 1, 1));
		}
	}
}

void CPropertyDelegate::clearModel()
{
	assert(m_model);

	//m_model->clear();
	for (;;) {
		if (m_model->rowCount() > 0)
			m_model->removeRow(0);
		else
			break;
	}
}

void CPropertyDelegate::fillModel()
{
	assert(m_model);

	clearModel();
	switch (m_nCurrentObject)
	{
	case NONE:
		break;
	case DEVICE:
		fillModelWithObject();
		break;
	case CLOUD:
		fillModelWithCloud();
		fillModelWithFiltered();
		fillModelWithCloudRemoveOutlier();
		fillModelWithCloudSmooth();
		break;
	case MESH:
		fillModelWithRebuild();
		fillModelWithMesh();
		break;
	default:break;
	}

	if (m_model) {
		connect(m_model, &QStandardItemModel::itemChanged, this, &CPropertyDelegate::updateItem);
	}
}

void CPropertyDelegate::fillModelWithObject()
{
	assert(m_model);

	addSeparator(QStringLiteral("设备"));

	appendRow(ITEM(tr("Name")), ITEM(tr("test object name"), Qt::ItemIsEditable, OBJECT_NAME));

	appendRow(ITEM(tr("Info")), ITEM(tr("Information of object"), Qt::ItemIsEditable, OBJECT_INFO));

	//appendRow(ITEM(tr("Visible")), PERSISTENT_EDITOR(OBJECT_VISIBILITY), false);
	appendRow(ITEM(tr("Visible")), CHECKABLE_ITEM(true, OBJECT_VISIBILITY));
}

void CPropertyDelegate::fillModelWithCloud()
{
	assert(m_model);

	addSeparator(QStringLiteral("点云"));

	unsigned number = 0;
	assert(m_wnd);
	m_wnd->getCloudPointNumbers(number);
	appendRow(ITEM(tr("Points")), ITEM(QString::number(number), Qt::ItemIsEditable, OBJECT_CLOUD_POINTS));

	appendRow(ITEM(tr("Point size")), PERSISTENT_EDITOR(OBJECT_CLOUD_POINT_SIZE), true);

	appendRow(ITEM(tr("Color")), PERSISTENT_EDITOR(OBJECT_CLOUD_DISPLAY_COLOR), true);

	appendRow(ITEM(tr("Coordinate")), CHECKABLE_ITEM(false, OBJECT_CLOUD_COORDINATE));
}

void CPropertyDelegate::fillModelWithFiltered()
{
	assert(m_model);

	addSeparator(QStringLiteral("点云滤波"));

	appendRow(ITEM(tr("Enable")), CHECKABLE_ITEM(false, OBJECT_FILTER_STATE));

	appendRow(ITEM(tr("Method")), PERSISTENT_EDITOR(OBJECT_FILTER_METHOD), true);

	appendRow(ITEM(tr("Parament1")), PERSISTENT_EDITOR(OBJECT_FILTER_PARAM1), true);

	appendRow(ITEM(tr("Parament2")), PERSISTENT_EDITOR(OBJECT_FILTER_PARAM2), true);

	appendRow(ITEM(tr("Parament3")), PERSISTENT_EDITOR(OBJECT_FILTER_PARAM3), true);
}

void CPropertyDelegate::fillModelWithCloudRemoveOutlier()
{
	assert(m_model);
	addSeparator(QStringLiteral("点云移除离群点"));
	appendRow(ITEM(tr("Enable")), CHECKABLE_ITEM(false, OBJECT_REMOVE_OUTLIER_STATE));
	appendRow(ITEM(tr("Mean K")), PERSISTENT_EDITOR(OBJECT_REMOVE_OUTLIER_PARAM1), true);
	appendRow(ITEM(tr("Standard Deviation")), PERSISTENT_EDITOR(OBJECT_REMOVE_OUTLIER_PARAM2), true);

}

void CPropertyDelegate::fillModelWithCloudSmooth()
{
	assert(m_model);
	addSeparator(QStringLiteral("点云平滑"));
	appendRow(ITEM(tr("Enable")), CHECKABLE_ITEM(false, OBJECT_SMOOTH_STATE));
	appendRow(ITEM(tr("Search Radius")), PERSISTENT_EDITOR(OBJECT_SMOOTH_PARAM1), true);
	appendRow(ITEM(tr("Parament2")), PERSISTENT_EDITOR(OBJECT_SMOOTH_PARAM2), true);

}


void CPropertyDelegate::fillModelWithRebuild()
{
	assert(m_model);

	addSeparator(QStringLiteral("三维重建"));

	appendRow(ITEM(tr("Method")), PERSISTENT_EDITOR(OBJECT_REBUILD_METHOD), true);

	appendRow(ITEM(tr("Nearest K")), PERSISTENT_EDITOR(OBJECT_REBUILD_NEAREST_K), false);

	appendRow(ITEM(tr("Search Radius")), PERSISTENT_EDITOR(OBJECT_REBUILD_SEARCH_RADIUS), false);

	appendRow(ITEM(tr("Distance Mu")), PERSISTENT_EDITOR(OBJECT_REBUILD_DISTANCE_MU), false);

	appendRow(ITEM(tr("Max NearestNeighbor")), PERSISTENT_EDITOR(OBJECT_REBUILD_MAX_NEAREST_NEIGHBOR), false);

	appendRow(ITEM(tr("Max Surface angle")), PERSISTENT_EDITOR(OBJECT_REBUILD_MAX_SURFACE_ANGLE), false);

	appendRow(ITEM(tr("Min angle")), PERSISTENT_EDITOR(OBJECT_REBUILD_MIN_ANGLE), false);

	appendRow(ITEM(tr("Max angle")), PERSISTENT_EDITOR(OBJECT_REBUILD_MAX_ANGLE), false);


	appendRow(ITEM(tr("Normal Consistency")), CHECKABLE_ITEM(false, OBJECT_REBUILD_NORMAL_CONSISTENCY));

}

void CPropertyDelegate::fillModelWithMesh()
{
	assert(m_model);

	addSeparator(QStringLiteral("网格"));

	appendRow(ITEM(tr("Name")), ITEM(tr("mesh name"), Qt::ItemIsEditable, OBJECT_MESH_NAME));

	appendRow(ITEM(tr("Display")), PERSISTENT_EDITOR(OBJECT_MESH_DISPLAY_TYPE), true);

	appendRow(ITEM(tr("Color")), PERSISTENT_EDITOR(OBJECT_MESH_DISPLAY_COLOR), true);

}

//*****************************************************SLOT******************************//
void CPropertyDelegate::objectVisible(bool state)
{

}

void CPropertyDelegate::cloudPointSizeChanged(int index)
{
	//qDebug() << "pointsize change to " << index + 1;
	if (m_nCurrentObject == NONE)
		return;

	double point_size = 0.0;
	m_wnd->getCloudPointSize(point_size);
	if (point_size != 0 && (int)point_size != index + 1) {
		m_wnd->setCloudPointSize(index + 1);
	}

}

void CPropertyDelegate::cloudColorChanged(int index)
{
	if (m_nCurrentObject == NONE)
		return;

	assert(m_wnd);
	m_wnd->setCloudPointColor(index);
}

void CPropertyDelegate::cloudCoordinateDisplay(bool state)
{
	if (m_nCurrentObject == NONE)
		return;
	m_wnd->displayCoordinate(state);
}

void CPropertyDelegate::filterEnable(bool state)
{
	if (m_nCurrentObject == NONE)
		return;
	m_wnd->filterEnable(state);
}

void CPropertyDelegate::filterMethodChanged(int index)
{
	qDebug() << "filter method = " << index;
	if (m_nCurrentObject == NONE)
		return;
	m_wnd->filterMethod(index);
}

void CPropertyDelegate::filterParam1Changed(double value)
{
	qDebug() << "filter param1 = " << value;
	if (m_nCurrentObject == NONE)
		return;

	float param1 = 0.0f, param2 = 0.0f, param3 = 0.0f;
	m_wnd->getFilterParams(param1, param2, param3);
	if (value != (double)param1)
		m_wnd->setFilterParams((float)value, param2, param3);
}

void CPropertyDelegate::filterParam2Changed(double value)
{
	qDebug() << "filter param2 = " << value;
	if (m_nCurrentObject == NONE)
		return;

	float param1 = 0.0f, param2 = 0.0f, param3 = 0.0f;
	m_wnd->getFilterParams(param1, param2, param3);
	if (value != (double)param2)
		m_wnd->setFilterParams(param1, (float)value, param3);
}

void CPropertyDelegate::filterParam3Changed(double value)
{
	qDebug() << "filter param3 = " << value;
	if (m_nCurrentObject == NONE)
		return;

	float param1 = 0.0f, param2 = 0.0f, param3 = 0.0f;
	m_wnd->getFilterParams(param1, param2, param3);
	if (value != (double)param3)
		m_wnd->setFilterParams(param1, param2, (float)value);
}

void CPropertyDelegate::removeOutlierEnable(bool state)
{
	qDebug() << "remove Outlier enable = " << state;
	if (m_nCurrentObject == NONE)
		return;
	m_wnd->removeOutlierEnable(state);
}

void CPropertyDelegate::removeOutlierParam1Changed(double value)
{
	qDebug() << "remove Outlier param1 = " << value;
	if (m_nCurrentObject == NONE)
		return;
	float param1 = 0.0f, param2 = 0.0f;
	m_wnd->getRemoveOutlierParams(param1, param2);
	if (value != (double)param1)
		m_wnd->setRemoveOutlierParams((float)value, param2);
}

void CPropertyDelegate::removeOutlierParam2Changed(double value) 
{
	qDebug() << "remove Outlier param2 = " << value;
	if (m_nCurrentObject == NONE)
		return;

	float param1 = 0.0f, param2 = 0.0f;
	m_wnd->getRemoveOutlierParams(param1, param2);
	if (value != (double)param2)
		m_wnd->setRemoveOutlierParams(param1, (float)value);
}

void CPropertyDelegate::smoothEnable(bool state) 
{
	qDebug() << "smooth enable = " << state;
	if (m_nCurrentObject == NONE)
		return;
	m_wnd->smoothEnable(state);
}

void CPropertyDelegate::smoothParam1Changed(double value) 
{
	qDebug() << "smooth param1 = " << value;
	if (m_nCurrentObject == NONE)
		return;

	float param1 = 0.0f, param2 = 0.0f;
	m_wnd->getSmoothParams(param1, param2);
	if (value != (double)param1)
		m_wnd->setSmoothParams((float)value, param2);
}

void CPropertyDelegate::smoothParam2Changed(double value)
{
	qDebug() << "smooth param2 = " << value;
	if (m_nCurrentObject == NONE)
		return;

	float param1 = 0.0f, param2 = 0.0f;
	m_wnd->getSmoothParams(param1, param2);
	if (value != (double)param2)
		m_wnd->setSmoothParams(param1, (float)value);
}

