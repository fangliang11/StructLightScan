
#include "CPropertyDelegate.h"
#include "CPointCloudWnd.h"

//System
#include <cassert>

constexpr const char* SEPARATOR_STYLESHEET = "QLabel { background-color : darkGray; color : white; }";

static QStandardItem* ITEM(const QString& name, Qt::ItemFlag additionalFlags = Qt::NoItemFlags,
	CPropertyDelegate::PROPERTY_CODE role = CPropertyDelegate::OBJECT_NO_PROPERTY)
{
	QStandardItem* item = new QStandardItem(name);
	item->setFlags(Qt::ItemIsEnabled | additionalFlags);
	if (role != CPropertyDelegate::OBJECT_NO_PROPERTY) {
		item->setData(role);
	}

	return item;
}

static QStandardItem* CHECKABLE_ITEM(bool checkState, CPropertyDelegate::PROPERTY_CODE role)
{
	QStandardItem* item = ITEM("", Qt::ItemIsUserCheckable, role);
	item->setCheckState(checkState ? Qt::Checked : Qt::Unchecked);

	return item;
}

static QStandardItem* PERSISTENT_EDITOR(CPropertyDelegate::PROPERTY_CODE role)
{
	return ITEM(QString(), Qt::ItemIsEditable, role);
}

//构造函数
CPropertyDelegate::CPropertyDelegate(QStandardItemModel* _model, QAbstractItemView* _view, QObject *parent)
	: m_model(_model), m_view(_view), QStyledItemDelegate(parent)
{
	assert(m_model && m_view);

	m_qstrImgPath = "";

	m_nCurrentObject = NONE;
	m_nCloudPointSize = 0;
	m_nCloudColor = 0;
	m_bCloudCoordinate = false;
	m_bFilterEnable = false;
	m_nFilterMethod = 0;
	m_fFilterParam1 = 0.01f;
	m_fFilterParam2 = 0.01f;
	m_fFilterParam3 = 0.01f;
	m_bremoveOutlierEnable = false;
	m_fremoveOutlierParam1 = 50.0f;
	m_fremoveOutlierParam2 = 1.0f;
	m_bsmoothEnable = false;
	m_fsmoothParam1 = 0.05f;
	m_fsmoothParam2 = 1.0f;
	m_nrebuildMethod = 0;
	m_nmeshSearchK = 20;
	m_fmeshSearchRadius = 2.0f;
	m_nmeshMaxNeighbors = 100;
	m_nmeshDisplayModel = 0;
	m_nmeshDisplayColor = 0;
	m_nImgIndex = 0;

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
		comboBox->addItem(QStringLiteral("GreedyProjectionTriangulation"));
		comboBox->addItem(QStringLiteral("Poisson"));
		comboBox->addItem(QStringLiteral("Method 3"));
		connect(comboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
			this, &CPropertyDelegate::meshBuildMethod);
		outputWidget = comboBox;
		break;
	}
	case OBJECT_REBUILD_SEARCH_K: {
		QSpinBox *spinBox = new QSpinBox(parent);
		spinBox->setRange(1, 1000);
		spinBox->setSingleStep(1);
		connect(spinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
			this, &CPropertyDelegate::meshSearchKChanged);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REBUILD_SEARCH_RADIUS: {
		QDoubleSpinBox* spinBox = new QDoubleSpinBox(parent);
		spinBox->setDecimals(2);
		spinBox->setRange(0.0, 100.0e2);
		spinBox->setSingleStep(0.01);
		connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
			this, &CPropertyDelegate::meshSearchRadiusChanged);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_REBUILD_MAX_NEIGHBORS: {
		QSpinBox *spinBox = new QSpinBox(parent);
		spinBox->setRange(1, 1000);
		spinBox->setSingleStep(2);
		connect(spinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
			this, &CPropertyDelegate::meshMaxNeighbors);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_MESH_DISPLAY_TYPE: {
		QComboBox *comboBox = new QComboBox(parent);
		comboBox->addItem(QString("Surface"));
		comboBox->addItem(QString("Point"));
		comboBox->addItem(QString("WireFrame"));
		connect(comboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
			this, &CPropertyDelegate::meshDisplayModel);
		outputWidget = comboBox;
		break;
	}
	case OBJECT_MESH_DISPLAY_COLOR: {
		QComboBox *comboBox = new QComboBox(parent);
		comboBox->addItem(QString("Default"));
		comboBox->addItem(QString("Red"));
		comboBox->addItem(QString("Green"));
		comboBox->addItem(QString("Blue"));
		connect(comboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
			this, &CPropertyDelegate::meshColor);
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
	case OBJECT_CLOUD_POINT_SIZE: 
		SetComboBoxIndex(editor, static_cast<int>(m_nCloudPointSize));
		break;
	case OBJECT_CLOUD_DISPLAY_COLOR:
		SetComboBoxIndex(editor, static_cast<int>(m_nCloudColor));
		break;
	case OBJECT_FILTER_METHOD:
		SetComboBoxIndex(editor, static_cast<int>(m_nFilterMethod));
		break;
	case OBJECT_FILTER_PARAM1:
		SetDoubleSpinBoxValue(editor, static_cast<double>(m_fFilterParam1));
		break;
	case OBJECT_FILTER_PARAM2:
		SetDoubleSpinBoxValue(editor, static_cast<double>(m_fFilterParam2));
		break;
	case OBJECT_FILTER_PARAM3:
		SetDoubleSpinBoxValue(editor, static_cast<double>(m_fFilterParam3));
		break;
	case OBJECT_REMOVE_OUTLIER_PARAM1:
		SetDoubleSpinBoxValue(editor, static_cast<double>(m_fremoveOutlierParam1));
		break;
	case OBJECT_REMOVE_OUTLIER_PARAM2:
		SetDoubleSpinBoxValue(editor, static_cast<double>(m_fremoveOutlierParam2));
		break;
	case OBJECT_SMOOTH_PARAM1:
		SetDoubleSpinBoxValue(editor, static_cast<double>(m_fsmoothParam1));
		break;
	case OBJECT_SMOOTH_PARAM2:
		SetDoubleSpinBoxValue(editor, static_cast<double>(m_fsmoothParam2));
		break;
	case OBJECT_REBUILD_METHOD:
		SetComboBoxIndex(editor, static_cast<int>(m_nrebuildMethod));
		break;
	case OBJECT_REBUILD_SEARCH_K:
		SetSpinBoxValue(editor, static_cast<int>(m_nmeshSearchK));
		break;
	case OBJECT_REBUILD_SEARCH_RADIUS:
		SetDoubleSpinBoxValue(editor, static_cast<double>(m_fmeshSearchRadius));
		break;
	case OBJECT_REBUILD_MAX_NEIGHBORS:
		SetSpinBoxValue(editor, static_cast<int>(m_nmeshMaxNeighbors));
		break;
	case OBJECT_MESH_DISPLAY_TYPE:
		SetComboBoxIndex(editor, static_cast<int>(m_nmeshDisplayModel));
		break;
	case OBJECT_MESH_DISPLAY_COLOR:
		SetComboBoxIndex(editor, static_cast<int>(m_nmeshDisplayColor));
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

// 更新图片属性
void CPropertyDelegate::updateImgIndex(int imgIndex)
{
	//qDebug() << "img index = " << imgIndex;
	m_nImgIndex = imgIndex;
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
	case IMAGE:
		fileModelWithImageInfo(m_nImgIndex);
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

	appendRow(ITEM(tr("Camera")), ITEM(tr("test object name"), Qt::ItemIsEditable, OBJECT_NAME));

	appendRow(ITEM(tr("Projection")), ITEM(tr("Information of object"), Qt::ItemIsEditable, OBJECT_INFO));

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

	appendRow(ITEM(tr("Coordinate")), CHECKABLE_ITEM(m_bCloudCoordinate, OBJECT_CLOUD_COORDINATE));
}

void CPropertyDelegate::fillModelWithFiltered()
{
	assert(m_model);

	addSeparator(QStringLiteral("点云滤波"));

	appendRow(ITEM(tr("Enable")), CHECKABLE_ITEM(m_bFilterEnable, OBJECT_FILTER_STATE));

	appendRow(ITEM(tr("Method")), PERSISTENT_EDITOR(OBJECT_FILTER_METHOD), true);

	appendRow(ITEM(tr("Parament1")), PERSISTENT_EDITOR(OBJECT_FILTER_PARAM1), true);

	appendRow(ITEM(tr("Parament2")), PERSISTENT_EDITOR(OBJECT_FILTER_PARAM2), true);

	appendRow(ITEM(tr("Parament3")), PERSISTENT_EDITOR(OBJECT_FILTER_PARAM3), true);
}

void CPropertyDelegate::fillModelWithCloudRemoveOutlier()
{
	assert(m_model);
	addSeparator(QStringLiteral("点云移除离群点"));
	appendRow(ITEM(tr("Enable")), CHECKABLE_ITEM(m_bremoveOutlierEnable, OBJECT_REMOVE_OUTLIER_STATE));
	appendRow(ITEM(tr("Mean K")), PERSISTENT_EDITOR(OBJECT_REMOVE_OUTLIER_PARAM1), true);
	appendRow(ITEM(tr("Standard Deviation")), PERSISTENT_EDITOR(OBJECT_REMOVE_OUTLIER_PARAM2), true);

}

void CPropertyDelegate::fillModelWithCloudSmooth()
{
	assert(m_model);
	addSeparator(QStringLiteral("点云平滑"));
	appendRow(ITEM(tr("Enable")), CHECKABLE_ITEM(m_bsmoothEnable, OBJECT_SMOOTH_STATE));
	appendRow(ITEM(tr("Search Radius")), PERSISTENT_EDITOR(OBJECT_SMOOTH_PARAM1), true);
	appendRow(ITEM(tr("Parament2")), PERSISTENT_EDITOR(OBJECT_SMOOTH_PARAM2), true);

}

void CPropertyDelegate::fillModelWithRebuild()
{
	assert(m_model);
	addSeparator(QStringLiteral("三维重建"));
	appendRow(ITEM(tr("Method")), PERSISTENT_EDITOR(OBJECT_REBUILD_METHOD), true);
	appendRow(ITEM(tr("Search K")), PERSISTENT_EDITOR(OBJECT_REBUILD_SEARCH_K), true);
	appendRow(ITEM(tr("Search Radius")), PERSISTENT_EDITOR(OBJECT_REBUILD_SEARCH_RADIUS), true);
	appendRow(ITEM(tr("Max NearestNeighbor")), PERSISTENT_EDITOR(OBJECT_REBUILD_MAX_NEIGHBORS), true);
}

void CPropertyDelegate::fillModelWithMesh()
{
	assert(m_model);

	addSeparator(QStringLiteral("网格"));

	appendRow(ITEM(tr("Name")), ITEM(tr("mesh name"), Qt::ItemIsEditable, OBJECT_MESH_NAME));

	appendRow(ITEM(tr("Display")), PERSISTENT_EDITOR(OBJECT_MESH_DISPLAY_TYPE), true);

	appendRow(ITEM(tr("Color")), PERSISTENT_EDITOR(OBJECT_MESH_DISPLAY_COLOR), true);

}

void CPropertyDelegate::fileModelWithImageInfo(int index)
{
	assert(m_model);

	addSeparator(QStringLiteral("图像参数"));
	
	//QString path = QStringLiteral("路径/XXXXXX/");
	QString name = QString::number(index) + ".bmp";
	appendRow(ITEM(tr("FileName")), ITEM(m_qstrImgPath + name));
	QImage img;
	img.load(m_qstrImgPath + name);
	//appendRow(ITEM(tr("Resolution")), ITEM(tr("1280 * 960")));
	appendRow(ITEM(tr("Resolution")), ITEM(QString::number(img.width()) + " , " + QString::number(img.height())));
	
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
	m_nCloudPointSize = index;
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
	m_nCloudColor = index;
	m_wnd->setCloudPointColor(index);
}

void CPropertyDelegate::cloudCoordinateDisplay(bool state)
{
	if (m_nCurrentObject == NONE)
		return;
	m_bCloudCoordinate = state;
	m_wnd->displayCoordinate(state);
}

void CPropertyDelegate::filterEnable(bool state)
{
	if (m_nCurrentObject == NONE)
		return;
	m_bFilterEnable = state;
	m_wnd->filterEnable(state);
}

void CPropertyDelegate::filterMethodChanged(int index)
{
	//qDebug() << "filter method = " << index;
	if (m_nCurrentObject == NONE)
		return;
	m_nFilterMethod = index;
	m_wnd->filterMethod(index);
}

void CPropertyDelegate::filterParam1Changed(double value)
{
	//qDebug() << "filter param1 = " << value;
	if (m_nCurrentObject == NONE)
		return;
	m_fFilterParam1 = value;
	float param1 = 0.0f, param2 = 0.0f, param3 = 0.0f;
	m_wnd->getFilterParams(param1, param2, param3);
	if (value != (double)param1)
		m_wnd->setFilterParams((float)value, param2, param3);
}

void CPropertyDelegate::filterParam2Changed(double value)
{
	//qDebug() << "filter param2 = " << value;
	if (m_nCurrentObject == NONE)
		return;
	m_fFilterParam2 = value;
	float param1 = 0.0f, param2 = 0.0f, param3 = 0.0f;
	m_wnd->getFilterParams(param1, param2, param3);
	if (value != (double)param2)
		m_wnd->setFilterParams(param1, (float)value, param3);
}

void CPropertyDelegate::filterParam3Changed(double value)
{
	//qDebug() << "filter param3 = " << value;
	if (m_nCurrentObject == NONE)
		return;
	m_fFilterParam3 = value;
	float param1 = 0.0f, param2 = 0.0f, param3 = 0.0f;
	m_wnd->getFilterParams(param1, param2, param3);
	if (value != (double)param3)
		m_wnd->setFilterParams(param1, param2, (float)value);
}

void CPropertyDelegate::removeOutlierEnable(bool state)
{
	//qDebug() << "remove Outlier enable = " << state;
	if (m_nCurrentObject == NONE)
		return;
	m_bremoveOutlierEnable = state;
	m_wnd->removeOutlierEnable(state);
}

void CPropertyDelegate::removeOutlierParam1Changed(double value)
{
	//qDebug() << "remove Outlier param1 = " << value;
	if (m_nCurrentObject == NONE)
		return;
	m_fremoveOutlierParam1 = value;
	float param1 = 0.0f, param2 = 0.0f;
	m_wnd->getRemoveOutlierParams(param1, param2);
	if (value != (double)param1)
		m_wnd->setRemoveOutlierParams((float)value, param2);
}

void CPropertyDelegate::removeOutlierParam2Changed(double value) 
{
	//qDebug() << "remove Outlier param2 = " << value;
	if (m_nCurrentObject == NONE)
		return;
	m_fremoveOutlierParam2 = value;
	float param1 = 0.0f, param2 = 0.0f;
	m_wnd->getRemoveOutlierParams(param1, param2);
	if (value != (double)param2)
		m_wnd->setRemoveOutlierParams(param1, (float)value);
}

void CPropertyDelegate::smoothEnable(bool state) 
{
	//qDebug() << "smooth enable = " << state;
	if (m_nCurrentObject == NONE)
		return;
	m_bsmoothEnable = state;
	m_wnd->smoothEnable(state);
}

void CPropertyDelegate::smoothParam1Changed(double value) 
{
	//qDebug() << "smooth param1 = " << value;
	if (m_nCurrentObject == NONE)
		return;
	m_fsmoothParam1 = value;
	float param1 = 0.0f, param2 = 0.0f;
	m_wnd->getSmoothParams(param1, param2);
	if (value != (double)param1)
		m_wnd->setSmoothParams((float)value, param2);
}

void CPropertyDelegate::smoothParam2Changed(double value)
{
	//qDebug() << "smooth param2 = " << value;
	if (m_nCurrentObject == NONE)
		return;
	m_fsmoothParam2 = value;
	float param1 = 0.0f, param2 = 0.0f;
	m_wnd->getSmoothParams(param1, param2);
	if (value != (double)param2)
		m_wnd->setSmoothParams(param1, (float)value);
}

void CPropertyDelegate::meshBuildMethod(int index)
{
	//qDebug() << "mesh method = " << index;
	m_nrebuildMethod = index;
	m_wnd->rebuildMethod(index);
}

void CPropertyDelegate::meshSearchKChanged(int value)
{
	//qDebug() << "mesh search k = " << value;
	if (m_nCurrentObject == NONE)
		return;
	m_nmeshSearchK = value;
	int k = 0, maxneigh = 0;
	float radius = 0.0f;
	m_wnd->getRebuildParams(k, radius, maxneigh);
	if (value != k)
		m_wnd->setRebuildParams(value, radius, maxneigh);
}

void CPropertyDelegate::meshSearchRadiusChanged(float value)
{
	//qDebug() << "mesh search radius = " << value;
	if (m_nCurrentObject == NONE)
		return;
	m_fmeshSearchRadius = value;
	int k = 0, maxneigh = 0;
	float radius = 0.0f;
	m_wnd->getRebuildParams(k, radius, maxneigh);
	if (value != radius)
		m_wnd->setRebuildParams(k, value, maxneigh);
}

void CPropertyDelegate::meshMaxNeighbors(int value)
{
	//qDebug() << "mesh max neighbors = " << value;
	if (m_nCurrentObject == NONE)
		return;
	m_nmeshMaxNeighbors = value;
	int k = 0, maxneigh = 0;
	float radius = 0.0f;
	m_wnd->getRebuildParams(k, radius, maxneigh);
	if (value != maxneigh)
		m_wnd->setRebuildParams(k, radius, value);
}

void CPropertyDelegate::meshDisplayModel(int index)
{
	//qDebug() << "mesh display model = " << index;
	if (m_nCurrentObject == NONE)
		return;
	m_nmeshDisplayModel = index;
	m_wnd->meshDisplayModel(index);
}

void CPropertyDelegate::meshColor(int index)
{
	//qDebug() << "mesh color = " << index;
	if (m_nCurrentObject == NONE)
		return;
	m_nmeshDisplayColor = index;
	m_wnd->setMeshColor(index);
}
