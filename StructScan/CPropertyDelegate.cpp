


#include "CPropertyDelegate.h"


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
static QStandardItem* ITEM(const QString& name,	Qt::ItemFlag additionalFlags = Qt::NoItemFlags,
	CPropertyDelegate::PROPERTY_CODE role = CPropertyDelegate::OBJECT_NO_PROPERTY)
{
	QStandardItem* item = new QStandardItem(name);
	//flags
	item->setFlags(Qt::ItemIsEnabled | additionalFlags);
	//role (if any)
	if (role != CPropertyDelegate::OBJECT_NO_PROPERTY){
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
	: m_model(_model),	m_view(_view), QStyledItemDelegate(parent)
{
	assert(m_model && m_view);

	initial();
}


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
	if (item->column() == 0){
		//on the first column, only editors spanning on 2 columns are allowed
		return nullptr;
	}

	QWidget* outputWidget = nullptr;
	switch (itemData)
	{
	case OBJECT_NAME:
		break;
	case OBJECT_VISIBILITY:	{
		QComboBox *comboBox = new QComboBox(parent);
		comboBox->addItem(QStringLiteral("是"));
		comboBox->addItem(QStringLiteral("否"));
		//connect(comboBox, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),
		//	this, &CPropertyDelegate::objectDisplayChanged);
		outputWidget = comboBox;
		break;
	}
	case OBJECT_CLOUD_NAME: {
		QLineEdit *edit = new QLineEdit(parent);
		edit->setText(QString("test cloud point"));
		//connect(edit, static_cast<void (QLineEdit::*)(const QString &)>(&QLineEdit::editingFinished),
		//	this, &CPropertyDelegate::objectCloudName);
		outputWidget = edit;
		break;
	}
	case OBJECT_CLOUD_POINT_SIZE: {
		QSpinBox *spinBox = new QSpinBox(parent);
		spinBox->setRange(1, 10);
		spinBox->setSingleStep(1);
		connect(spinBox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
			this, &CPropertyDelegate::cloudPointSizeChanged);
		outputWidget = spinBox;
		break;
	}
	case OBJECT_CLOUD_DISPLAY_COLOR:
		break;
	case OBJECT_CLOUD_COORDINATE:
		break;
	case OBJECT_FILTER_METHOD:
		break;
	case OBJECT_FILTER_SEARCH_RADIUS:
		break;
	case OBJECT_FILTER_LEAF_SIZE:
		break;
	case OBJECT_FILTER_MEAN_K:
		break;
	case OBJECT_FILTER_MULTY_THRESHOLD:
		break;
	case OBJECT_FILTER_POLYNOMIAL_FIT:
		break;
	case OBJECT_REBUILD_METHOD:
		break;
	case OBJECT_REBUILD_NEAREST_K:
		break;
	case OBJECT_REBUILD_SEARCH_RADIUS:
		break;
	case OBJECT_REBUILD_DISTANCE_MU:
		break;
	case OBJECT_REBUILD_MAX_NEAREST_NEIGHBOR:
		break;
	case OBJECT_REBUILD_MAX_SURFACE_ANGLE:
		break;
	case OBJECT_REBUILD_MIN_ANGLE:
		break;
	case OBJECT_REBUILD_NORMAL_CONSISTENCY:
		break;
	case OBJECT_MESH_NAME:
		break;
	case OBJECT_MESH_DISPLAY_TYPE:
		break;
	case OBJECT_MESH_DISPLAY_COLOR:
		break;
	default:break;
	}

	return outputWidget;
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
	case OBJECT_NAME:
		break;
	case OBJECT_VISIBILITY: {
		QComboBox *comboBox = qobject_cast<QComboBox*>(editor);
		if (!comboBox){
			assert(false);
			return;
		}
		comboBox->setCurrentIndex(comboBox->currentIndex()); //0 = "NONE"
		break;
	}
	case OBJECT_CLOUD_NAME: {
		QLineEdit *edit = qobject_cast<QLineEdit*>(editor);
		if (!edit) {
			assert(false);
			return;
		}
		break;
	}
	case OBJECT_CLOUD_POINT_SIZE: {
		QLabel* label = qobject_cast<QLabel*>(editor);
		if (label)
		{
			label->setText(item->accessibleDescription());
		}
		break;
	}
	case OBJECT_CLOUD_DISPLAY_COLOR:
		break;
	case OBJECT_CLOUD_COORDINATE:
		break;
	case OBJECT_FILTER_METHOD:
		break;
	case OBJECT_FILTER_SEARCH_RADIUS:
		break;
	case OBJECT_FILTER_LEAF_SIZE:
		break;
	case OBJECT_FILTER_MEAN_K:
		break;
	case OBJECT_FILTER_MULTY_THRESHOLD:
		break;
	case OBJECT_FILTER_POLYNOMIAL_FIT:
		break;
	case OBJECT_REBUILD_METHOD:
		break;
	case OBJECT_REBUILD_NEAREST_K:
		break;
	case OBJECT_REBUILD_SEARCH_RADIUS:
		break;
	case OBJECT_REBUILD_DISTANCE_MU:
		break;
	case OBJECT_REBUILD_MAX_NEAREST_NEIGHBOR:
		break;
	case OBJECT_REBUILD_MAX_SURFACE_ANGLE:
		break;
	case OBJECT_REBUILD_MIN_ANGLE:
		break;
	case OBJECT_REBUILD_NORMAL_CONSISTENCY:
		break;
	case OBJECT_MESH_NAME:
		break;
	case OBJECT_MESH_DISPLAY_TYPE:
		break;
	case OBJECT_MESH_DISPLAY_COLOR:
		break;
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
	Q_UNUSED(index)

	editor->setGeometry(option.rect);
}


void CPropertyDelegate::updateDisplay()
{
	emit signalObjectPropertiesChanged();
}


void CPropertyDelegate::updateModel()
{
	fillModel();
	//fillModelWithCloud();
}


void CPropertyDelegate::updateItem()
{

}


void CPropertyDelegate::initial()
{
}


void CPropertyDelegate::addSeparator(const QString& title)
{
	if (m_model){
		//DGM: we can't use the 'text' of the item as it will be displayed under the associated editor (label)!
		//So we simply use the 'accessible description' field
		QStandardItem* leftItem = new QStandardItem(title);
		leftItem->setData(0);
		leftItem->setAccessibleDescription(title);
		leftItem->setForeground(QBrush(QColor(Qt::red)));
		m_model->appendRow(leftItem);

		if (m_view != nullptr){
			m_view->openPersistentEditor(m_model->index(m_model->rowCount() - 1, 0));
		}
	}
}


void CPropertyDelegate::appendRow(QStandardItem* leftItem, QStandardItem* rightItem, bool openPersistentEditor/*=false*/)
{
	assert(leftItem && rightItem);
	assert(m_model);

	if (m_model){
		//append row
		QList<QStandardItem*> rowItems{ leftItem, rightItem };

		m_model->appendRow(rowItems);

		//the persistent editor (if any) is always the right one!
		if (openPersistentEditor && (m_view != nullptr)){
			m_view->openPersistentEditor(m_model->index(m_model->rowCount() - 1, 1));
		}
	}
}


void CPropertyDelegate::fillModel()
{
	assert(m_model);

	addSeparator("model");

	//appendRow(ITEM(tr("Model method")), ITEM(tr("55")), OBJECT_CLOUD_POINT_SIZE);
	appendRow(ITEM(tr("Point size")), PERSISTENT_EDITOR(OBJECT_VISIBILITY), true);
}


void CPropertyDelegate::fillModelWithCloud()
{
	assert(m_model);

	addSeparator("Cloud");

	//appendRow(ITEM(tr("Points")), ITEM(QLocale(QLocale::English).toString(4096)), false);

	//appendRow(ITEM(tr("Point size")), PERSISTENT_EDITOR(OBJECT_CLOUD_POINT_SIZE), true);

	//appendRow(ITEM(tr("Current")), PERSISTENT_EDITOR(OBJECT_CURRENT_COLOR_RAMP), true);

	//appendRow(ITEM(tr("Steps")), PERSISTENT_EDITOR(OBJECT_COLOR_RAMP_STEPS), true);

	//appendRow(ITEM(tr("Visible")), CHECKABLE_ITEM(true, OBJECT_SF_SHOW_SCALE), true);

}


void CPropertyDelegate::fillModelWithFiltered()
{

}


void CPropertyDelegate::fileModelWithRebuild()
{

}


void CPropertyDelegate::fileModelWithMesh()
{

}


void CPropertyDelegate::fileModelWithSelect()
{

}


void CPropertyDelegate::cloudPointSizeChanged()
{
	int size = 0;

}

