
#include "CImageWnd.h"


CImageWnd::CImageWnd(QWidget *parents)
{
	ui.setupUi(this);
	setWindowFlags(Qt::WindowStaysOnTopHint);

	this->setMinimumSize(300, 300);

}


CImageWnd::~CImageWnd()
{

}


void CImageWnd::paintEvent(QPaintEvent *event)
{
	Q_UNUSED(event);

	//QStyleOption opt;
	//opt.init(this);
	//QPainter *p = new QPainter(this);
	//style()->drawPrimitive(QStyle::PE_Widget, &opt, p, this);
	//QWidget::paintEvent(event);

}