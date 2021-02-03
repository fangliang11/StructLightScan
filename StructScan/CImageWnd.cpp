
#include "CImageWnd.h"
#include <QStyleOption>
#include <QPainter>
#include <QPalette>



CImageWnd::CImageWnd(QWidget *parent): QWidget(parent), m_frame("TestImageInitial.bmp")
{
	setAttribute(Qt::WA_OpaquePaintEvent);
	setWindowFlags(this->windowFlags() | Qt::WindowStaysOnTopHint);

	setFixedSize(300, 300);
	QPalette pe;
	pe.setColor(QPalette::Background, QColor("black"));
	this->setAutoFillBackground(true);
	this->setPalette(pe);


	//ui.setupUi(parent);
	////setFixedSize(500, 500);
	//this->setWindowFlags(this->windowFlags() | Qt::WindowStaysOnTopHint);
	//this->setWindowTitle(QStringLiteral("œ‡ª˙"));

	//this->move(800, 800);
	//this->show();

}


CImageWnd::~CImageWnd()
{

}


void CImageWnd::paintEvent(QPaintEvent *event)
{
	Q_UNUSED(event);

	QStyleOption opt;
	opt.initFrom(this);
	QPainter p(this);
	style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
	p.drawPixmap(0, 0, width(), height(), m_frame);//ªÊ÷∆±≥æ∞

}


void CImageWnd::setFrame(const QPixmap &pixmap)
{
	m_frame = pixmap;

	update();
}
