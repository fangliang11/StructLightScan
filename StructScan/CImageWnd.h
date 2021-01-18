#pragma once

#include <QWidget>
#include <QPixmap>



class CImageWnd :	public QWidget
{
	Q_OBJECT
public:
	explicit CImageWnd(QWidget *parent = nullptr);
	~CImageWnd();


	void setFrame(const QPixmap &pixmap);

	void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
	QPixmap m_frame; 



};

