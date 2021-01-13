#pragma once

#include <QStyleOption>

#include "ui_ImageWnd.h"


class CImageWnd :	public QWidget
{
	Q_OBJECT
public:
	CImageWnd(QWidget *parents = 0);
	~CImageWnd();


private:
	Ui::ImageWnd ui;


	void paintEvent(QPaintEvent *event);

};

