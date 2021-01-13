#pragma once


#include "ui_PlotWnd.h"

class CPlotWnd : public QWidget
{
	Q_OBJECT
public:
	CPlotWnd(QWidget *parents = 0);
	~CPlotWnd();


private:
	Ui::plotwnd ui;


};

