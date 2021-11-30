#pragma once

#include "ui_Roi.h"

class CRoi : public QWidget
{
	Q_OBJECT
public:
	CRoi(QWidget *parents = 0);
	~CRoi();


private:
	Ui::Roi ui;

};

