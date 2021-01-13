#pragma once

#include "ui_CalibrationWnd.h"

class CCalibrationWnd : public QWidget
{

	Q_OBJECT
public:
	CCalibrationWnd(QWidget *parents = 0);
	~CCalibrationWnd();


private:
	Ui::CalibrationWnd ui;

	void InitialWnd();

};

