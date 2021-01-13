#include "CCalibrationWnd.h"


CCalibrationWnd::CCalibrationWnd(QWidget *parents /* = 0 */)
{
	ui.setupUi(this);

	this->setMinimumSize(400, 400);

	InitialWnd();
}


CCalibrationWnd::~CCalibrationWnd()
{

}


void CCalibrationWnd::InitialWnd()
{
	//this->setAttribute(QtCore.Qt.WA_StyledBackground, true);
	//this->setAutoFillBackground(true);

	//ÉèÖÃÑÕÉ«
	//this->setStyleSheet(QString("background-color: rgb(255, 255, 255)"));
	//QPalette palette(this->palette());
	//palette.setColor(QPalette::Background, Qt::white);
	//this->setPalette(palette);
}
