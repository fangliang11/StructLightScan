
#include <QtWidgets/QApplication>
#include "StructScan.h"


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    StructScan mainwnd;
	mainwnd.setWindowIcon(QIcon("s_logo.ico"));
	mainwnd.resize(1280, 720);


	//QMdiArea *centralWidget = new QMdiArea;
	//CCalibrationWnd *calibwnd = new CCalibrationWnd;
	//calibwnd->setMinimumSize(400, 400);
	//centralWidget->addSubWindow(calibwnd);
	//mainwnd.setCentralWidget(centralWidget);


    mainwnd.show();
    return app.exec();
}
