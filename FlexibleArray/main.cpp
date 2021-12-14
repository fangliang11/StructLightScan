
#include <QtWidgets/QApplication>
#include "FlexibleArray.h"


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
	FlexibleArray mainwnd;
	mainwnd.setWindowIcon(QIcon("bitbug_favicon.ico"));
	mainwnd.resize(1280, 720);


	//QMdiArea *centralWidget = new QMdiArea;
	//CCalibrationWnd *calibwnd = new CCalibrationWnd;
	//calibwnd->setMinimumSize(400, 400);
	//centralWidget->addSubWindow(calibwnd);
	//mainwnd.setCentralWidget(centralWidget);


    mainwnd.show();
    return app.exec();
}
