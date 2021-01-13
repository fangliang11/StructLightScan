#include "CloudPointDisplayQt.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CloudPointDisplayQt w;
    w.show();
    return a.exec();
}
