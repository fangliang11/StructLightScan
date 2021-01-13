#include "GenerateUI.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    GenerateUI w;
    w.show();
    return a.exec();
}
