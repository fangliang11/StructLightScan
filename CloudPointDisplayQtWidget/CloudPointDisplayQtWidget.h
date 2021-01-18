#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_CloudPointDisplayQtWidget.h"

class CloudPointDisplayQtWidget : public QMainWindow
{
    Q_OBJECT

public:
    CloudPointDisplayQtWidget(QWidget *parent = Q_NULLPTR);

private:
    Ui::CloudPointDisplayQtWidgetClass ui;
};
