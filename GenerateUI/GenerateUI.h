#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_GenerateUI.h"

class GenerateUI : public QMainWindow
{
    Q_OBJECT

public:
    GenerateUI(QWidget *parent = Q_NULLPTR);

private:
    Ui::GenerateUIClass ui;
};
