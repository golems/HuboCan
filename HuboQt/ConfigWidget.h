#ifndef CONFIGWIDGET_H
#define CONFIGWIDGET_H

#include "HuboRT/ManagerReq.hpp"

#include "ui_ConfigWidget.h"

namespace HuboQt {

class ConfigWidget : public QWidget
{
    Q_OBJECT

public:

    ConfigWidget();
    ~ConfigWidget();

public Q_SLOTS:

    void initialize();

protected:

    HuboRT::ManagerReq* _req;

    Ui::ConfigWidget* _ui;


};

}

#endif // CONFIGWIDGET_H
