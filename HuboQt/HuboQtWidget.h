#ifndef HUBOQTWIDGET_H
#define HUBOQTWIDGET_H

#include "ManagerWidget.h"
#include "LogRelayWidget.h"
#include "JointWidget.h"
#include "ConfigWidget.h"

namespace HuboQt {

class HuboQtWidget : public QTabWidget
{
    Q_OBJECT

public:

    HuboQtWidget();

    ManagerWidget* mgr;
    JointWidget* joints;
    LogRelayWidget* relay;
    ConfigWidget* configs;

protected Q_SLOTS:

    void handle_tab_change(int new_tab);

Q_SIGNALS:

    void refresh_configs();

};

}

#endif // HUBOQTWIDGET_H
