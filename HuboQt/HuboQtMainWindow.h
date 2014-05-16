#ifndef HUBOQTMAINWINDOW_H
#define HUBOQTMAINWINDOW_H

#include <QMainWindow>

#include "ManagerWidget.h"
#include "LogRelayWidget.h"
#include "JointWidget.h"
#include "ConfigWidget.h"

namespace HuboQt {

class HuboQtMainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    
    HuboQtMainWindow();

    ManagerWidget* mgr;
    JointWidget* joints;
    LogRelayWidget* relay;
    ConfigWidget* configs;

    QTabWidget* tabs;

protected Q_SLOTS:

    void handle_tab_change(int new_tab);

Q_SIGNALS:

    void refresh_configs();
    
};

} // namespace HuboQt

#endif // HUBOQTMAINWINDOW_H
