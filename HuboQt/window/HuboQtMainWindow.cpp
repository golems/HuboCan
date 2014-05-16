
#include "../HuboQtMainWindow.h"
//#include <QHBoxLayout>

#include "../ManagerWidget.h"
#include "../LogRelayWidget.h"
#include "../JointWidget.h"
#include "../ConfigWidget.h"

using namespace HuboQt;

HuboQtMainWindow::HuboQtMainWindow()
{
    ManagerWidget* mgr = new ManagerWidget;
    JointWidget* joints = new JointWidget;
    LogRelayWidget* relay = new LogRelayWidget;
    ConfigWidget* configs = new ConfigWidget;

    connect(mgr, SIGNAL(channels_created()), joints, SLOT(initialize()));
    connect(mgr, SIGNAL(channels_created()), relay, SLOT(attempt_restart()));
    connect(mgr, SIGNAL(manager_channels_created()), configs, SLOT(initialize()));

    QTabWidget* tabs = new QTabWidget;
    tabs->addTab(mgr, "Manager");
    tabs->addTab(joints, "Joints");
    tabs->addTab(relay, "Logs");
    tabs->addTab(configs, "Configs");
    
    setCentralWidget(tabs);
}
