
#include "../HuboQtMainWindow.h"
//#include <QHBoxLayout>

#include "../ManagerWidget.h"
#include "../LogRelayWidget.h"
#include "../JointWidget.h"

using namespace HuboQt;

HuboQtMainWindow::HuboQtMainWindow()
{
    ManagerWidget* mgr = new ManagerWidget;
    JointWidget* joints = new JointWidget;
    LogRelayWidget* relay = new LogRelayWidget;

    connect(mgr, SIGNAL(channels_opened()), joints, SLOT(initialize()));
    connect(mgr, SIGNAL(channels_opened()), relay, SLOT(attempt_restart()));

    QTabWidget* tabs = new QTabWidget;
    tabs->addTab(mgr, "Manager");
    tabs->addTab(joints, "Joints");
    tabs->addTab(relay, "Logs");
    
    setCentralWidget(tabs);
}
