
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


    QTabWidget* tabs = new QTabWidget;
    tabs->addTab(mgr, "Manager");
    tabs->addTab(joints, "Joints");
    tabs->addTab(relay, "Logs");
    
    setCentralWidget(tabs);
}
