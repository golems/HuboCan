
#include "../HuboQtMainWindow.h"
#include "../ManagerWidget.h"
#include <QHBoxLayout>

#include "../LogRelayWidget.h"

using namespace HuboQt;

HuboQtMainWindow::HuboQtMainWindow()
{
    ManagerWidget* mgr = new ManagerWidget;
    LogRelayWidget* relay = new LogRelayWidget;

    QTabWidget* tabs = new QTabWidget;
    tabs->addTab(mgr, "Manager");
    tabs->addTab(relay, "Logs");
    
    setCentralWidget(tabs);
}
