
#include "../HuboQtMainWindow.h"
//#include <QHBoxLayout>

using namespace HuboQt;

HuboQtMainWindow::HuboQtMainWindow()
{
    mgr = new ManagerWidget;
    joints = new JointWidget;
    relay = new LogRelayWidget;
    configs = new ConfigWidget;

    connect(mgr, SIGNAL(channels_created()), joints, SLOT(initialize()));
    connect(mgr, SIGNAL(channels_created()), relay, SLOT(attempt_restart()));
    configs->initialize();

    tabs = new QTabWidget;
    tabs->addTab(mgr, "Manager");
    tabs->addTab(joints, "Joints");
    tabs->addTab(relay, "Logs");
    tabs->addTab(configs, "Configs");

    connect(tabs, SIGNAL(currentChanged(int)), this, SLOT(handle_tab_change(int)));
    connect(this, SIGNAL(refresh_configs()), configs, SLOT(refresh_lists()));

    setCentralWidget(tabs);
}

void HuboQtMainWindow::handle_tab_change(int new_tab)
{
    if( tabs->widget(new_tab) == configs )
    {
        emit refresh_configs();
    }
}

