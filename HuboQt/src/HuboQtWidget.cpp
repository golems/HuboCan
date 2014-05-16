
#include "../HuboQtWidget.h"

using namespace HuboQt;

HuboQtWidget::HuboQtWidget()
{
    mgr = new ManagerWidget;
    joints = new JointWidget;
    relay = new LogRelayWidget;
    configs = new ConfigWidget;

    connect(mgr, SIGNAL(channels_created()), joints, SLOT(initialize()));
    connect(mgr, SIGNAL(channels_created()), relay, SLOT(attempt_restart()));
    configs->initialize();

    addTab(mgr, "Manager");
    addTab(joints, "Joints");
    addTab(relay, "Logs");
    addTab(configs, "Configs");

    connect(this, SIGNAL(currentChanged(int)), this, SLOT(handle_tab_change(int)));
    connect(this, SIGNAL(refresh_configs()), configs, SLOT(refresh_lists()));
}

void HuboQtWidget::handle_tab_change(int new_tab)
{
    if( widget(new_tab) == configs )
    {
        emit refresh_configs();
    }
}
