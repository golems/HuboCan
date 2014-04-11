
#include "../HuboQtMainWindow.h"
#include "../AchNetworkWidget.h"
#include <QHBoxLayout>

using namespace HuboQt;

HuboQtMainWindow::HuboQtMainWindow()
{
    AchNetworkWidget* ach = new AchNetworkWidget;
    
    setCentralWidget(ach);
}
