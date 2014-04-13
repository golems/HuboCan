
#include "../HuboQtMainWindow.h"
#include "../ManagerWidget.h"
#include <QHBoxLayout>

using namespace HuboQt;

HuboQtMainWindow::HuboQtMainWindow()
{
    ManagerWidget* ach = new ManagerWidget;
    
    setCentralWidget(ach);
}
