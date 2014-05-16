
#include "../HuboQtMainWindow.h"
//#include <QHBoxLayout>

using namespace HuboQt;

HuboQtMainWindow::HuboQtMainWindow()
{
    setCentralWidget(new HuboQtWidget);
}
