
#include <QApplication>
#include "../HuboQtMainWindow.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    
    HuboQt::HuboQtMainWindow window;
    window.show();
    
    return app.exec();
}

