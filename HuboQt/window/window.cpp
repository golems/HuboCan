
#include <QApplication>
#include "../HuboQtMainWindow.h"

#include "../LogRelayWidget.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    
    HuboQt::HuboQtMainWindow window;
    window.show();

    return app.exec();
}

