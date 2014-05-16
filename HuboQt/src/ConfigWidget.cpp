
#include "../ConfigWidget.h"

using namespace HuboQt;

ConfigWidget::ConfigWidget() :
    _ui(new Ui::ConfigWidget)
{
    _req = NULL;
    _ui->setupUi(this);
}

ConfigWidget::~ConfigWidget()
{
    delete _ui;
}

void ConfigWidget::initialize()
{
    _req = new HuboRT::ManagerReq;
}
