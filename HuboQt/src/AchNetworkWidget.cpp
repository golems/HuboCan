
#include "../AchNetworkWidget.h"

extern "C" {
#include "HuboRT/Daemonizer_C.h"
}

using namespace HuboQt;

const char* anw_save_directory = "/opt/hubo/qt/anw";
const char* anw_save_hostname_file = "/opt/hubo/qt/anw/hostname.txt";

AchNetworkWidget::AchNetworkWidget() :
    _ui(new Ui::AchNetworkWidget)
{
    _ui->setupUi(this);
    
    
    hubo_rt_safe_make_directory(anw_save_directory);
    
    load_hostname();
}

void AchNetworkWidget::load_hostname()
{
    
}

void AchNetworkWidget::save_hostname()
{
    
}











