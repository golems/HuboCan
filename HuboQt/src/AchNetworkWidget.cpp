
#include <QFile>
#include <iostream>

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
    
    connect(_ui->hostname_edit, SIGNAL(textChanged(QString)), this, SLOT(save_hostname(QString)));
}

void AchNetworkWidget::load_hostname()
{
    QFile hostname(anw_save_hostname_file);
    if(!hostname.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        std::cout << "Unable to find the hostname file. Leaving hostname blank for now."
                  << std::endl;
        return;
    }
    
    QString line = hostname.readLine();
    _ui->hostname_edit->setText(line);
    
    hostname.close();
}

void AchNetworkWidget::save_hostname(const QString& new_name)
{
    QFile hostname(anw_save_hostname_file);
    if(!hostname.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        std::cout << "Unable to save hostname file. Maybe a permissions issue?";
        return;
    }
    
    hostname.write(new_name);
    
    hostname.close();
}











