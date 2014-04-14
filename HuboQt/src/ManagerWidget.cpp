
#include <QFile>
#include <iostream>
#include <QTime>

#include "../ManagerWidget.h"

extern "C" {
#include "HuboRT/Daemonizer_C.h"
}

using namespace HuboQt;

const char* anw_save_directory = "/opt/hubo/qt/anw";
const char* anw_save_hostname_file = "/opt/hubo/qt/anw/hostname.txt";

ManagerWidget::ManagerWidget() :
    _ui(new Ui::ManagerWidget)
{
    _ui->setupUi(this);
    
    hubo_rt_safe_make_directory(anw_save_directory);
    
    load_hostname();
    
    AchdHandle* handle = new AchdHandle;
    handle->parse_description(QString(hubo_rt_mgr_req_chan)+":20:2048:PUSH:");
    connect(&(handle->achd_process), SIGNAL(finished(int)), this, SLOT(inform_disconnect(int)));
    _perm_achd_handles.push_back(handle);
    
    handle = new AchdHandle;
    handle->parse_description(QString(hubo_rt_mgr_reply_chan)+":20:2048:PULL:");
    connect(&(handle->achd_process), SIGNAL(finished(int)), this, SLOT(inform_disconnect(int)));
    _perm_achd_handles.push_back(handle);
    
    _start_achds(_perm_achd_handles);
    
    _req = new HuboRT::ManagerReq;
    if(!_req->is_initialized())
    {
        std::cerr << "Insane unexplainable error!"
                  << " The ManagerReq instance could not be initialized!!"
                  << std::endl;
    }
    
    // ---- Connecting UI signals ----
    
    connect(_ui->hostname_edit, SIGNAL(textChanged(QString)), this, SLOT(save_hostname(QString)));
    
    connect(_ui->reconnect, SIGNAL(clicked()), this, SLOT(start_all_achds()));
    connect(_ui->disconnect, SIGNAL(clicked()), this, SLOT(disconnect_all_achds()));
    
    connect(_ui->startup, SIGNAL(clicked()), this, SLOT(startup_everything()));
    connect(_ui->shutdown, SIGNAL(clicked()), this, SLOT(shutdown_everything()));
    connect(_ui->homeall, SIGNAL(clicked()), this, SLOT(homeall()));
}

void ManagerWidget::startup_everything()
{
    StringArray reply;
    manager_err_t result = _req->start_up(reply);
    
    if(NO_ERROR != result)
    {
        
    }
}

void ManagerWidget::shutdown_everything()
{
    
}

void ManagerWidget::homeall()
{
    
}

void ManagerWidget::disconnect_all_achds()
{
    _disconnect_achds(_perm_achd_handles);
    _disconnect_achds(_more_achd_handles);
}

void ManagerWidget::_disconnect_achds(AchdPtrArray &achds)
{
    for(int i=0; i<achds.size(); ++i)
    {
        achds[i]->achd_process.kill();
    }
}

void ManagerWidget::inform_disconnect(int exit_code)
{
    _ui->network_status->setText("Disconnected!");
    _check_if_achds_running(_perm_achd_handles, exit_code);
    _check_if_achds_running(_more_achd_handles, exit_code);
}

void ManagerWidget::_check_if_achds_running(AchdPtrArray &achds, int exit_code)
{
    for(int i=0; i<achds.size(); ++i)
    {
        if(achds[i]->achd_process.state() != QProcess::Running && achds[i]->started )
        {
            achds[i]->started = false;
            std::cout << "Channel '" << achds[i]->channel_name.toStdString()
                      << "' disconnected at "
                      << QTime::currentTime().toString().toStdString()
                      << " with exit code " << exit_code
                      << std::endl;
        }
    }
}

void ManagerWidget::_start_achds(AchdPtrArray &achds)
{
    _ui->network_status->setText("Connected");
    for(int i=0; i<achds.size(); ++i)
    {
        achds[i]->start(_ui->hostname_edit->text());
    }
}

void ManagerWidget::start_all_achds()
{
    _start_achds(_perm_achd_handles);
    _start_achds(_more_achd_handles);
}

ManagerWidget::~ManagerWidget()
{
    for(int i=0; i<_perm_achd_handles.size(); ++i)
    {
        delete _perm_achd_handles[i];
    }
    
    for(int i=0; i<_more_achd_handles.size(); ++i)
    {
        delete _more_achd_handles[i];
    }
    
    delete _req;
    delete _ui;
}

void ManagerWidget::load_hostname()
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

void ManagerWidget::save_hostname(const QString& new_name)
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











