
#include <QFile>
#include <iostream>
#include <QTime>

#include "../ManagerWidget.h"
#include "HuboRT/utils.hpp"

extern "C" {
#include "HuboRT/Daemonizer_C.h"
}

using namespace HuboQt;

const char* anw_save_directory = "/opt/hubo/qt/anw";
const char* anw_save_hostname_file = "/opt/hubo/qt/anw/hostname.txt";

ManagerWidget::ManagerWidget() :
    _ui(new Ui::ManagerWidget),
    _init(false)
{
    _ui->setupUi(this);
    
    hubo_rt_safe_make_directory(anw_save_directory);
    
    load_hostname();
    
    AchdHandle* handle = new AchdHandle;
    handle->parse_description("manager_request:"+QString(hubo_rt_mgr_req_chan)+":20:2048:PUSH:");
    connect(&(handle->achd_process), SIGNAL(finished(int)), this, SLOT(inform_disconnect(int)));
    _perm_achd_handles.push_back(handle);
    
    handle = new AchdHandle;
    handle->parse_description("manager_reply:"+QString(hubo_rt_mgr_reply_chan)+":20:2048:PULL:");
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
    
    connect(_ui->create_all, SIGNAL(clicked()), this, SLOT(create_all()));
    connect(_ui->destroy_all, SIGNAL(clicked()), this, SLOT(destroy_all()));
    connect(_ui->launch_all, SIGNAL(clicked()), this, SLOT(launch_all()));
    connect(_ui->stop_all, SIGNAL(clicked()), this, SLOT(stop_all()));
    
    connect(_ui->refresh_chan, SIGNAL(clicked()), this, SLOT(refresh_chans()));
    connect(_ui->refresh_registered_procs, SIGNAL(clicked()), this, SLOT(refresh_registered_procs()));
    connect(_ui->refresh_locked_procs, SIGNAL(clicked()), this, SLOT(refresh_locked_procs()));
    
    
}

bool ManagerWidget::_double_check_init()
{
    if(!_init.ready())
        _init.initialize();
    
    if(!_init.ready())
    {
        _ui->reply_status->setText("Could not use the Initializer "
                                   "(check terminal output for details)");
    }
    
    return _init.ready();
}

void ManagerWidget::_set_status(manager_err_t incoming_status, const QString &status_context)
{
    if(incoming_status != NO_ERROR)
    {
        std::cerr << "Error during '" << status_context.toStdString() << "': "
                  << manager_err_to_string(incoming_status) << std::endl;
    }
    
    _ui->reply_status->setText(QString::fromStdString(manager_err_to_string(incoming_status))
                               + " (" + status_context + ")");
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

void ManagerWidget::startup_everything()
{
    StringArray reply;
    manager_err_t result = _req->start_up(reply);
    
    _set_status(result, "startup command");
    
    if(NO_ERROR != result)
        return;
    
    _parse_channel_descriptions(reply);
}

void ManagerWidget::_parse_channel_descriptions(const StringArray &descs)
{
    for(int i=0; i<_more_achd_handles.size(); ++i)
    {
        delete _more_achd_handles[i];
    }
    _more_achd_handles.resize(0);
    
    for(size_t i=0; i<descs.size(); ++i)
    {
        AchdHandle* handle = new AchdHandle;
        handle->start(_ui->hostname_edit->text(), QString::fromStdString(descs[i]));
        _more_achd_handles.push_back(handle);
    }
    
    _display_channels();
}

void ManagerWidget::_display_channels()
{
    _ui->chan_list->clear();
    for(int i=0; i<_more_achd_handles.size(); ++i)
    {
        QListWidgetItem* new_chan = new QListWidgetItem;
        new_chan->setText(_more_achd_handles[i]->nickname);
        new_chan->setToolTip("Details: "
                             +_more_achd_handles[i]->channel_name
                             +", "+QString::number(_more_achd_handles[i]->message_count)
                             +"x"+QString::number(_more_achd_handles[i]->nominal_size)
                             +" bytes, "
                             +QString::fromStdString(
                                 achd_network_to_string(
                                     _more_achd_handles[i]->push_or_pull)));
    }
}

void ManagerWidget::_display_registered_processes(const StringArray &procs)
{
    _ui->proc_list->clear();
    _displaying_registered = true;
    _ui->stop_proc->setEnabled(false);
    _ui->launch_proc->setEnabled(true);
    _ui->refresh_locked_procs->setChecked(false);
    for(size_t i=0; i<procs.size(); ++i)
    {
        StringArray components;
        if(HuboRT::split_components(procs[i], components) < 2)
        {
            std::cerr << "Invalid process description: " << procs[i] << std::endl;
            continue;
        }
        
        QListWidgetItem* new_proc = new QListWidgetItem;
        new_proc->setText(QString::fromStdString(components[0]));
        new_proc->setToolTip("args: "+QString::fromStdString(components[1]));
    }
}

void ManagerWidget::_display_locked_processes(const StringArray &locks)
{
    _ui->proc_list->clear();
    _displaying_registered = false;
    _ui->stop_proc->setEnabled(true);
    _ui->launch_proc->setEnabled(false);
    _ui->refresh_registered_procs->setChecked(false);
    for(size_t i=0; i<locks.size(); ++i)
    {
        StringArray components;
        if(HuboRT::split_components(locks[i], components) < 2)
        {
            std::cerr << "Invalid process description: " << locks[i] << std::endl;
            continue;
        }
        
        QListWidgetItem* new_lock = new QListWidgetItem;
        new_lock->setText(QString::fromStdString(components[0]));
        new_lock->setToolTip("pid: "+QString::fromStdString(components[1]));
    }
}

void ManagerWidget::shutdown_everything()
{
    manager_err_t result = _req->shut_down();
    _set_status(result, "shutdown command");
}

void ManagerWidget::homeall()
{
    if(!_double_check_init())
        return;
    
    _init.home_all_joints();
}

void ManagerWidget::create_all()
{
    StringArray reply;
    manager_err_t result = _req->create_all_ach_channels(reply);
    
    _set_status(result, "create all channels");
    
    if(NO_ERROR != result)
        return;
    
    _parse_channel_descriptions(reply);
}

void ManagerWidget::destroy_all()
{
    _set_status(_req->close_all_ach_channels(), "destroy all channels");
}

void ManagerWidget::launch_all()
{
    _set_status(_req->run_all_processes(), "launch all processes");
}

void ManagerWidget::stop_all()
{
    _set_status(_req->stop_all_processes(), "stop all processes");
}

void ManagerWidget::refresh_chans()
{
    StringArray reply;
    manager_err_t result = _req->list_channels(reply);
    
    _set_status(result, "refresh channels");
    
    if(NO_ERROR != result)
        return;
    
    _parse_channel_descriptions(reply);
}

void ManagerWidget::refresh_registered_procs()
{
    StringArray reply;
    manager_err_t result = _req->list_registered_processes(reply);
    
    _set_status(result, "refresh all processes");
    
    if(NO_ERROR != result)
    {
        _ui->refresh_registered_procs->setChecked(false);
        return;
    }
    
    _display_registered_processes(reply);
}

void ManagerWidget::refresh_locked_procs()
{
    StringArray reply;
    manager_err_t result = _req->list_locked_processes(reply);
    
    _set_status(result, "refresh locked processes");
    
    if(NO_ERROR != result)
    {
        _ui->refresh_locked_procs->setChecked(false);
        return;
    }
    
    _display_locked_processes(reply);
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
            std::cout << "Channel '" << achds[i]->nickname.toStdString() << "' ("
                      << achds[i]->channel_name.toStdString() << ")"
                      << " disconnected at "
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











