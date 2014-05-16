
#include <QFile>
#include <iostream>
#include <QTime>
#include <QTimer>
#include <QColor>

#include <stdio.h>

#include "../ManagerWidget.h"
#include "HuboRT/utils.hpp"
#include "HuboRT/Manager.hpp"

extern "C" {
#include "HuboRT/Daemonizer_C.h"
}

using namespace HuboQt;

// anw = ach networking widget
const char* anw_save_directory = "/opt/hubo/qt/anw";
const char* anw_save_hostname_file = "/opt/hubo/qt/anw/hostname.txt";

ManagerWidget::ManagerWidget(const ManagerWidget&) : QWidget() { }
ManagerWidget& ManagerWidget::operator =(const ManagerWidget&) { return *this; }

ManagerWidget::ManagerWidget() :
    _ui(new Ui::ManagerWidget),
    _init(false)
{
    _connected = false;
    QColor color(100, 230, 100);
    selected_button_style = "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, "
            "stop: 0 #dadbde, stop: 1 " + color.name() + ")";

    _ui->setupUi(this);
    
    hubo_rt_safe_make_directory(anw_save_directory);
    
    load_hostname();
    
    AchdHandle* handle = new AchdHandle;
    handle->parse_description("manager_request:"+QString(hubo_rt_mgr_req_chan)+":20:2048:PUSH:");
    connect(&(handle->achd_process),
            SIGNAL(finished(int)),
            this, SLOT(inform_disconnect(int)), Qt::UniqueConnection);
    _mngr_achd_handles.push_back(handle);
    
    handle = new AchdHandle;
    handle->parse_description("manager_reply:"+QString(hubo_rt_mgr_reply_chan)+":20:2048:PULL:");
    connect(&(handle->achd_process),
            SIGNAL(finished(int)),
            this, SLOT(inform_disconnect(int)), Qt::UniqueConnection);
    _mngr_achd_handles.push_back(handle);
    
    if(_ui->hostname_edit->text().size() > 0)
        _start_achds(_mngr_achd_handles);
    
    _req = new HuboRT::ManagerReq;
    if(!_req->is_initialized())
    {
        std::cerr << "Insane unexplainable error!"
                  << " The ManagerReq instance could not be initialized!!"
                  << std::endl;
    }
    emit manager_channels_created();
    
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
    connect(_ui->kill_all, SIGNAL(clicked()), this, SLOT(kill_all()));
    
    connect(_ui->refresh_chan, SIGNAL(clicked()), this, SLOT(refresh_chans()));
    connect(_ui->refresh_registered_procs, SIGNAL(clicked()), this, SLOT(refresh_registered_procs()));
    connect(_ui->refresh_locked_procs, SIGNAL(clicked()), this, SLOT(refresh_locked_procs()));
    
    connect(_ui->create_chan, SIGNAL(clicked()), this, SLOT(create_chan()));
    connect(_ui->destroy_chan, SIGNAL(clicked()), this, SLOT(destroy_chan()));
    connect(_ui->launch_proc, SIGNAL(clicked()), this, SLOT(launch_proc()));
    connect(_ui->stop_proc, SIGNAL(clicked()), this, SLOT(stop_proc()));
    connect(_ui->kill_proc, SIGNAL(clicked()), this, SLOT(kill_proc()));

    connect(_ui->localMgrLaunch, SIGNAL(clicked()), this, SLOT(local_mgr_launch()));
    connect(_ui->localMgrStop, SIGNAL(clicked()), this, SLOT(local_mgr_stop()));

    connect(this, SIGNAL(stop_lmgr()), &_lmgr, SLOT(stop()));

    if(_ui->hostname_edit->text().size() > 0)
        QTimer::singleShot(2000, this, SLOT(create_all()));

    // TODO: Figure out a better way to periodically update the locked processes
//    QTimer* refresh_locked_timer = new QTimer(this);
//    connect(refresh_locked_timer, SIGNAL(timeout()), this, SLOT(timer_refresh()));
//    refresh_locked_timer->start(5000);
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
    if(incoming_status != NO_ERROR && incoming_status != EMPTY_LIST)
    {
        std::cerr << "Error during '" << status_context.toStdString() << "': "
                  << manager_err_to_string(incoming_status) << std::endl;
    }
    
    _ui->reply_status->setText(QString::fromStdString(manager_err_to_string(incoming_status))
                               + " (" + status_context + ")");
}

void ManagerWidget::disconnect_all_achds()
{
    _disconnect_achds(_mngr_achd_handles);
    _disconnect_achds(_main_achd_handles);
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
    create_all();
    launch_all();
}

void ManagerWidget::_clear_achd_handles(AchdPtrArray &achds)
{
    AchdPtrArray backup = achds;
    achds.clear();
    for(int i=0; i<backup.size(); ++i)
    {
        delete backup[i];
    }
}

void ManagerWidget::_parse_channel_descriptions(const StringArray &descs)
{
    _clear_achd_handles(_main_achd_handles);
    for(size_t i=0; i<descs.size(); ++i)
    {
        AchdHandle* handle = new AchdHandle;
        connect(&(handle->achd_process),
                SIGNAL(finished(int)),
                this, SLOT(inform_disconnect(int)), Qt::UniqueConnection);
        handle->start(_ui->hostname_edit->text(), QString::fromStdString(descs[i]));
        _main_achd_handles.push_back(handle);
    }
    
    _display_channels();
}

void ManagerWidget::_display_channels()
{
    _ui->chan_list->clear();
    for(int i=0; i<_main_achd_handles.size(); ++i)
    {
        QListWidgetItem* new_chan = new QListWidgetItem;
        new_chan->setText(_main_achd_handles[i]->nickname);
        new_chan->setToolTip("Details: "
                             +_main_achd_handles[i]->channel_name
                             +", "+QString::number(_main_achd_handles[i]->message_count)
                             +"x"+QString::number(_main_achd_handles[i]->nominal_size)
                             +" bytes, "
                             +QString::fromStdString(
                                 achd_network_to_string(
                                     _main_achd_handles[i]->push_or_pull)));

        _ui->chan_list->addItem(new_chan);
    }
    emit channels_created();
}

void ManagerWidget::_display_registered_processes(const StringArray &procs)
{
    _ui->proc_list->clear();
    _ui->stop_proc->setEnabled(false);
    _ui->kill_proc->setEnabled(false);
    _ui->launch_proc->setEnabled(true);
    for(size_t i=0; i<procs.size(); ++i)
    {
        StringArray components;
        if(HuboRT::split_components(procs[i], components) < 3)
        {
            std::cerr << "Invalid process description: " << procs[i] << std::endl;
            continue;
        }
        
        QListWidgetItem* new_proc = new QListWidgetItem;
        new_proc->setText(QString::fromStdString(components[0]));
        new_proc->setToolTip("bin: "+QString::fromStdString(components[1])
                +", args: "+QString::fromStdString(components[2]));

        _ui->proc_list->addItem(new_proc);
    }
}

void ManagerWidget::_display_locked_processes(const StringArray &locks)
{
    _ui->proc_list->clear();
    _ui->stop_proc->setEnabled(true);
    _ui->kill_proc->setEnabled(true);
    _ui->launch_proc->setEnabled(false);
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

        _ui->proc_list->addItem(new_lock);
    }
}

void ManagerWidget::shutdown_everything()
{
    stop_all();
    destroy_all();
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
    _disconnect_achds(_main_achd_handles);
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

void ManagerWidget::kill_all()
{
    _set_status(_req->kill_all_processes(), "kill all processes");
}

void ManagerWidget::refresh_chans()
{
    StringArray reply;
    manager_err_t result = _req->list_channels(reply);
    
    _set_status(result, "refresh channels");
    
    if(NO_ERROR != result && EMPTY_LIST != result)
        return;
    
    _parse_channel_descriptions(reply);
}

void ManagerWidget::set_displaying_locked_procs(bool displaying_locked_procs)
{
    _displaying_locked_procs = displaying_locked_procs;
    if(displaying_locked_procs)
    {
        _ui->refresh_locked_procs->setStyleSheet(selected_button_style);
        _ui->refresh_registered_procs->setStyleSheet("");
        refresh_locked_procs_raw();
    }
    else
    {
        _ui->refresh_registered_procs->setStyleSheet(selected_button_style);
        _ui->refresh_locked_procs->setStyleSheet("");
        refresh_registered_procs_raw();
    }
}

void ManagerWidget::refresh_registered_procs()
{
    set_displaying_locked_procs(false);
}

void ManagerWidget::refresh_registered_procs_raw()
{
    StringArray reply;
    manager_err_t result = _req->list_registered_processes(reply);
    
    _set_status(result, "refresh all processes");
    
    if(NO_ERROR != result && EMPTY_LIST != result)
    {
        _ui->refresh_registered_procs->setChecked(false);
        return;
    }
    
    _display_registered_processes(reply);
}

void ManagerWidget::refresh_locked_procs()
{
    set_displaying_locked_procs(true);
}

void ManagerWidget::refresh_locked_procs_raw()
{
    StringArray reply;
    manager_err_t result = _req->list_locked_processes(reply);
    
    _set_status(result, "refresh locked processes");
    
    if(NO_ERROR != result && EMPTY_LIST != result)
    {
        _ui->refresh_locked_procs->setChecked(false);
        return;
    }
    
    _display_locked_processes(reply);
}

void ManagerWidget::timer_refresh()
{
    if(_displaying_locked_procs && _connected)
        refresh_locked_procs_raw();
}

void ManagerWidget::create_chan()
{
    QList<QListWidgetItem*> items = _ui->chan_list->selectedItems();

    StringArray reply;
    for(int i=0; i<items.size(); ++i)
    {
        _set_status(_req->create_ach_channel(items[i]->text().toStdString(), reply), "create ach channel");
    }
}

void ManagerWidget::destroy_chan()
{
    QList<QListWidgetItem*> items = _ui->chan_list->selectedItems();

    for(int i=0; i<items.size(); ++i)
    {
        _set_status(_req->close_ach_channel(items[i]->text().toStdString()), "destroy ach channel");
    }
}

void ManagerWidget::launch_proc()
{
    QList<QListWidgetItem*> items = _ui->proc_list->selectedItems();

    for(int i=0; i<items.size(); ++i)
    {
        _set_status(_req->run_process(items[i]->text().toStdString()), "launch process");
    }
}

void ManagerWidget::stop_proc()
{
    QList<QListWidgetItem*> items = _ui->proc_list->selectedItems();

    for(int i=0; i<items.size(); ++i)
    {
        _set_status(_req->stop_process(items[i]->text().toStdString()), "stop process");
    }
}

void ManagerWidget::kill_proc()
{
    QList<QListWidgetItem*> items = _ui->proc_list->selectedItems();

    for(int i=0; i<items.size(); ++i)
    {
        _set_status(_req->kill_process(items[i]->text().toStdString()), "kill process");
    }
}

void ManagerWidget::inform_disconnect(int exit_code)
{
    _check_if_achds_running(_mngr_achd_handles, exit_code);
    _check_if_achds_running(_main_achd_handles, exit_code);
}

void ManagerWidget::_check_if_achds_running(AchdPtrArray &achds, int exit_code)
{
    for(int i=0; i<achds.size(); ++i)
    {
        if( (achds[i]->achd_process.state() == QProcess::NotRunning) && achds[i]->started )
        {
            _ui->network_status->setText("Disconnected!");
            _connected = false;
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
    _connected = true;
    for(int i=0; i<achds.size(); ++i)
    {
        achds[i]->start(_ui->hostname_edit->text());
    }
}

void ManagerWidget::start_all_achds()
{
    _start_achds(_mngr_achd_handles);
    _start_achds(_main_achd_handles);
}

ManagerWidget::~ManagerWidget()
{
    local_mgr_stop();

    _clear_achd_handles(_mngr_achd_handles);
    _clear_achd_handles(_main_achd_handles);
    
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

void ManagerWidget::local_mgr_launch()
{
    disconnect_all_achds();
    _lmgr.start();
}

void ManagerWidget::local_mgr_stop()
{
    emit stop_lmgr();
    _lmgr.wait(1000);
}

void ManagerWidget::save_hostname(const QString& new_name)
{
    QFile hostname(anw_save_hostname_file);
    if(!hostname.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        std::cout << "Unable to save hostname file. Maybe a permissions issue?" << std::endl;
        return;
    }
    
    hostname.write(new_name);
    
    hostname.close();
}


void LocalManager::run()
{
    _okay = true;

//    FILE* lfp = fopen("/opt/hubo/rt/lock/test_lockfile", "w");
//    if( lfp == NULL )
//    {
//        std::cout << "Unable to create lock file" << std::endl;
//        return;
//    }

//    fprintf(lfp, "%d", 100);
//    fclose(lfp);

    HuboRT::Manager mgr;
    std::cout << "Starting local manager" << std::endl;
    while(_okay)
    {
        mgr.step(0.2);
    }
    std::cout << "Quitting local manager" << std::endl;

}

void LocalManager::stop()
{
    _okay = false;
}






