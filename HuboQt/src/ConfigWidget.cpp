
#include "../ConfigWidget.h"
#include "HuboRT/utils.hpp"

using namespace HuboQt;

ConfigWidget::ConfigWidget() :
    _ui(new Ui::ConfigWidget)
{
    _req = NULL;
    _ui->setupUi(this);

    connect(_ui->refreshLists, SIGNAL(clicked()), this, SLOT(refresh_lists()));

    connect(_ui->registerChan, SIGNAL(clicked()), this, SLOT(register_chan()));
    connect(_ui->unregisterChan, SIGNAL(clicked()), this, SLOT(unregister_chan()));

    connect(_ui->registerProc, SIGNAL(clicked()), this, SLOT(register_proc()));
    connect(_ui->unregisterProc, SIGNAL(clicked()), this, SLOT(unregister_proc()));

    connect(_ui->loadConfig, SIGNAL(clicked()), this, SLOT(load_config()));
    connect(_ui->saveConfig, SIGNAL(clicked()), this, SLOT(save_config()));
    connect(_ui->deleteConfig, SIGNAL(clicked()), this, SLOT(delete_config()));

    _ui->chanTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    _ui->procTable->setSelectionBehavior(QAbstractItemView::SelectRows);

    _ui->configList->setSelectionMode(QAbstractItemView::SingleSelection);
}

ConfigWidget::~ConfigWidget()
{
    delete _ui;
}

void ConfigWidget::_set_status(manager_err_t incoming_status, const QString &status_context)
{
    if(incoming_status != NO_ERROR && incoming_status != EMPTY_LIST)
    {
        std::cerr << "Error during '" << status_context.toStdString() << "': "
                  << manager_err_to_string(incoming_status) << std::endl;
    }

    _ui->reply_status->setText(QString::fromStdString(manager_err_to_string(incoming_status))
                               + " (" + status_context + ")");
}

void ConfigWidget::_parse_descriptions(const StringArray& descs, QTableWidget* table,
                                       size_t expectation, const std::string& type)
{
    StringArray components;
    table->clearContents();
    while(table->rowCount() > 0)
        table->removeRow(0);

    for(size_t i=0; i<descs.size(); ++i)
    {
        if(HuboRT::split_components(descs[i], components) < expectation)
        {
            std::cerr << "Invalid " << type << " description: " << descs[i] << std::endl;
            continue;
        }

        table->insertRow(i);
        for(size_t j=0; j<components.size(); ++j)
        {
            QTableWidgetItem* item = new QTableWidgetItem(QString::fromStdString(components[j]));
            table->setItem(i,j,item);
        }
    }
}

void ConfigWidget::_parse_channel_descriptions(const StringArray &descs)
{
    _parse_descriptions(descs, _ui->chanTable, 5, "ach channel");
}

void ConfigWidget::_parse_proc_descriptions(const StringArray& descs)
{
    _parse_descriptions(descs, _ui->procTable, 3, "process");
}

void ConfigWidget::_parse_configurations(const StringArray &configs)
{
    QListWidget* list = _ui->configList;
    list->clear();
    for(size_t i=0; i<configs.size(); ++i)
    {
        QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(configs[i]));
        list->addItem(item);
    }
}

void ConfigWidget::initialize()
{
    _req = new HuboRT::ManagerReq;
}

void ConfigWidget::register_chan()
{
    if( NULL == _req )
        return;

    manager_err_t result =
            _req->register_new_channel(_ui->chanNameEdit->text().toStdString(),
                               _ui->chanChanEdit->text().toStdString(),
                               (achd_network_t)_ui->chanAchdType->currentIndex(),
                               (size_t)_ui->chanMsgCount->value(),
                               (size_t)_ui->chanMsgSize->value());
    _set_status(result, "Register Channel");
    _quiet_update_chans();
}

void ConfigWidget::unregister_chan()
{
    if( NULL == _req )
        return;

    int row = _ui->chanTable->currentRow();
    if(row == -1)
        return;

    _set_status(_req->unregister_old_channel(_ui->chanTable->item(row,0)->text().toStdString()),
                "Unregister Channel");
    _quiet_update_chans();
}

void ConfigWidget::register_proc()
{
    if( NULL == _req )
        return;

    manager_err_t result =
            _req->register_new_process(_ui->procNameEdit->text().toStdString(),
                                       _ui->procProcEdit->text().toStdString(),
                                       _ui->procArgsEdit->text().toStdString());
    _set_status(result, "Register Process");
    _quiet_update_procs();
}

void ConfigWidget::unregister_proc()
{
    if( NULL == _req )
        return;

    int row = _ui->procTable->currentRow();
    if(row == -1)
        return;

    _set_status(_req->unregister_old_process(_ui->procTable->item(row, 0)->text().toStdString()),
                "Unregister Process");
    _quiet_update_procs();
}

void ConfigWidget::load_config()
{
    if( NULL == _req )
        return;

    _req->timeout = 5;
    _set_status(_req->load_config(_ui->configList->currentItem()->text().toStdString()),
                "Load Configuration");
    _req->timeout = 1;
    _quiet_update_chans();
    _quiet_update_procs();
}

void ConfigWidget::save_config()
{
    if( NULL == _req )
        return;

    if(_ui->saveConfigEdit->text().isEmpty())
        return;

    _set_status(_req->save_current_config(_ui->saveConfigEdit->text().toStdString()),
                "Save Configuration");
    _quiet_update_configs();
}

void ConfigWidget::delete_config()
{
    if( NULL == _req )
        return;

    _set_status(_req->delete_config(_ui->configList->currentItem()->text().toStdString()),
                "Delete Configuration");
    _quiet_update_configs();
}

void ConfigWidget::_quiet_update_chans()
{
    StringArray reply;
    manager_err_t result = _req->list_channels(reply);
    if(result != NO_ERROR && result != EMPTY_LIST)
        return;
    _parse_channel_descriptions(reply);
}

void ConfigWidget::_quiet_update_procs()
{
    StringArray reply;
    manager_err_t result = _req->list_registered_processes(reply);
    if(result != NO_ERROR && result != EMPTY_LIST)
        return;
    _parse_proc_descriptions(reply);
}

void ConfigWidget::_quiet_update_configs()
{
    StringArray reply;
    manager_err_t result = _req->list_configs(reply);
    if(result != NO_ERROR && result != EMPTY_LIST)
        return;
    _parse_configurations(reply);
}

void ConfigWidget::refresh_lists()
{
    if( NULL == _req )
        return;

    StringArray reply;
    manager_err_t result = _req->list_channels(reply);
    _set_status(result, "get channel regsitry");
    if(result != NO_ERROR && result != EMPTY_LIST)
        return;
    _parse_channel_descriptions(reply);

    result = _req->list_registered_processes(reply);
    _set_status(result, "get process registry");
    if(result != NO_ERROR && result != EMPTY_LIST)
        return;
    _parse_proc_descriptions(reply);

    result = _req->list_configs(reply);
    _set_status(result, "get configuration registry");
    if(result != NO_ERROR && result != EMPTY_LIST)
        return;
    _parse_configurations(reply);
}
