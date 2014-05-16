
#include "../ConfigWidget.h"
#include "HuboRT/utils.hpp"

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
            std::cout << components[j] << "\t";
            QTableWidgetItem* item = new QTableWidgetItem(QString::fromStdString(components[j]));
            table->setItem(i,j,item);
        }
        std::cout << std::endl;
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
}

void ConfigWidget::unregister_chan()
{
    if( NULL == _req )
        return;


}

void ConfigWidget::register_proc()
{
    if( NULL == _req )
        return;


}

void ConfigWidget::unregister_proc()
{
    if( NULL == _req )
        return;


}

void ConfigWidget::load_config()
{
    if( NULL == _req )
        return;


}

void ConfigWidget::save_config()
{
    if( NULL == _req )
        return;


}

void ConfigWidget::refresh_lists()
{
    if( NULL == _req )
        return;
    std::cout << "refreshing lists" << std::endl;

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
