#ifndef CONFIGWIDGET_H
#define CONFIGWIDGET_H

#include "HuboRT/ManagerReq.hpp"

#include "ui_ConfigWidget.h"

namespace HuboQt {

class ConfigWidget : public QWidget
{
    Q_OBJECT

public:

    ConfigWidget();
    ~ConfigWidget();

protected:

    HuboRT::ManagerReq* _req;

    Ui::ConfigWidget* _ui;

    void _set_status(manager_err_t incoming_status, const QString& status_context);

    void _parse_channel_descriptions(const StringArray& descs);
    void _parse_proc_descriptions(const StringArray& descs);
    void _parse_descriptions(const StringArray& descs, QTableWidget* table, size_t expectation, const std::string& type);

    void _parse_configurations(const StringArray& configs);

    void _quiet_update_chans();
    void _quiet_update_procs();
    void _quiet_update_configs();

public Q_SLOTS:

    void initialize();

    void register_chan();
    void unregister_chan();

    void register_proc();
    void unregister_proc();

    void load_config();
    void save_config();

    void refresh_lists();

};

}

#endif // CONFIGWIDGET_H
