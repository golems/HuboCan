#ifndef HUBOQTMANAGERWIDGET_H
#define HUBOQTMANAGERWIDGET_H

#include <QWidget>

#include <vector>
#include <QVector>

#include "ui_ManagerWidget.h"
#include "AchdHandle.h"

#include "HuboRT/ManagerReq.hpp"

namespace HuboQt {

typedef QVector<AchdHandle*> AchdPtrArray;

class ManagerWidget : public QWidget
{
    Q_OBJECT
    
public:
    ManagerWidget();
    ~ManagerWidget();
    
    void load_hostname();
    
protected:
    
    Ui::ManagerWidget* _ui;
    
    HuboRT::ManagerReq* _req;
    
    AchdPtrArray _perm_achd_handles;    // This is for the Manager's channels which will
                                        // always be exactly the same
    
    AchdPtrArray _more_achd_handles;    // This is for the rest of the channels which we
                                        // will depend on the Manager to inform us about
    
    void _parse_channel_descriptions(const StringArray& descs);
    void _display_channels();
    
    void _start_achds(AchdPtrArray& achds);
    void _disconnect_achds(AchdPtrArray& achds);
    
    void _check_if_achds_running(AchdPtrArray& achds, int exit_code);
    
protected Q_SLOTS:
    
    void save_hostname(const QString& new_name);
    
    void start_all_achds();
    void disconnect_all_achds();
    
    void startup_everything();
    void shutdown_everything();
    void homeall();
    
    
    
    void inform_disconnect(int exit_code);
    
private:
    
    inline ManagerWidget(const ManagerWidget& doNotCopy) { }
    inline ManagerWidget& operator=(const ManagerWidget& doNotCopy) { return *this; }
    
};

} // namespace HuboQt

#endif // HUBOQTMANAGERWIDGET_H
