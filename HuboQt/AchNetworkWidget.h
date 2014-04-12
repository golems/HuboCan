#ifndef ACHNETWORKWIDGET_H
#define ACHNETWORKWIDGET_H

#include <QWidget>
#include "ui_AchNetworkWidget.h"

namespace HuboQt {


class AchNetworkWidget : public QWidget
{
    Q_OBJECT
    
public:
    AchNetworkWidget();
    
    void load_hostname();
    
protected:
    
    Ui::AchNetworkWidget* _ui;
    
protected Q_SLOTS:
    void save_hostname(const QString& new_name);
    
};

} // namespace HuboQt

#endif // ACHNETWORKWIDGET_H
