
#include "../LogRelayWidget.h"
#include "HuboRT/LogRelay.hpp"
#include <string>
#include <iostream>
#include <QVBoxLayout>
#include <QPushButton>
#include <QToolButton>
#include <QPainter>
#include <QStyleOptionTab>

using namespace HuboQt;

RelayWorker::RelayWorker(LogRelayTabs *master)
{
    connect(this, SIGNAL(new_log_message(QString,QString)), master,
            SLOT(handle_new_log_message(QString,QString)));
}

void RelayWorker::run()
{
    HuboRT::LogRelay relay;
    if(!relay.open_channels())
        return;

    std::string log;
    std::string message;

    while(true)
    {
        if(relay.receive(log, message))
        {
            emit new_log_message(QString::fromStdString(log),
                                 QString::fromStdString(message));
        }
    }
}

LogRelayTabs::LogRelayTabs()
{
    QTabBar* bar = new QTabBar;
    bar->setStyle(new LogNameStyle);
    setTabBar(bar);
    setTabPosition(QTabWidget::West);
    connect(this, SIGNAL(currentChanged(int)), this, SLOT(clear_content_notice(int)));
}

void LogRelayTabs::handle_new_log_message(const QString& log, const QString& message)
{
    LogMap::iterator it = _map.find(log);
    if(it == _map.end())
    {
        _create_log_display(log);
        it = _map.find(log);
        if(it == _map.end())
        {
            std::cerr << "Inexplicable error creating a new log display!! REPORT BUG IMMEDIATELY"
                      << std::endl;
            return;
        }
    }

    if( (it->text_field != NULL) && (it->tab_position != -1) )
    {
        it->text_field->setText(it->text_field->text() + message);
        _notify_new_content(it->tab_position);
    }
    else
    {
        std::cerr << "LogHandle not initialized correctly!! REPORT BUG IMMEDIATELY" << std::endl;
    }
}

void LogRelayTabs::_notify_new_content(int index)
{
    if(currentIndex() == index)
    {
        clear_content_notice(index);
    }
    else
    {
        QString title = tabText(index);
        if(title.at(0) != '*')
        {
            title.insert(0, '*');
            setTabText(index, title);
        }
    }
}

void LogRelayTabs::clear_content_notice(int index)
{
    QString title = tabText(index);
    if(title.at(0) == '*')
    {
        title.remove(0,1);
        setTabText(index, title);
    }
}

void LogRelayTabs::_create_log_display(const QString &log_name)
{
    LogHandle new_handle;
    QTextEdit* new_field = new QTextEdit;
    new_handle.text_field = new_field;

    addTab(new_field, log_name);
    new_handle.tab_position = count()-1;

    _map[log_name] = new_handle;
}

LogRelayWidget::LogRelayWidget()
{
    setLayout(new QVBoxLayout);
    layout()->addWidget(&tabs);

    QToolButton* restart = new QToolButton;
    restart->setText("Restart");
    restart->setToolTip("Restart the log reader\n"
                        "(Use this button if ach channels needed to be recreated)");
    connect(restart, SIGNAL(clicked()), this, SLOT(attempt_restart()));
    layout()->addWidget(restart);

    worker = new RelayWorker(&tabs);
    worker->start();
}

void LogRelayWidget::attempt_restart()
{
    worker->quit();
    worker->start();
}


QSize LogNameStyle::sizeFromContents(ContentsType type,
                                     const QStyleOption *option,
                                     const QSize &size,
                                     const QWidget *widget) const
{
    QSize s = QProxyStyle::sizeFromContents(type, option, size, widget);

    if( QStyle::CT_TabBarTab == type )
    {
        s.transpose();
    }

    return s;
}

void LogNameStyle::drawControl(ControlElement element,
                               const QStyleOption *option,
                               QPainter *painter,
                               const QWidget *widget) const
{
    if(CE_TabBarTabLabel == element)
    {
        if(const QStyleOptionTab* tab = qstyleoption_cast<const QStyleOptionTab*>(option))
        {
            QStyleOptionTab opt(*tab);
            opt.shape = QTabBar::RoundedNorth;

            return QProxyStyle::drawControl(element, &opt, painter, widget);
        }
    }
    QProxyStyle::drawControl(element, option, painter, widget);
}
