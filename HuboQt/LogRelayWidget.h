#ifndef LOGRELAYWIDGET_H
#define LOGRELAYWIDGET_H

#include <QWidget>
#include <QThread>
#include <QString>
#include <QMap>
#include <QTextEdit>
#include <QTabWidget>
#include <QProxyStyle>

namespace HuboQt {


class LogHandle
{
public:

    inline LogHandle()
    {
        text_field = NULL;
        tab_position = -1;
    }

    QTextEdit* text_field;
    int tab_position;

};

typedef QMap<QString,LogHandle> LogMap;

class LogRelayTabs : public QTabWidget
{
    Q_OBJECT

public:

    LogRelayTabs();

protected:

    LogMap _map;

    void _notify_new_content(int index);
    void _create_log_display(const QString& log_name);

protected Q_SLOTS:

    void handle_new_log_message(const QString& log, const QString& message);
    void clear_content_notice(int index);


};

class RelayWorker : public QThread
{
    Q_OBJECT

public:

    explicit RelayWorker(LogRelayTabs* master);

    void run();

signals:

    void new_log_message(const QString& log, const QString& message);

protected:

};

class LogRelayWidget : public QWidget
{
    Q_OBJECT

public:

    LogRelayWidget();

protected:

    LogRelayTabs tabs;
    RelayWorker* worker;

public Q_SLOTS:

    void attempt_restart();

};

// Largely inspired by http://qtcenter.org/wiki/index.php?title=Customizing_QTabWidget%27s_QTabBar
class LogNameStyle : public QProxyStyle
{
public:

    QSize sizeFromContents(ContentsType type,
                           const QStyleOption *option,
                           const QSize &size,
                           const QWidget *widget) const;

    void drawControl(ControlElement element,
                     const QStyleOption *option,
                     QPainter *painter,
                     const QWidget *widget) const;
};

} // namespace HuboQt

#endif // LOGRELAYWIDGET_H
