/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <greyxmike@gmail.com>
 *
 * Humanoid Robotics Lab
 *
 * Directed by Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HUBOQT_LOGRELAYWIDGET_H
#define HUBOQT_LOGRELAYWIDGET_H

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

#endif // HUBOQT_LOGRELAYWIDGET_H
