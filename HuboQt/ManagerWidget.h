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

#ifndef HUBOQT_MANAGERWIDGET_H
#define HUBOQT_MANAGERWIDGET_H

#include <QWidget>

#include <vector>
#include <QVector>
#include <QThread>

#include "ui_ManagerWidget.h"
#include "AchdHandle.h"

#include "HuboRT/ManagerReq.hpp"
#include "HuboCmd/AuxSender.hpp"

namespace HuboQt {

typedef QVector<AchdHandle*> AchdPtrArray;

class LocalManager : public QThread
{
    Q_OBJECT

public:

    void run();

protected:

    bool _okay;

protected Q_SLOTS:

    void stop();

};

class ManagerWidget : public QWidget
{
    Q_OBJECT
    
public:
    ManagerWidget();
    ~ManagerWidget();
    
    void load_hostname();

    void set_displaying_locked_procs(bool displaying_locked_procs);
    QString selected_button_style;
    
protected:

    LocalManager _lmgr;
    
    bool _displaying_locked_procs;
    bool _connected;

    
    Ui::ManagerWidget* _ui;
    
    HuboRT::ManagerReq* _req;
    HuboCmd::AuxSender _init;
    
    AchdPtrArray _mngr_achd_handles;    // This is for the Manager's channels which will
                                        // always be exactly the same
    
    AchdPtrArray _main_achd_handles;    // This is for the rest of the channels which we
                                        // will depend on the Manager to inform us about
    
    bool _double_check_init();
    
    void _clear_achd_handles(AchdPtrArray& achds);
    void _parse_channel_descriptions(const StringArray& descs);
    void _display_channels();
    
    void _display_registered_processes(const StringArray& procs);
    void _display_locked_processes(const StringArray& locks);
    
    void _start_achds(AchdPtrArray& achds);
    void _disconnect_achds(AchdPtrArray& achds);
    
    void _check_if_achds_running(AchdPtrArray& achds, int exit_code);
    
    void _set_status(manager_err_t incoming_status, const QString& status_context);
    
protected Q_SLOTS:

    void local_mgr_launch();
    void local_mgr_stop();
    
    void save_hostname(const QString& new_name);
    
    void start_all_achds();
    void disconnect_all_achds();
    
    void startup_everything();
    void shutdown_everything();
    void homeall();
    
    void create_all();
    void destroy_all();
    void launch_all();
    void stop_all();
    void kill_all();
    
    void refresh_chans();
    void refresh_registered_procs();
    void refresh_registered_procs_raw();
    void refresh_locked_procs();
    void refresh_locked_procs_raw();
    void timer_refresh();

    void create_chan();
    void destroy_chan();
    void launch_proc();
    void stop_proc();
    void kill_proc();

    void deactivate_network();
    void activate_network();
    
    void inform_disconnect(int exit_code);

Q_SIGNALS:

    void channels_created();
    void manager_channels_created();

    void stop_lmgr();
    
private:
    
    ManagerWidget(const ManagerWidget& doNotCopy);
    ManagerWidget& operator=(const ManagerWidget& doNotCopy);
    
};

} // namespace HuboQt

#endif // HUBOQT_MANAGERWIDGET_H
