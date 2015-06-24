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

#ifndef HUBOQT_CONFIGWIDGET_H
#define HUBOQT_CONFIGWIDGET_H

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
    void delete_config();

    void refresh_lists();

};

}

#endif // HUBOQT_CONFIGWIDGET_H
