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

#include "HuboQt/HuboQtWidget.h"

namespace HuboQt {

HuboQtWidget::HuboQtWidget()
{
    mgr = new ManagerWidget;
    joints = new JointWidget;
    relay = new LogRelayWidget;
    configs = new ConfigWidget;

    connect(mgr, SIGNAL(channels_created()), joints, SLOT(initialize()));
    connect(mgr, SIGNAL(channels_created()), relay, SLOT(attempt_restart()));
    configs->initialize();

    addTab(mgr, "Manager");
    addTab(joints, "Joints");
    addTab(relay, "Logs");
    addTab(configs, "Configs");

    connect(this, SIGNAL(currentChanged(int)), this, SLOT(handle_tab_change(int)));
    connect(this, SIGNAL(refresh_configs()), configs, SLOT(refresh_lists()));
}

void HuboQtWidget::handle_tab_change(int new_tab)
{
    if( widget(new_tab) == configs )
    {
        emit refresh_configs();
    }
}

} // namespace HuboQt
