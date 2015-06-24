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

#include <iostream>

#include <QTime>

#include "HuboQt/AchdHandle.h"
#include "HuboRT/utils.hpp"

namespace HuboQt {

AchdHandle::AchdHandle()
{
    started = false;
}

bool AchdHandle::start(QString hostname)
{
    robot_hostname = hostname;
    
    QString start_type = "";
    if(ACHD_PULL_FROM_ROBOT == push_or_pull)
    {
        start_type = "achd pull ";
    }
    else if(ACHD_PUSH_TO_ROBOT == push_or_pull)
    {
        start_type = "achd push ";
    }
    
    if(start_type.size() > 0)
    {
        if(achd_process.state() == QProcess::Running
                || achd_process.state() == QProcess::Starting)
        {
            std::cout << "Killing the achd process for channel "
                      << nickname.toStdString()
                      << " in order to restart it." << std::endl;
            stop();
        }

        started = true;
        achd_process.start(start_type + hostname + " " + channel_name);
    }
    
    return true;
}

bool AchdHandle::start(QString hostname, QString description)
{
    if(!parse_description(description))
        return false;
    
    return start(hostname);
}

void AchdHandle::stop()
{
    started = false;
    if(achd_process.state() != QProcess::NotRunning)
    {
        std::cout << "About to kill " << nickname.toStdString()
                  << " | " << QTime::currentTime().toString().toStdString() << std::endl;
        achd_process.kill();
//        if(!achd_process.waitForFinished(200))
//        {
//            std::cout << "Had to wait for " << nickname.toStdString()
//                      << " | " << QTime::currentTime().toString().toStdString() << std::endl;
//            if(achd_process.state() != QProcess::NotRunning)
//                std::cout << "Achd is hung up on quitting for '"
//                          << nickname.toStdString() << "'!"
//                          << " | " << QTime::currentTime().toString().toStdString() << std::endl;
//        }
    }
}

AchdHandle::~AchdHandle()
{
    stop();
}

bool AchdHandle::parse_description(QString description)
{
    channel_description = description;
    
    StringArray components;
    if(HuboRT::split_components(description.toStdString(), components) < 5)
    {
        std::cerr << "Invalid ach channel description: " << description.toStdString() << std::endl;
        return false;
    }
    
    nickname = components[0].c_str();
    channel_name = components[1].c_str();
    message_count = atoi(components[2].c_str());
    nominal_size = atoi(components[3].c_str());
    
    std::string achd_type = components[4];
    if(ACHD_INTERNAL_STRING == achd_type)
    {
        push_or_pull = ACHD_NOTHING;
    }
    else if(ACHD_PULL_STRING == achd_type)
    {
        push_or_pull = ACHD_PULL_FROM_ROBOT;
    }
    else if(ACHD_PUSH_STRING == achd_type)
    {
        push_or_pull = ACHD_PUSH_TO_ROBOT;
    }
    else
    {
        std::cerr << "Invalid achd networking type: " << achd_type << std::endl;
        return false;
    }

    QProcess ach_make;
    ach_make.start("ach mk " + channel_name + " -1 "
                       + "-m " + QString::number(message_count)
                       + " -n " + QString::number(nominal_size)
                       + " -o 666", QIODevice::ReadWrite);

    if(!ach_make.waitForFinished(5000))
    {
        std::cout << "QProcess is hung up on attempting to create ach channel '"
                     << channel_name.toStdString() << "'!" << std::endl;
    }
    
    return true;
}

} // namespace HuboQt
