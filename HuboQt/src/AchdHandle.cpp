
#include "../AchdHandle.h"

#include "HuboRT/utils.hpp"

#include <iostream>

using namespace HuboQt;

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
        started = true;
        if(achd_process.state() == QProcess::Running
                || achd_process.state() == QProcess::Starting)
        {
            std::cout << "Killing the achd process for channel "
                      << nickname.toStdString()
                      << " in order to restart it." << std::endl;
            achd_process.kill();
        }
        
        achd_process.start("ach mk " + channel_name + " -1 "
                           + "-m " + QString::number(message_count)
                           + " -n " + QString::number(nominal_size)
                           + " -o 666", QIODevice::ReadWrite);
        
        achd_process.waitForFinished(5000);
        if(achd_process.state() != QProcess::NotRunning)
        {
            std::cout << "QProcess is hung up on attempting to create ach channel '"
                         << channel_name.toStdString() << "'!" << std::endl;
            return false;
        }
        
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
    achd_process.kill();
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
    
    return false;
}


