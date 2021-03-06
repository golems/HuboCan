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

extern "C" {
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
}

#include <sstream>
#include <iostream>

#include "HuboCan/HuboDescription.hpp"

#include "HuboCan/HuboJoint.hpp"
#include "HuboCan/DeviceStrings.hpp"

namespace HuboCan {

HuboDescription::HuboDescription()
    : _okay(false),
      _numImus(0),
      _numFts(0),
      _data(NULL)
{
    memset(&params, 0, sizeof(params));
}

HuboDescription::HuboDescription(const HuboDescription &desc)
{
    _copy_description(desc);
}

HuboDescription& HuboDescription::operator=(const HuboDescription& desc)
{
    free(_data);
    _copy_description(desc);

    return *this;
}

void HuboDescription::_copy_description(const HuboDescription &desc)
{
    _clear_memory();

    for(size_t i=0; i<desc.joints.size(); ++i)
    {
        HuboJoint* newJoint = new HuboJoint(*desc.joints[i]);
        joints.push_back(newJoint);
    }

    for(size_t i=0; i<desc.jmcs.size(); ++i)
    {
        HuboJmc* newJmc = new HuboJmc(*desc.jmcs[i]);
        jmcs.push_back(newJmc);
    }

    for(size_t i=0; i<desc.sensors.size(); ++i)
    {
        HuboSensor* newSensor = new HuboSensor(*desc.sensors[i]);
        sensors.push_back(newSensor);
    }

    params = desc.params;

    _okay = true;

    _data = NULL;
}

HuboDescription::~HuboDescription()
{
    free(_data);

    _clear_memory();
}

void HuboDescription::_clear_memory()
{
    for(size_t i=0; i<joints.size(); ++i)
    {
        delete joints[i];
    }
    joints.clear();

    for(size_t i=0; i<jmcs.size(); ++i)
    {
        delete jmcs[i];
    }
    jmcs.clear();

    for(size_t i=0; i<sensors.size(); ++i)
    {
        delete sensors[i];
    }
    sensors.clear();

    memset(&params, 0, sizeof(params));
}

error_result_t HuboDescription::receiveInfo(double timeout_sec)
{
    _okay = false;
    free(_data);
    _data = hubo_info_receive_data(timeout_sec);

    if(_data == NULL)
        return ACH_ERROR;

    size_t joint_count = hubo_info_get_joint_count(_data);
    for(size_t i=0; i < joint_count; ++i)
    {
        HuboJoint* newJoint = new HuboJoint;
        newJoint->info = *hubo_info_get_joint_info(_data, i);
        joints.push_back(newJoint);
    }

    size_t jmc_count = hubo_info_get_jmc_count(_data);
    for(size_t i=0; i < jmc_count; ++i)
    {
        HuboJmc* newJmc = new HuboJmc;
        newJmc->info = *hubo_info_get_jmc_info(_data, i);
        jmcs.push_back(newJmc);
    }

    size_t sensor_count = hubo_info_get_sensor_count(_data);
    for(size_t i=0; i < sensor_count; ++i)
    {
        _constructSensor(*hubo_info_get_sensor_info(_data, i), false);
    }

    memcpy(&params, hubo_info_get_params_info(_data), sizeof(params));

    free(_data);
    _data = NULL;

    _okay = true;

    return OKAY;
}

error_result_t HuboDescription::broadcastInfo()
{
    free(_data);
    _data = hubo_info_init_data(joints.size(), jmcs.size(), sensors.size());

    for(size_t i=0; i < joints.size(); ++i)
    {
        size_t loc = hubo_info_get_joint_location(/*_data,*/ i);
        memcpy(_data+loc, &(joints[i]->info), sizeof(hubo_joint_info_t));
    }

    for(size_t i=0; i < jmcs.size(); ++i)
    {
        size_t loc = hubo_info_get_jmc_location(_data, i);
        memcpy(_data+loc, &(jmcs[i]->info), sizeof(hubo_jmc_info_t));
    }

    for(size_t i=0; i < sensors.size(); ++i)
    {
        size_t loc = hubo_info_get_sensor_location(_data, i);
        memcpy(_data+loc, &(sensors[i]->info), sizeof(hubo_sensor_info_t));
    }

    memcpy(hubo_info_get_params_info(_data), &params, sizeof(params));

    int result = hubo_info_send_data(_data);

    free(_data);
    _data = NULL;

    if(result==0)
        return OKAY;
    else
        return ACH_ERROR;
}

std::string HuboDescription::getJointTable() const
{
    std::stringstream table;

    table << HuboJoint::header() << "\n";

    for(size_t i=0; i < joints.size(); ++i)
    {
        table << (*joints[i]) << "\n";
    }

    return table.str();
}

std::string HuboDescription::getJmcTable() const
{
    std::stringstream table;

    table << HuboJmc::header() << "\n";

    for(size_t i=0; i < jmcs.size(); ++i)
    {
        table << (*jmcs[i]) << "\n";
    }

    return table.str();
}

bool HuboDescription::parseFile(const std::string& filename)
{
    _okay = false;
    if(!_parser.load_file(filename))
        return false;

    StringArray components;
    while(_parser.status() != HuboCan::DD_END_FILE && _parser.status() != HuboCan::DD_ERROR)
    {
        _parser.next_line(components);
        if(_parser.status() == HuboCan::DD_BEGIN_DEVICE)
        {
            if(!_parseDevice(components[1]))
                return false;
        }
    }

    return _postParseProcessing();
}

void HuboDescription::assignAggregator(HuboCmd::Aggregator* agg)
{
    for(size_t i=0; i<jmcs.size(); ++i)
        jmcs[i]->assignAggregator(agg);

    for(size_t i=0; i<sensors.size(); ++i)
        sensors[i]->assignAggregator(agg);
}

void HuboDescription::assignState(HuboState::State* state)
{
    for(size_t i=0; i<jmcs.size(); ++i)
        jmcs[i]->assignState(state);

    for(size_t i=0; i<sensors.size(); ++i)
        sensors[i]->assignState(state);
}

bool HuboDescription::_parseDevice(const std::string& device_type)
{
    if(joint_device_string == device_type)
    {
        if(!_parseJoint())
            return false;
    }
    else if(jmc_device_string == device_type)
    {
        if(!_parseJMC())
            return false;
    }
    else if(imu_sensor_string == device_type)
    {
        if(!_parseIMU())
            return false;
    }
    else if(ft_sensor_string == device_type)
    {
        if(!_parseForceTorque())
            return false;
    }
    else if(meta_device_string == device_type)
    {
        if(!_parseMeta())
            return false;
    }

    return true;
}

bool HuboDescription::_parseJoint(bool strict)
{
    size_t joint_index = size_t(-1);
    hubo_joint_info_t new_joint_info;
    memset(&new_joint_info, 0, sizeof(hubo_joint_info_t));

    StringArray components;
    while(_parser.next_line(components) == HuboCan::DD_OKAY)
    {
        if(components.size() < 2)
        {
            _parser.error() << "Every component must have at least one argument!";
            _parser.report_error();
        }

        if(     "name" == components[0])
        {
            if(components[1].size() > HUBO_COMPONENT_NAME_MAX_LENGTH)
            {
                _parser.error() << "Component name string overflow! Max size is " << HUBO_COMPONENT_NAME_MAX_LENGTH;
                _parser.report_error();
            }
            else
            {
                strcpy(new_joint_info.name, components[1].c_str());
            }
        }
        else if("drive" == components[0])
        {
            new_joint_info.drive_factor = atof(components[1].c_str());
        }
        else if("driven" == components[0])
        {
            new_joint_info.driven_factor = atof(components[1].c_str());
        }
        else if("harmonic" == components[0])
        {
            new_joint_info.harmonic_factor = atof(components[1].c_str());
        }
        else if("encoder" == components[0])
        {
            new_joint_info.enc_resolution = atof(components[1].c_str());
        }
        else if("default_kp" == components[0])
        {
            new_joint_info.default_kp = atof(components[1].c_str());
        }
        else if("default_kd" == components[0])
        {
            new_joint_info.default_max_pwm = atof(components[1].c_str());
        }
        else if("default_max_pwm" == components[0])
        {
            new_joint_info.default_max_pwm = atof(components[1].c_str());
        }
        else if("hardware_index" == components[0])
        {
            new_joint_info.hardware_index = atof(components[1].c_str());
        }
        else if("min_max_positions" == components[0])
        {
            if(components.size() < 3)
            {
                _parser.error() << "min_max_positions must give both a minimum and a maximum value!";
                _parser.report_error();
            }
            else
            {
                new_joint_info.limits.min_position = atof(components[1].c_str());
                new_joint_info.limits.max_position = atof(components[2].c_str());
            }
        }
        else if("nominal_and_max_speeds" == components[0])
        {
            if(components.size() < 3)
            {
                _parser.error() << "nominal_and_max_speeds must give both a nominal and a maximum value!";
                _parser.report_error();
            }
            else
            {
                new_joint_info.limits.nominal_speed = atof(components[1].c_str());
                new_joint_info.limits.max_speed = atof(components[2].c_str());
            }
        }
        else if("nominal_and_max_accels" == components[0])
        {
            if(components.size() < 3)
            {
                _parser.error() << "nominal_and_max_accels must give both a nominal and a maximum value!";
                _parser.report_error();
            }
            else
            {
                new_joint_info.limits.nominal_accel = atof(components[1].c_str());
                new_joint_info.limits.max_accel = atof(components[2].c_str());
            }
        }
        else if("jmc_name" == components[0])
        {
            if(components[1].size() > HUBO_COMPONENT_NAME_MAX_LENGTH)
            {
                _parser.error() << "Component name string overflow! Max size is " << HUBO_COMPONENT_NAME_MAX_LENGTH;
                _parser.report_error();
            }
            else
            {
                strcpy(new_joint_info.jmc_name, components[1].c_str());
            }
        }
        else if("software_index" == components[0])
        {
            joint_index = atoi(components[1].c_str());
        }
        else
        {
            if(strict)
            {
                _parser.error() << "Invalid component: " << components[0];
                _parser.report_error();
            }
        }

        if(_parser.status() == DD_ERROR)
            return false;
    }

    if(size_t(-1) == joint_index)
    {
        _parser.error() << "software_index parameter is never specified!";
        _parser.report_error();
    }

    HuboJointPtrMap::iterator it;
    for( it = _tempJointMap.begin(); it != _tempJointMap.end(); ++it)
    {
        if(strcmp(new_joint_info.name,it->second->info.name)==0)
        {
            _parser.error() << "Repeated joint name: " << new_joint_info.name;
            _parser.report_error();
            break;
        }
    }

    HuboJointPtrMap::iterator check = _tempJointMap.find(joint_index);
    if(check != _tempJointMap.end())
    {
        _parser.error() << "Repeated software index: " << check->first;
        _parser.error() << "\n --This already belongs to a joint named '" << check->second->info.name << "'";
        _parser.report_error();
    }

    if(_parser.status() == DD_ERROR)
        return false;

    HuboJoint* new_joint = new HuboJoint;
    new_joint_info.software_index = joint_index;
    new_joint->info = new_joint_info;
    _tempJointMap[joint_index] = new_joint;

    return true;
}

bool HuboDescription::_parseJMC(bool strict)
{
    hubo_jmc_info_t new_jmc_info;
    memset(&new_jmc_info, 0, sizeof(hubo_jmc_info_t));

    StringArray components;
    while(_parser.next_line(components) == HuboCan::DD_OKAY)
    {
        if(components.size() < 2)
        {
            _parser.error() << "Every component must have at least one argument!";
            _parser.report_error();
        }

        if(     "name" == components[0])
        {
            if(components[1].size() > HUBO_COMPONENT_NAME_MAX_LENGTH)
            {
                _parser.error() << "Component name string overflow! Max size is "
                                << HUBO_COMPONENT_NAME_MAX_LENGTH;
                _parser.report_error();
            }
            else
            {
                strcpy(new_jmc_info.name, components[1].c_str());
            }
        }
        else if("type" == components[0])
        {
            if(components[1].size() > HUBO_COMPONENT_TYPE_MAX_LENGTH)
            {
                _parser.error() << "Component type string overflow! Max size is "
                                << HUBO_COMPONENT_TYPE_MAX_LENGTH;
                _parser.report_error();
            }
            else
            {
                strcpy(new_jmc_info.type, components[1].c_str());
            }
        }
        else if("can_channel" == components[0])
        {
            new_jmc_info.can_channel = atoi(components[1].c_str());
        }
        else if("hardware_index" == components[0])
        {
            new_jmc_info.hardware_index = strtol(components[1].c_str(), NULL, 0);
        }
        else
        {
            if(strict)
            {
                _parser.error() << "Invalid component: " << components[0];
                _parser.report_error();
            }
        }

        if(_parser.status() == DD_ERROR)
            return false;
    }

    for(size_t i=0; i < jmcs.size(); ++i)
    {
        if(strcmp(new_jmc_info.name, jmcs[i]->info.name) == 0)
        {
            _parser.error() << "Repated JMC name: " << new_jmc_info.name;
            _parser.report_error();
            break;
        }
    }

    if(_parser.status() == DD_ERROR)
        return false;

    _constructJMC(new_jmc_info, true);

    return true;
}

bool HuboDescription::_constructJMC(const hubo_jmc_info_t& jmc_info, bool parsed)
{
    HuboJmc* new_jmc = NULL;
    const std::string type_string(jmc_info.type);
    if(     type_string == hubo2plus_1ch_type_string ||
            type_string == hubo2plus_2ch_type_string)
    {
        new_jmc = new Hubo2Plus2chJmc;
    }
    else if(type_string == hubo2plus_nck_type_string)
    {
        new_jmc = new Hubo2PlusNckJmc;
    }
    else if(type_string == hubo2plus_5ch_type_string)
    {
        new_jmc = new Hubo2Plus5chJmc;
    }
    else if(type_string == drchubo_2ch_type_string)
    {
        new_jmc = new DrcHubo2chJmc;
    }
    else if(type_string == drchubo_3ch_type_string)
    {
        new_jmc = new DrcHubo3chJmc;
    }

    if( NULL == new_jmc )
    {
        if(parsed)
        {
            _parser.error() << "Invalid JMC type: " << jmc_info.type;
            _parser.report_error();
        }
        else
        {
            std::cerr << "Received invalid JMC type: " << jmc_info.type << std::endl;
        }
        return false;
    }

    new_jmc->info = jmc_info;
    jmcs.push_back(new_jmc);

    return true;
}

bool HuboDescription::_parseSensor(hubo_sensor_info_t& info, bool strict)
{
    memset(&info, 0, sizeof(hubo_sensor_info_t));

    StringArray components;
    while(_parser.next_line(components) == HuboCan::DD_OKAY)
    {
        if(components.size() < 2)
        {
            _parser.error() << "Every component must have at least one argument!";
            _parser.report_error();
        }

        if(     "name" == components[0])
        {
            if(components[1].size() > HUBO_COMPONENT_NAME_MAX_LENGTH)
            {
                _parser.error() << "Component name string overflow! Max size is "
                                << HUBO_COMPONENT_NAME_MAX_LENGTH;
                _parser.report_error();
            }
            else
            {
                strcpy(info.name, components[1].c_str());
            }
        }
        else if("type" == components[0])
        {
            if(components[1].size() > HUBO_COMPONENT_TYPE_MAX_LENGTH)
            {
                _parser.error() << "Component type string overflow! Max size is "
                                << HUBO_COMPONENT_TYPE_MAX_LENGTH;
                _parser.report_error();
            }
            else
            {
                strcpy(info.type, components[1].c_str());
            }
        }
        else if("can_channel" == components[0])
        {
            info.can_channel = atoi(components[1].c_str());
        }
        else if("hardware_index" == components[0])
        {
            info.hardware_index = strtol(components[1].c_str(), NULL, 0);
        }
        else
        {
            if(strict)
            {
                _parser.error() << "Invalid component: " << components[0];
                _parser.report_error();
            }
        }

        if(_parser.status() == DD_ERROR)
            return false;
    }

    for(size_t i=0; i<sensors.size(); ++i)
    {
        if(strcmp(info.name, sensors[i]->info.name) == 0)
        {
            _parser.error() << "Repeated sensor name: " << info.name;
            _parser.report_error();
            return false;
        }
    }

    return true;
}

bool HuboDescription::_constructSensor(const hubo_sensor_info_t& sensor_info, bool parsed)
{
    if(     sensor_info.sensor == imu_sensor_string)
        return _constructIMU(sensor_info, parsed);

    else if(sensor_info.sensor == ft_sensor_string)
        return _constructForceTorque(sensor_info, parsed);

    return false;
}

bool HuboDescription::_parseIMU(bool strict)
{
    hubo_sensor_info_t new_imu_info;
    if(!_parseSensor(new_imu_info, strict))
        return false;

    strcpy(new_imu_info.sensor, imu_sensor_string.c_str());

    return _constructIMU(new_imu_info, true);
}

bool HuboDescription::_constructIMU(const hubo_sensor_info_t& imu_info, bool parsed)
{
    HuboImu* new_imu = NULL;
    const std::string type_string(imu_info.type);
    if(     type_string == hubo2plus_imu_sensor_type_string)
    {
        new_imu = new Hubo2PlusImu(_numImus);
    }
    else if(type_string == drchubo_imu_sensor_type_string)
    {
        new_imu = new DrcHuboImu(_numImus);
    }
    else if(type_string == hubo2plus_tilt_sensor_type_string)
    {
        new_imu = new Hubo2PlusTilt(_numImus);
    }

    if( NULL == new_imu )
    {
        if(parsed)
        {
            _parser.error() << "Invalid IMU type: " << imu_info.type;
            _parser.report_error();
        }
        else
        {
            std::cerr << "Received invalid IMU type: " << imu_info.type << std::endl;
        }
        return false;
    }

    new_imu->info = imu_info;
    sensors.push_back(new_imu);
    ++_numImus;

    return true;
}

bool HuboDescription::_parseForceTorque(bool strict)
{
    hubo_sensor_info_t new_ft_info;
    if(!_parseSensor(new_ft_info, strict))
        return false;

    strcpy(new_ft_info.sensor, ft_sensor_string.c_str());

    return _constructForceTorque(new_ft_info, true);
}

bool HuboDescription::_constructForceTorque(const hubo_sensor_info_t& ft_info, bool parsed)
{
    HuboFt* new_ft = NULL;
    const std::string type_string(ft_info.type);
    if(     type_string == hubo2plus_ft_sensor_type_string)
    {
        new_ft = new Hubo2PlusFt(_numFts);
    }
    else if(type_string == drchubo_ft_sensor_type_string)
    {
        new_ft = new DrcHuboFt(_numFts);
    }

    if( NULL == new_ft )
    {
        if(parsed)
        {
            _parser.error() << "Invalid FT type: " << ft_info.type;
            _parser.report_error();
        }
        else
        {
            std::cerr << "Received invalid FT type: " << ft_info.type << std::endl;
        }
        return false;
    }

    new_ft->info = ft_info;
    sensors.push_back(new_ft);
    ++_numFts;

    return true;
}

bool HuboDescription::_parseMeta(bool strict)
{
    StringArray components;
    while(_parser.next_line(components) == HuboCan::DD_OKAY)
    {
        if(components.size() < 2)
        {
            _parser.error() << "Every component must have at least one argument!";
            _parser.report_error();
        }

        if(     "name" == components[0])
        {
            if(components[1].size() > HUBO_COMPONENT_NAME_MAX_LENGTH)
            {
                _parser.error() << "Component name string overflow! Max size is "
                                << HUBO_COMPONENT_NAME_MAX_LENGTH;
                _parser.report_error();
            }
            else
            {
                strcpy(params.name, components[1].c_str());
            }
        }
        else if("default_frequency" == components[0])
        {
            params.frequency = atof(components[1].c_str());
        }
        else if("default_can_bus_count" == components[0])
        {
            params.can_bus_count = atoi(components[1].c_str());
        }
        else
        {
            if(strict)
            {
                _parser.error() << "Invalid component: " << components[0];
                _parser.report_error();
            }
        }

        if(_parser.status() == DD_ERROR)
            return false;
    }

    return true;
}

bool HuboDescription::_postParseProcessing()
{
    joints.clear();

    std::string report;
    HuboJointPtrMap::iterator it = _tempJointMap.begin();
    for(size_t i=0; i < _tempJointMap.size(); ++i)
    {
        if( it->first != i )
        {
            _parser.error() << "Your device description file is missing a joint for index #"
                      << i << "!";
            _parser.report_error();
            return false;
        }

        joints.push_back(it->second);

        size_t jmc_index = getJmcIndex(joints[i]->info.jmc_name);

        if(InvalidIndex == jmc_index)
        {
            std::cerr << "Could not find JMC named '" << joints[i]->info.jmc_name
                      << "' needed by Joint named '" << joints[i]->info.name << "'.\n"
                      << " -- Your Hubo Description is invalid!"
                      << std::endl;
            return false;
        }

        if(!jmcs[jmc_index]->addJoint(joints[i], report))
        {
            _parser.error() << report;
            _parser.report_error();
            return false;
        }

        ++it;
    }

    for(size_t i=0; i < jmcs.size(); ++i)
    {
        if(!jmcs[i]->sortJoints(report))
        {
            _parser.error() << report;
            _parser.report_error();
        }

        if(_parser.status() == DD_ERROR)
            return false;
    }

    _tempJointMap.clear();
    _okay = true;
    return true;
}

size_t HuboDescription::getJointIndex(const std::string& joint_name) const
{
    for(size_t i=0; i < getJointCount(); ++i)
    {
        std::string current_name(joints[i]->info.name);
        if(joint_name == current_name)
        {
            return i;
        }
    }

    return InvalidIndex;
}

IndexArray HuboDescription::getJointIndices(StringArray joint_names) const
{
    IndexArray result;
    for(size_t i=0; i < joint_names.size(); ++i)
    {
        result.push_back(getJointIndex(joint_names[i]));
    }
    return result;
}

std::string HuboDescription::getJointName(size_t joint_index) const
{
    return std::string(getJointInfo(joint_index).name);
}

StringArray HuboDescription::getJointNames(IndexArray joints) const
{
    StringArray result;
    for(size_t i=0; i<joints.size(); ++i)
    {
        result.push_back(getJointName(joints[i]));
    }
    return result;
}

StringArray HuboDescription::getJointNames() const
{
    StringArray result;
    for(size_t i=0; i<getJointCount(); ++i)
    {
        result.push_back(getJointName(i));
    }
    return result;
}

hubo_joint_info_t HuboDescription::getJointInfo(size_t joint_index) const
{
    if( joint_index >= getJointCount() )
    {
        fprintf(stderr, "Requesting joint info for an index which is out-of-bounds (%zu)!\n"
                " -- Maximum index is %zu\n", joint_index, getJointCount()-1);
        hubo_joint_info_t info;
        memset(&info, 0, sizeof(hubo_joint_info_t));
        return info;
    }

    return joints[joint_index]->info;
}

size_t HuboDescription::getJmcIndex(const std::string& jmc_name) const
{
    size_t result = InvalidIndex;
    // TODO: Replace this with a map
    for(size_t i=0; i < jmcs.size(); ++i)
    {
        if(jmc_name.compare(jmcs[i]->info.name)==0)
        {
            return i;
        }
    }
    return result;
}

} // namespace HuboCan
