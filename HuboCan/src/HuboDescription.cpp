
#include "../HuboDescription.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sstream>

#include <iostream>

#include "HuboCan/HuboJoint.hpp"

using namespace HuboCan;

HuboDescription::HuboDescription()
{
    _data = NULL;
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

    _data = NULL;
}

HuboDescription::~HuboDescription()
{
    free(_data);

    for(size_t i=0; i<joints.size(); ++i)
    {
        delete joints[i];
    }

    for(size_t i=0; i<jmcs.size(); ++i)
    {
        delete jmcs[i];
    }

    for(size_t i=0; i<sensors.size(); ++i)
    {
        delete sensors[i];
    }
}

error_result_t HuboDescription::receiveInfo(double timeout)
{
    free(_data);
    _data = hubo_info_receive_data(timeout);

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
        HuboSensor* newSensor = new HuboSensor;
        newSensor->info = *hubo_info_get_sensor_info(_data, i);
        sensors.push_back(newSensor);
    }

    free(_data);
    _data = NULL;

    return OKAY;
}

error_result_t HuboDescription::broadcastInfo()
{
    free(_data);
    _data = hubo_info_init_data(joints.size(), jmcs.size(), sensors.size());

    for(size_t i=0; i < joints.size(); ++i)
    {
        size_t loc = hubo_info_get_joint_location(_data, i);
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
        memcpy(_data+loc, &(sensors[i]->info), sizeof(hubo_jmc_info_t));
    }

    int result = hubo_info_send_data(_data);

    free(_data);
    _data = NULL;

    if(result==0)
        return OKAY;
    else
        return ACH_ERROR;
}

std::string HuboDescription::getJointTable()
{
    std::stringstream table;

    table << HuboJoint::header() << "\n";

    for(size_t i=0; i < joints.size(); ++i)
    {
        table << (*joints[i]) << "\n";
    }

    return table.str();
}

std::string HuboDescription::getJmcTable()
{
    std::stringstream table;

    table << HuboJmc::header() << "\n";

    for(size_t i=0; i < jmcs.size(); ++i)
    {
        table << (*jmcs[i]) << "\n";
    }

    return table.str();
}

bool HuboDescription::parseFile(const std::string &filename)
{
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

bool HuboDescription::_parseDevice(const std::string &device_type)
{
    if("Joint" == device_type)
    {
        if(!_parseJoint())
            return false;
    }
    else if("JMC" == device_type)
    {
        if(!_parseJMC())
            return false;
    }
    else if("IMU" == device_type)
    {
        if(!_parseIMU())
            return false;
    }
    else if("ForceTorque" == device_type)
    {
        if(!_parseForceTorque())
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
                _parser.error() << "Component name string overflow! Max size is " << HUBO_COMPONENT_NAME_MAX_LENGTH;
                _parser.report_error();
            }
            else
            {
                strcpy(new_jmc_info.name, components[1].c_str());
            }
        }
        else if("type" == components[0])
        {
            if(components[1].size() > HUBO_COMPONENT_NAME_MAX_LENGTH)
            {
                _parser.error() << "Component type string overflow! max size is " << HUBO_COMPONENT_TYPE_MAX_LENGTH;
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

    HuboJmc* new_jmc = NULL;
    std::string type_string(new_jmc_info.type);
    if(     type_string == hubo2plus_1ch_code ||
            type_string == hubo2plus_2ch_code)
    {
        new_jmc = new Hubo2Plus2chJmc;
    }
    else if(type_string == hubo2plus_nck_code)
    {
        new_jmc = new Hubo2PlusNckJmc;
    }
    else if(type_string == hubo2plus_5ch_code)
    {
        new_jmc = new Hubo2Plus5chJmc;
    }
    else if(type_string == drchubo_2ch_code)
    {
        new_jmc = new DrcHubo2chJmc;
    }
    else if(type_string == drchubo_3ch_code)
    {
        new_jmc = new DrcHubo3chJmc;
    }

    if( NULL == new_jmc )
    {
        _parser.error() << "Invalid JMC type: " << new_jmc_info.type;
        _parser.report_error();
        return false;
    }

    new_jmc->info = new_jmc_info;
    jmcs.push_back(new_jmc);

    return true;
}

bool HuboDescription::_parseIMU(bool strict)
{



    return true;
}

bool HuboDescription::_parseForceTorque(bool strict)
{

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
    return true;
}

size_t HuboDescription::getJointIndex(const std::string& joint_name)
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

IndexArray HuboDescription::getJointIndices(StringArray joint_names)
{
    IndexArray result;
    for(size_t i=0; i < joint_names.size(); ++i)
    {
        result.push_back(getJointIndex(joint_names[i]));
    }
    return result;
}

std::string HuboDescription::getJointName(size_t joint_index)
{
    return std::string(getJointInfo(joint_index).name);
}

StringArray HuboDescription::getJointNames(IndexArray joints)
{
    StringArray result;
    for(size_t i=0; i<joints.size(); ++i)
    {
        result.push_back(getJointName(joints[i]));
    }
    return result;
}

StringArray HuboDescription::getJointNames()
{
    StringArray result;
    for(size_t i=0; i<getJointCount(); ++i)
    {
        result.push_back(getJointName(i));
    }
    return result;
}

hubo_joint_info_t HuboDescription::getJointInfo(size_t joint_index)
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


size_t HuboDescription::getJmcIndex(const std::string &jmc_name)
{
    size_t result = InvalidIndex;
    for(size_t i=0; i < jmcs.size(); ++i)
    {
        if(jmc_name.compare(jmcs[i]->info.name)==0)
        {
            return i;
        }
    }
    return result;
}
