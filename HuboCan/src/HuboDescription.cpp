
#include "HuboDescription.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <HuboJoint.hpp>

using namespace HuboCan;

HuboDescription::HuboDescription()
{
    _data = NULL;
}

HuboDescription::~HuboDescription()
{
    free(_data);

    for(size_t i=0; i<_joints.size(); ++i)
    {
        delete _joints[i];
    }

    for(size_t i=0; i<_jmcs.size(); ++i)
    {
        delete _jmcs[i];
    }
}

int HuboDescription::receiveInfo(double timeout)
{
    free(_data);
    _data = hubo_info_receive_data(timeout);

    if(_data == NULL)
        return -1;

    size_t joint_count = hubo_info_get_joint_count(_data);

    for(size_t i=0; i < joint_count; ++i)
    {
        HuboJoint* newJoint = new HuboJoint;
        newJoint->info = *hubo_info_get_joint_info(_data, i);
        _joints.push_back(newJoint);
    }

    for(size_t i=0; i < joint_count; ++i)
    {
        HuboJmc* newJmc = new HuboJmc;
        newJmc->info = *hubo_info_get_jmc_info(_data, i);
        _jmcs.push_back(newJmc);
    }

    free(_data);
    _data = NULL;

    return 0;
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

    return true;
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
    hubo_joint_info_t new_joint_info;
    memset(&new_joint_info, 0, sizeof(hubo_joint_info_t));

    StringArray components;
    while(_parser.next_line(components) == HuboCan::DD_OKAY)
    {
        if(components.size() < 2)
        {
            _parser.error << "Every component must have at least one argument!";
            _parser.report_error();
        }

        if(     "name" == components[0])
        {
            if(components[1].size() > HUBO_COMPONENT_NAME_MAX_LENGTH)
            {
                _parser.error << "Component name string overflow! Max size is " << HUBO_COMPONENT_NAME_MAX_LENGTH;
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
                _parser.error << "Component name string overflow! Max size is " << HUBO_COMPONENT_NAME_MAX_LENGTH;
                _parser.report_error();
            }
            else
            {
                strcpy(new_joint_info.jmc_name, components[1].c_str());
            }
        }
        else
        {
            if(strict)
            {
                _parser.error << "Invalid component: " << components[0];
                _parser.report_error();
            }
        }

        if(_parser.status() == DD_ERROR)
            return false;
    }

    for(size_t i=0; i < _joints.size(); ++i)
    {
        if(strcmp(new_joint_info.name, _joints[i]->info.name)==0)
        {
            _parser.error << "Repeated joint name: " << new_joint_info.name;
            _parser.report_error();
            return false;
        }
    }

    HuboJoint* new_joint = new HuboJoint;
    new_joint->info = new_joint_info;
    _joints.push_back(new_joint);

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
            _parser.error << "Every component must have at least one argument!";
            _parser.report_error();
        }

        if(     "name" == components[0])
        {
            if(components[1].size() > HUBO_COMPONENT_NAME_MAX_LENGTH)
            {
                _parser.error << "Component name string overflow! Max size is " << HUBO_COMPONENT_NAME_MAX_LENGTH;
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
                _parser.error << "Component type string overflow! max size is " << HUBO_COMPONENT_TYPE_MAX_LENGTH;
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
                _parser.error << "Invalid component: " << components[0];
                _parser.report_error();
            }
        }

        if(_parser.status() == DD_ERROR)
            return false;
    }

    for(size_t i=0; i < _jmcs.size(); ++i)
    {
        if(strcmp(new_jmc_info.name, _jmcs[i]->info.name) == 0)
        {
            _parser.error << "Repated JMC name: " << new_jmc_info.name;
            _parser.report_error();
            return false;
        }
    }

    HuboJmc* new_jmc = NULL;
    std::string type_string(new_jmc_info.type);
    if(     type_string == hubo2plus_1ch_code ||
            type_string == hubo2plus_2ch_code)
    {
        new_jmc = new Hubo2Plus2chJmc;
    }
    else if(type_string == hubo2plus_3ch_code)
    {
        new_jmc = new Hubo2Plus3chJmc;
    }
    else if(type_string == hubo2plus_5ch_code)
    {
        new_jmc = new Hubo2Plus5chJmc;
    }
    else if(type_string == drchubo_2ch_code)
    {
        new_jmc = new DrcHubo2chJmc;
    }
    else if(type_string == drchubo_hybrid_code)
    {
        new_jmc = new DrcHuboHybridJmc;
    }

    if( NULL == new_jmc )
    {
        _parser.error << "Invalid JMC type: " << new_jmc_info.type;
        _parser.report_error();
        return false;
    }

    new_jmc->info = new_jmc_info;
    _jmcs.push_back(new_jmc);

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

JointIndex HuboDescription::getJointIndex(const std::string& joint_name)
{
    for(JointIndex i=0; i < jointCount(); ++i)
    {
        std::string current_name(_joints[i]->info.name);
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

hubo_joint_info_t HuboDescription::getJointInfo(JointIndex joint_index)
{
    if( joint_index >= jointCount() )
    {
        fprintf(stderr, "Requesting joint info for an index which is out-of-bounds (%zu)!\n"
                " -- Maximum index is %zu\n", joint_index, jointCount()-1);
        hubo_joint_info_t info;
        memset(&info, 0, sizeof(hubo_joint_info_t));
        return info;
    }

    return _joints[joint_index]->info;
}
