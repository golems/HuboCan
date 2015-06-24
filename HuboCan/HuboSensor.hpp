#ifndef HUBOSENSOR_HPP
#define HUBOSENSOR_HPP

#include <vector>
#include <string>

extern "C" {
#include "hubo_info_c.h"
#include "HuboCmd/hubo_aux_cmd_c.h"
}

#include "CanDevice.hpp"

const std::string imu_sensor_string = "IMU";
const std::string hubo2plus_imu_sensor_type_string = "H2P_IMU";
const std::string hubo2plus_tilt_sensor_type_string = "H2P_TILT";
const std::string drchubo_imu_sensor_type_string = "DRC_IMU";

const std::string ft_sensor_string = "FT";
const std::string hubo2plus_ft_sensor_type_string = "H2P_FT";
const std::string drchubo_ft_sensor_type_string = "DRC_FT";

namespace HuboCmd {
class Aggregator;
} // namespace HuboCmd

namespace HuboState {
class State;
} // namespace HuboState

namespace HuboCan {

class HuboSensor : public CanDevice
{
public:

    HuboSensor();

    void assign_pointers(HuboCmd::Aggregator* agg, HuboState::State* state);

    hubo_sensor_info_t info;

    void auxiliary_command(const hubo_aux_cmd_t& command);

protected:

    std::vector<hubo_aux_cmd_t> _aux_commands;

    HuboCmd::Aggregator* _agg;
    HuboState::State* _state;

    can_frame_t _frame;

};

class HuboImu : public HuboSensor
{
public:

    HuboImu(size_t index);

    size_t getImuIndex() const;

protected:

    size_t _index;
};

class Hubo2PlusImu : public HuboImu
{
public:

    Hubo2PlusImu(size_t index);

    virtual void update();
    virtual bool decode(const can_frame_t& frame, size_t channel);

protected:

    virtual void _request_imu_readings();
    virtual void _process_auxiliary_commands();
    virtual void _handle_auxiliary_command(const hubo_aux_cmd_t& cmd);
    virtual void _initialize_imu();
};

class Hubo2PlusTilt : public Hubo2PlusImu
{
public:

    Hubo2PlusTilt(size_t index);

    virtual bool decode(const can_frame_t& frame, size_t channel);

protected:

    virtual void _request_imu_readings();
    virtual void _initialize_imu();
};

class DrcHuboImu : public Hubo2PlusImu
{
public:

    DrcHuboImu(size_t index);

    // This class is identical to the Hubo2PlusImu. We create a separate class
    // for this type of IMU just in case its protocol gets changed in the future.
};

class HuboFt : public HuboSensor
{
public:

    HuboFt(size_t index);

    size_t getFtIndex() const;

protected:

    size_t _index;
};

class Hubo2PlusFt : public HuboFt
{
public:

    Hubo2PlusFt(size_t index);

    virtual void update();
    virtual bool decode(const can_frame_t& frame, size_t channel);

protected:

    virtual void _request_ft_readings();
    virtual void _process_auxiliary_commands();
    virtual void _handle_auxiliary_command(const hubo_aux_cmd_t& cmd);
    virtual void _initialize_ft();
};

class DrcHuboFt : public Hubo2PlusFt
{
public:

    DrcHuboFt(size_t index);

    // This class is identical to the Hubo2PlusFt. We create a separate class
    // for this type of FT just in case its protocol gets changed in the future.

};

typedef std::vector<HuboSensor*> HuboSensorPtrArray;

} // namespace HuboCan

#endif // HUBOSENSOR_HPP
