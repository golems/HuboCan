#ifndef HUBOJMC_HPP
#define HUBOJMC_HPP

extern "C" {
#include "hubo_info_c.h"
#include "HuboCmd/hubo_aux_cmd_c.h"
}

#include "CanDevice.hpp"
#include "HuboJoint.hpp"

#include <string>
#include <string.h>

const char hubo2plus_1ch_code[] = "H2P_1CH";
const char hubo2plus_2ch_code[] = "H2P_2CH";
const char hubo2plus_nck_code[] = "H2P_NCK"; // Hubo2Plus neck board
const char hubo2plus_5ch_code[] = "H2P_5CH";

const char drchubo_2ch_code[] = "DRC_2CH";
const char drchubo_3ch_code[] = "DRC_3CH";

namespace HuboCmd {

class Aggregator;
typedef std::vector<hubo_aux_cmd_t> HuboAuxArray;

} // namespace HuboCmd

namespace HuboState {

class State;

} // namespace HuboState

namespace HuboCan {

class HuboJmc : public CanDevice
{
public:
    
    inline void assign_pointers(HuboCmd::Aggregator* agg, HuboState::State* state)
    {
        _agg = agg;
        _state = state;
    }

    hubo_jmc_info_t info;
    HuboJointPtrArray joints;

    bool addJoint(HuboJoint* joint, std::string& error_report);
    bool sortJoints(std::string& error_report);

    void auxiliary_command(const hubo_aux_cmd_t& command);
    
    static std::string header();

    std::string table() const;

protected:

    HuboCmd::HuboAuxArray _aux_commands;

    HuboCmd::Aggregator* _agg;
    HuboState::State* _state;
    
    HuboJointPtrMap _tempJointMap;
    can_frame_t _frame;
    
    inline bool _is_type(const char* type) { return strcmp(info.type, type) == 0; }

};

typedef std::vector<HuboJmc*> HuboJmcPtrArray;

class Hubo2PlusBasicJmc : public HuboJmc
{
public:

    Hubo2PlusBasicJmc();
    virtual void update();
    virtual bool decode(const can_frame_t &frame, size_t channel);

    virtual unsigned long sign_convention_converter(int encoder_value);

protected:

    bool _startup;
    virtual void _cycle_reset();
    virtual void _process_auxiliary_commands();
    virtual void _request_encoder_readings();
    virtual void _send_reference_commands();

    virtual bool _decode_encoder_reading(const can_frame_t& frame);
    virtual bool _decode_status_reading(const can_frame_t& frame);

    virtual void _handle_auxiliary_command(const hubo_aux_cmd_t& cmd);
    virtual void _handle_home_joint(const hubo_aux_cmd_t& cmd);
    virtual void _handle_home_all_joints();

    virtual void _handle_rigid_reference_cmd();


};

class Hubo2Plus2chJmc : public Hubo2PlusBasicJmc
{
public:

protected:

};

class Hubo2PlusNckJmc : public Hubo2PlusBasicJmc
{
public:

    // TODO: The neck has a different protocol than the rest of the joints

protected:

};

class Hubo2Plus5chJmc : public Hubo2PlusBasicJmc
{
public:

protected:

    void _request_encoder_readings();
    void _send_reference_commands();

    bool _decode_encoder_reading(const can_frame_t& frame);
    bool _decode_status_reading(const can_frame_t &frame);

};

class DrcHubo2chJmc : public Hubo2Plus2chJmc
{
public:

    // TODO: Handle compliance control

protected:

};

class DrcHubo3chJmc : public Hubo2PlusBasicJmc
{
public:


protected:

    void _send_reference_commands();

    bool _decode_encoder_reading(const can_frame_t& frame);

};

inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::HuboJmc& jmc)
{
    oStrStream << jmc.table();
    return oStrStream;
}

} // namespace HuboCan

inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::HuboJmc& jmc)
{
    oStrStream << jmc.table();
    return oStrStream;
}

#endif // HUBOJMC_HPP
