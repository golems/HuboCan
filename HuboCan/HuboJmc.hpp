#ifndef HUBOJMC_HPP
#define HUBOJMC_HPP

extern "C" {
#include "hubo_info_c.h"
}

#include "CanDevice.hpp"
#include "HuboJoint.hpp"
//#include "HuboCmd/Aggregator.hpp"
//#include "HuboState/State.hpp"

#include <string>
#include <string.h>

const char hubo2plus_1ch_code[] = "H2P_1CH";
const char hubo2plus_2ch_code[] = "H2P_2CH";
const char hubo2plus_3ch_code[] = "H2P_3CH";
const char hubo2plus_5ch_code[] = "H2P_5CH";

const char drchubo_2ch_code[] = "DRC_2CH";
const char drchubo_hybrid_code[] = "DRC_HYBRID";

namespace HuboCmd {

class Aggregator;

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
    
    static std::string header();

    std::string table() const;
    
    virtual void update();
    virtual void decode(const can_frame_t &frame, size_t channel);

protected:
    
    void _request_encoder_readings();
    void _send_reference_commands();
    
//    void _decode_2ch_enc(const can_frame_t& frame);
//    void _decode_nck_enc(const can_frame_t& frame);
//    void _decode_finger_enc(const can_frame_t& frame);

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

protected:

};

class Hubo2Plus2chJmc : public Hubo2PlusBasicJmc
{
public:

protected:

};

class Hubo2Plus3chJmc : public Hubo2PlusBasicJmc
{
public:

protected:

};

class Hubo2Plus5chJmc : public Hubo2PlusBasicJmc
{
public:

protected:

};

class DrcHubo2chJmc : public Hubo2Plus2chJmc
{
public:

protected:

};

class DrcHuboHybridJmc : public Hubo2PlusBasicJmc
{
public:

protected:

};

} // namespace HuboCan

std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::HuboJmc& jmc);

#endif // HUBOJMC_HPP
