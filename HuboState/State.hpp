#ifndef HUBOSTATE_HPP
#define HUBOSTATE_HPP

extern "C" {
#include "HuboCan/AchIncludes.h"
#include "hubo_sensor_c.h"
#include "HuboCmd/hubo_cmd_c.h"
}
#include "HuboCan/HuboDescription.hpp"

#include "HuboData.hpp"

namespace HuboState {

class State
{
public:
    State(double timeout=1);
    State(const HuboCan::HuboDescription& description);
    ~State();

    bool receive_description(double timeout_sec=2);
    void load_description(const HuboCan::HuboDescription& description);

    virtual HuboCan::error_result_t update(double timeout_sec=1);
    virtual HuboCan::error_result_t publish();
    
    HuboData<hubo_joint_state_t>    joints;
    HuboData<hubo_imu_state_t>      imus;
    HuboData<hubo_ft_state_t>       force_torques;
    
protected:

    bool _channels_opened;

    virtual void _initialize();
    virtual void _create_memory();

    hubo_cmd_data*    _last_cmd_data;
    HuboCan::HuboDescription _desc;

    inline State(const State& doNotCopy) { }
    inline State& operator=(const State& doNotCopy) { return *this; }

};

} // namespace HuboState

#endif // HUBOSTATE_HPP
