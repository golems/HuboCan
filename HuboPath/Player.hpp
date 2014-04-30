#ifndef PLAYER_HPP
#define PLAYER_HPP

#include "HuboCmd/Commander.hpp"
#include "hubo_path.hpp"

namespace HuboPath {

class Player : public HuboCmd::Commander
{
public:
    Player(double timeout=1);
    Player(const HuboCan::HuboDescription& description);

    virtual bool receive_description(double timeout_sec);
    virtual void load_description(const HuboCan::HuboDescription& description);

    virtual bool open_channels();

    bool step();

    const hubo_path_element_t& current_element();

protected:

    double _last_time;

    bool _receive_incoming_trajectory();
    void _send_element_commands(const hubo_path_element_t& elem);

    void _report_state();

    bool _first_step;
    bool _new_trajectory;
    bool _new_instruction;
    void _check_for_instructions();
    
    bool _check_limits();

    bool _channels_opened;
    void _initialize_player();

    Trajectory _trajectory;
    size_t _current_index;
    hubo_path_element_t _current_elem;
    hubo_path_element_t _last_elem;

    hubo_path_command_t _incoming_cmd;
    hubo_path_command_t _current_cmd;

    ach_channel_t _instruction_chan;
    ach_channel_t _input_chan;
    ach_channel_t _feedback_chan;
    ach_channel_t _state_chan;
};

} // namespace HuboPath


#endif // PLAYER_HPP
