
#include "../Player.hpp"

using namespace HuboPath;

Player::Player(double timeout)
{
    _initialize_player();
    receive_description(timeout);
}

Player::Player(const HuboCan::HuboDescription &description)
{
    _initialize_player();
    load_description(description);
}

void Player::_initialize_player()
{
    _channels_opened = false;
    _new_instruction = false;
    _new_trajectory = false;
    _first_step = true;
    memset(&_current_cmd, 0, sizeof(_current_cmd));
    memset(&_incoming_cmd, 0, sizeof(_incoming_cmd));
    memset(&_current_elem, 0, sizeof(_current_elem));
    memset(&_last_elem, 0, sizeof(_last_elem));
    _current_index = 0;
    open_channels();
}

bool Player::open_channels()
{
    if(_channels_opened)
        return true;

    _channels_opened = true;

    ach_status_t result = ach_open(&_instruction_chan, HUBO_PATH_INSTRUCTION_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Error opening path instruction channel: "
                  << ach_result_to_string(result) << std::endl;
        _channels_opened = false;
    }
    ach_flush(&_instruction_chan);

    result = ach_open(&_input_chan, HUBO_PATH_INPUT_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Error opening path input channel: "
                  << ach_result_to_string(result) << std::endl;
        _channels_opened = false;
    }
    ach_flush(&_input_chan);

    result = ach_open(&_feedback_chan, HUBO_PATH_FEEDBACK_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Error opening path feedback channel: "
                  << ach_result_to_string(result) << std::endl;
        _channels_opened = false;
    }
    ach_flush(&_feedback_chan);

    _channels_opened = _channels_opened && HuboCmd::Commander::open_channels();

    return _channels_opened;
}

void Player::_check_for_instructions()
{
    size_t fs;
    ach_status_t result = ach_get(&_instruction_chan, &_incoming_cmd,
                                  sizeof(_incoming_cmd), &fs, NULL, ACH_O_LAST);
    if( ACH_OK != result && ACH_STALE_FRAMES != result && ACH_MISSED_FRAME != result )
    {
        std::cout << "Unexpected ach result on the path instructions channel: "
                  << ach_result_to_string(result) << std::endl;
    }

    if( ACH_OK == result || ACH_MISSED_FRAME == result )
    {
        _new_instruction = true;
    }
    else
    {
        _new_instruction = false;
    }
}

bool Player::_receive_incoming_trajectory()
{
    HuboCan::error_result_t result = receive_trajectory(_input_chan, _feedback_chan,
                                                        _current_traj, 10);
    if(result != HuboCan::OKAY)
    {
        return false;
    }

    // TODO: Should the trajectory be passed through the controller before evaluating the refs?
    // Almost certainly.
    bool all_valid = true;
    for(size_t i=0; i<HUBO_PATH_JOINT_MAX_SIZE; ++i)
    {
        if( ((_current_traj.params.bitmap >> i) & 0x01) == 0x01 )
        {
            if(_current_traj.elements[0].references[i] != joints[i].reference)
            {
                all_valid = false;
            }
        }
    }

    if(all_valid)
    {
        for(size_t i=0; i<HUBO_PATH_JOINT_MAX_SIZE; ++i)
        {
            if( ((_current_traj.params.bitmap >> i) & 0x01) == 0x01 )
            {
                claim_joint(i);
            }
        }
    }
    else
    {
        return false;
    }

    send_commands();
    _current_index = 0;

    _new_trajectory = true;
    return true;
}

bool Player::step()
{
    HuboCan::error_result_t update_result = update();

    if(update_result != HuboCan::OKAY)
    {
        std::cout << "Player failed to update its states: " << update_result << std::endl;
        return false;
    }

    if(_first_step)
    {
        _last_time = get_time();
        _first_step = false;
    }

    double dt = get_time() - _last_time;
    if( dt <= 0 )
    {
        return true;
    }

    _check_for_instructions();
    if(_new_instruction)
    {
        std::cout << "Received new instruction: " << _incoming_cmd.instruction << std::endl;
    }

    if( HUBO_PATH_QUIT == _current_cmd.instruction )
    {
        if( HUBO_PATH_QUIT != _incoming_cmd.instruction )
        {
            if(_receive_incoming_trajectory())
            {
                _current_cmd = _incoming_cmd;
            }
            else
            {
                _current_traj.clear();
            }
        }
        else if( _new_instruction )
        {
            std::cout << "Quit command received -- but we do not have an active trajectory anyway!"
                      << std::endl;
        }
        return true;
    }

    _current_cmd = _incoming_cmd;

    if( HUBO_PATH_LOAD == _current_cmd.instruction )
    {
        if( _new_instruction )
        {
            if(_receive_incoming_trajectory())
            {
                _current_cmd.instruction = HUBO_PATH_PAUSE;
                _incoming_cmd.instruction = HUBO_PATH_PAUSE;
            }
            else
            {
                _current_traj.clear();
                _current_cmd.instruction = HUBO_PATH_QUIT;
                _incoming_cmd.instruction = HUBO_PATH_QUIT;
            }
        }
        return true;
    }

    if(_current_traj.size() == 0)
    {
        return true;
    }

    if(_new_trajectory)
    {
        _last_elem = _current_traj[0];
        _current_elem = _current_traj[0];
        _current_index = 0;
        _new_trajectory = false;
    }

    if( HUBO_PATH_RUN == _current_cmd.instruction )
    {
        ++_current_index;
        if(_current_index >= _current_traj.size())
            _current_index = _current_traj.size()-1;
    }
    else if( HUBO_PATH_PAUSE == _current_cmd.instruction )
    {
        // No-op
    }
    else if( HUBO_PATH_REVERSE == _current_cmd.instruction )
    {
        --_current_index;
        if(_current_index < 0)
            _current_index = 0;
    }

    _current_elem = _current_traj[_current_index];

    // TODO: Write a controller skeleton and use a controller class instance here

    _send_element_commands(_current_elem);

    _last_elem = _current_elem;
    return true;
}

void Player::_send_element_commands(const hubo_path_element_t &elem)
{
    for(size_t i=0; i<HUBO_PATH_JOINT_MAX_SIZE; ++i)
    {
        if( ((_current_traj.params.bitmap >> i) & 0x01) == 0x01 )
        {
            set_position(i, elem.references[i]);
        }
    }
    send_commands();
}
