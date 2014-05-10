
#include "../hubo_path.hpp"

int main(int, char* [])
{
    HuboPath::Trajectory traj;

    ach_channel_t output_chan;
    ach_status_t result = ach_open(&output_chan, HUBO_PATH_INPUT_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Ach error: " << ach_result_to_string(result) << std::endl;
        return 1;
    }
    ach_flush(&output_chan);

    ach_channel_t feedback_chan;
    result = ach_open(&feedback_chan, HUBO_PATH_FEEDBACK_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Feedback channel Ach error: " << ach_result_to_string(result) << std::endl;
        return 2;
    }

    HuboCan::error_result_t received = HuboPath::receive_trajectory(output_chan,
                                                                    feedback_chan,
                                                                    traj, 10);

    std::cout << "Receiving status: " << received << std::endl;
    std::cout << traj << std::endl;

    return 0;
}
