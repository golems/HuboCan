
#include "../hubo_path.hpp"

int main(int, char* [])
{
    HuboPath::Trajectory traj;
    hubo_path_element_t elem;
    
    IndexArray indices;
    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(4);
    indices.push_back(7);
    indices.push_back(10);

    traj.claim_joints(indices);

    for(size_t i=0; i<50; ++i)
    {
        elem.references[1] = i;
        traj.push_back(elem);
    }

    std::cout << traj << std::endl;

    ach_channel_t output_chan;
    ach_status_t result = ach_open(&output_chan, HUBO_PATH_INPUT_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Ach error: " << ach_result_to_string(result) << std::endl;
        return 1;
    }

    ach_channel_t feedback_chan;
    result = ach_open(&feedback_chan, HUBO_PATH_FEEDBACK_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Feedback channel Ach error: " << ach_result_to_string(result) << std::endl;
        return 2;
    }
    ach_flush(&feedback_chan);

    HuboCan::error_result_t sent = HuboPath::send_trajectory(output_chan,
                                                             feedback_chan,
                                                             traj, 10);

    std::cout << "Sending status: " << sent << std::endl;
    
    return 0;
}
