
#include "../hubo_path.hpp"
#include <sstream>

HuboCan::error_result_t HuboPath::send_trajectory(ach_channel_t &output_channel,
                                            ach_channel_t &feedback_channel,
                                            const Trajectory& trajectory,
                                            int max_wait_time)
{
    hubo_path_rx_t feedback;
    memset(&feedback, 0, sizeof(feedback));

    ach_status_t result;
    struct timespec t;
    size_t fs;
    int attempts = 0;

    do {

        clock_gettime(ACH_DEFAULT_CLOCK, &t);
        t.tv_sec += 1; ++attempts;
        result = ach_get( &feedback_channel, &feedback, sizeof(feedback), &fs, &t,
                         ACH_O_WAIT | ACH_O_LAST );

        if(ACH_OK != result && ACH_TIMEOUT != result && ACH_MISSED_FRAME != result)
        {
            std::cout << "Unexpected ach result while sending trajectory: "
                      << ach_result_to_string(result) << std::endl;
            return HuboCan::ACH_ERROR;
        }

    } while( feedback.state != PATH_RX_READ_READY && attempts <= max_wait_time );

    if(attempts > max_wait_time)
    {
        std::cout << "We did not receive readiness acknowledgment from listener\n"
                  << " -- Status: " << feedback << "\n"
                  << " -- We will NOT send off the trajectory!" << std::endl;
    }

    uint32_t total_chunks = ceil((double)(trajectory.size())/(double)(HUBO_PATH_CHUNK_MAX_SIZE));

    hubo_path_chunk_t chunk;
    size_t counter = 0;
    for(uint32_t i=0; i<total_chunks; ++i)
    {

        // TODO: The rest of the sending procedure
    }


    return HuboCan::OKAY;
}

HuboCan::error_result_t HuboPath::receive_trajectory(ach_channel_t &input_channel,
                                               ach_channel_t &feedback_channel,
                                               Trajectory &new_trajectory,
                                               int max_wait_time)
{


    return HuboCan::OKAY;
}

const char* hubo_path_interp_to_string(hubo_path_interp_t type)
{
    switch(type)
    {
        case HUBO_PATH_SPLINE:      return "HUBO_PATH_SPLINE";  break;
        case HUBO_PATH_OPTIMIZE:    return "HUBO_PATH_OPTIMIZE";break;
        case HUBO_PATH_DENSIFY:     return "HUBO_PATH_DENSIFY"; break;
        case HUBO_PATH_RAW:         return "HUBO_PATH_RAW";     break;
        default:                    return "HUBO_PATH_UNKNOWN"; break;
    }

    return "HUBO_PATH_IMPOSSIBLE";
}

std::ostream& operator<<(std::ostream& stream, const hubo_path_interp_t& type)
{
    return (stream << hubo_path_interp_to_string(type));
}

const char* hubo_path_instruction_to_string(hubo_path_instruction_t type)
{
    switch(type)
    {
        case HUBO_PATH_QUIT:        return "HUBO_PATH_QUIT";    break;
        case HUBO_PATH_RUN:         return "HUBO_PATH_RUN";     break;
        case HUBO_PATH_PAUSE:       return "HUBO_PATH_PAUSE";   break;
        case HUBO_PATH_REVERSE:     return "HUBO_PATH_REVERSE"; break;
        case HUBO_PATH_LOAD:        return "HUBO_PATH_LOAD";    break;
        default:                    return "HUBO_PATH_UNKNOWN"; break;
    }

    return "HUBO_PATH_IMPOSSIBLE";
}

std::ostream& operator<<(std::ostream& stream, const hubo_path_instruction_t& type)
{
    return (stream << hubo_path_instruction_to_string(type));
}

const char* hubo_path_rx_state_to_string(hubo_path_rx_state_t state)
{
    switch(state)
    {
        case PATH_RX_IGNORING:      return "PATH_RX_IGNORING";      break;
        case PATH_RX_READ_READY:    return "PATH_RX_READ_READY";    break;
        case PATH_RX_LISTENING:     return "PATH_RX_LISTENING";     break;
        case PATH_RX_TIMEOUT:       return "PATH_RX_TIMEOUT";       break;
        case PATH_RX_DISCONTINUITY: return "PATH_RX_DISCONTINUITY"; break;
        case PATH_RX_ACH_ERROR:     return "PATH_RX_ACH_ERROR";     break;
        case PATH_RX_FINISHED:      return "PATH_RX_FINISHED";      break;
        case PATH_RX_CANCELED:      return "PATH_RX_CANCELED";      break;
        default:                    return "PATH_RX_UNKNOWN";       break;
    }

    return "PATH_RX_IMPOSSIBLE";
}

std::ostream& operator<<(std::ostream& stream, const hubo_path_rx_state_t& state)
{
    return (stream << hubo_path_rx_state_to_string(state));
}

const char* hubo_path_rx_to_string(hubo_path_rx_t rx)
{
    std::stringstream sstream;
    sstream << "Status:" << rx.state << ", Chunk ID:" << rx.chunk_id
               << ", Expected Size:" << rx.expected_size;
    return sstream.str().c_str();
}

std::ostream& operator<<(std::ostream& stream, const hubo_path_rx_t& rx)
{
    return (stream << hubo_path_rx_to_string(rx));
}



