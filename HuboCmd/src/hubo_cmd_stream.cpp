
#include "../hubo_cmd_stream.hpp"

const char* hubo_cmd_mode_to_string(hubo_cmd_mode_t mode)
{
    switch(mode)
    {
    case HUBO_CMD_IGNORE:       return "HUBO_CMD_IGNORE";   break;
    case HUBO_CMD_RIGID:        return "HUBO_CMD_RIGID";    break;
    case HUBO_CMD_COMPLIANT:    return "HUBO_CMD_COMPLIANT";break;
    case HUBO_CMD_HYBRID:       return "HUBO_CMD_HYBRID";   break;

    case HUBO_CMD_CLAIM:        return "HUBO_CMD_CLAIM";    break;
    case HUBO_CMD_RELEASE:      return "HUBO_CMD_RELEASE";  break;
    default:                    return "HUBO_CMD_UNKNOWN";  break;
    }

    return "HUBO_CMD_IMPOSSIBLE";
}

std::ostream& operator<<(std::ostream& stream, const hubo_cmd_mode_t& mode)
{
    stream << hubo_cmd_mode_to_string(mode);
    return stream;
}

const char* hubo_data_error_to_string(hubo_data_error_t error)
{
    switch(error)
    {
    case HUBO_DATA_OKAY:                return "HUBO_DATA_OKAY";                break;
    case HUBO_DATA_NULL:                return "HUBO_DATA_NULL";                break;
    case HUBO_DATA_OUT_OF_BOUNDS:       return "HUBO_DATA_OUT_OF_BOUNDS";       break;
    case HUBO_DATA_READ_ONLY:           return "HUBO_DATA_READ_ONLY";           break;
    case HUBO_DATA_UNAVAILABLE_INDEX:   return "HUBO_DATA_UNAVAILABLE_INDEX";   break;
    case HUBO_DATA_MALFORMED_HEADER:    return "HUBO_DATA_MALFORMED_HEADER";    break;
    case HUBO_DATA_IMPOSSIBLE:          return "HUBO_DATA_IMPOSSIBLE";          break;
    default:                            return "HUBO_DATA_UNKNOWN";             break;
    }

    return "HUBO_DATA_IMPOSSIBLE";
}

std::ostream& operator<<(std::ostream& stream, const hubo_data_error_t& error)
{
    stream << hubo_data_error_to_string(error);
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const hubo_joint_cmd_t& cmd)
{
    stream.precision(3);
    stream << std::fixed;
    int width = 7;

    stream << "mode:";
    stream.width(20);
    stream << cmd.mode;

    stream << "  pos:";
    stream.width(width);
    stream << cmd.position;

    stream << "  torque:";
    stream.width(width);
    stream << cmd.base_torque;

    stream << "  kP:";
    stream.width(width);
    stream << cmd.kP_gain;

    stream << "  kD:";
    stream.width(width);
    stream << cmd.kD_gain;

    return stream;
}









