
#include "../hubo_sensor_stream.hpp"

std::ostream& operator<<(std::ostream& stream, const hubo_joint_state_t& state)
{
    stream.precision(3);
    int width = 7;
    stream << std::fixed;

    stream << "pos:";
    stream.width(width);
    stream << state.position;
    stream << "  ref:";
    stream.width(width);
    stream << state.reference;

    stream << "  duty:";
    stream.width(width);
    stream << state.duty;

    stream << "  cur:";
    stream.width(width);
    stream << state.current;

    stream << "  temp:";
    stream.width(width);
    stream << state.temperature;

    return stream;
}

std::ostream& operator<<(std::ostream& stream, const hubo_imu_state_t& imu)
{
    stream.precision(3);
    int width = 7;
    stream << std::fixed;

    stream << "Angles x:";
    stream.width(width);
    stream << imu.angular_position[0];
    stream << "  y:";
    stream.width(width);
    stream << imu.angular_position[1];
    stream << "  z:";
    stream.width(width);
    stream << imu.angular_position[2];

    stream << "Velocities x:";
    stream.width(width);
    stream << imu.angular_velocity[0];
    stream << "  y:";
    stream.width(width);
    stream << imu.angular_velocity[1];
    stream << "  z:";
    stream.width(width);
    stream << imu.angular_velocity[2];

    return stream;
}

std::ostream& operator<<(std::ostream& stream, const hubo_ft_state_t& ft)
{
    stream.precision(3);
    int width = 7;
    stream << std::fixed;

    stream << "Forces x:";
    stream.width(width);
    stream << ft.force[0];
    stream << "  y:";
    stream.width(width);
    stream << ft.force[1];
    stream << "  z:";
    stream.width(width);
    stream << ft.force[2];

    return stream;
}
