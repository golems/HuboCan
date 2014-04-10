
#include "HuboCan/HuboJoint.hpp"
#include <math.h>
#include <string.h>

using namespace HuboCan;

HuboJoint::HuboJoint()
{
    memset(&info, 0, sizeof(info));
    updated = true;
    dropped_count = 0;
}

double HuboJoint::encoder2radian(int encoder)
{
    return 2*M_PI*(double)(encoder*info.drive_factor)
            /(info.driven_factor*info.harmonic_factor*info.enc_resolution);
}

int HuboJoint::radian2encoder(double radian)
{
    return radian*(info.driven_factor*info.harmonic_factor*info.enc_resolution)
            /(2*M_PI*info.drive_factor);
}

std::string HuboJoint::header()
{
    std::stringstream str;
    str.setf(std::ios::fixed);
    str.setf(std::ios::right);

    str.width(5);
    str << "Index";
    str.width(16);
    str << "Joint Name";
    str.width(16);
    str << "Board Index";
    str.width(8);
    str << "Drive";
    str.width(8);
    str << "Driven";
    str.width(10);
    str << "Harmonic";
    str.width(10);
    str << "Encoder";
    str.width(10);
    str << "JMC Name";

    return str.str();
}

std::string HuboJoint::table() const
{
    std::stringstream str;
    str.setf(std::ios::fixed);
    str.setf(std::ios::right);
    str.precision(0);

    str.width(5);   // Index
    str << info.software_index;
    str.width(16);  // Joint Name
    str << info.name;
    str.width(16);  // Board Index
    str << info.hardware_index;
    str.width(8);   // Drive
    str << info.drive_factor;
    str.width(8);   // Driven
    str << info.driven_factor;
    str.width(10);  // Harmonic
    str << info.harmonic_factor;
    str.width(10);  // Encoder
    str << info.enc_resolution;
    str.width(10);  // JMC Name
    str << info.jmc_name;

    return str.str();
}
