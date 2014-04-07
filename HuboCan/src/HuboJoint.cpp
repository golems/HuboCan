
#include "HuboCan/HuboJoint.hpp"
#include <math.h>

using namespace HuboCan;

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

