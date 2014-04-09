
#include "HuboCmd/Initializer.hpp"

using namespace HuboCmd;

int main(int argc, char* argv[])
{
    Initializer initializer;
    HuboCan::HuboDescription desc;
    desc.receiveInfo();

    std::string joint_name;
    if(argc > 1)
        joint_name = argv[1];
    else
        joint_name = "LEP";

    size_t joint = desc.getJointIndex(joint_name);

    initializer.home_joint(joint);

    return 0;
}
