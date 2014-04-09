
#include "HuboCmd/Initializer.hpp"

using namespace HuboCmd;

int main(int argc, char* argv[])
{
    Initializer initializer;
    HuboCan::HuboDescription desc;
    desc.receiveInfo();

    std::string joint_name;
    if(argc > 1)
    {
        if(strcmp(argv[1],"all")==0)
        {
            initializer.home_all_joints();
            return 0;
        }
        joint_name = argv[1];
    }
    else
        joint_name = "LKP";

    size_t joint = desc.getJointIndex(joint_name);

    initializer.home_joint(joint);

    return 0;
}
