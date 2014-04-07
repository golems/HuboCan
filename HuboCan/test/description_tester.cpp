
#include "HuboCan/HuboDescription.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
    HuboCan::HuboDescription desc;
//    desc.parseFile("../HuboCan/misc/Hubo2Plus.dd");
    desc.parseFile("../HuboCan/misc/DrcHubo.dd");

    std::cout << desc.getJointTable() << std::endl << std::endl;

    std::cout << desc.getJmcTable() << std::endl;

    desc.broadcastInfo();

    HuboCan::HuboDescription copy_desc(desc);

    std::cout << "Copy-constructed:" << std::endl;
    std::cout << copy_desc.getJointTable() << std::endl << std::endl;

    HuboCan::HuboDescription assign_desc;
    assign_desc = desc;
    std::cout << "Copy-assigned:" << std::endl;
    std::cout << assign_desc.getJointTable() << std::endl;

    return 0;
}
