
#include "HuboCan/HuboDescription.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
    HuboCan::HuboDescription desc;
    desc.parseFile("../HuboCan/devices/Hubo2Plus.dd");
//    desc.parseFile("../HuboCan/devices/DrcHubo.dd");
    desc.broadcastInfo();

//    desc.receiveInfo();

//    std::cout << desc.getJointTable() << std::endl << std::endl;

//    std::cout << desc.getJmcTable() << std::endl;

//    desc.broadcastInfo();

//    HuboCan::HuboDescription copy_desc(desc);

//    std::cout << "Copy-constructed:" << std::endl;
//    std::cout << copy_desc.getJointTable() << std::endl << std::endl;

//    HuboCan::HuboDescription assign_desc;
//    assign_desc = desc;
//    std::cout << "Copy-assigned:" << std::endl;
//    std::cout << assign_desc.getJointTable() << std::endl;

    StringArray joint_names;
    joint_names.push_back("RSP");
    joint_names.push_back("REP");

    IndexArray indices = desc.getJointIndices(joint_names);
    for(size_t i=0; i<indices.size(); ++i)
        std::cout << indices[i] << "\t";
    std::cout << std::endl;


    return 0;
}
