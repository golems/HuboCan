
#include "HuboCan/HuboDescription.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
    HuboCan::HuboDescription desc;
    desc.parseFile("../HuboCan/misc/Hubo2Plus.dd");

    std::cout << desc.getJointTable() << std::endl << std::endl;

    std::cout << desc.getJmcTable() << std::endl;

    return 0;
}
