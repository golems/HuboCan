
#include "HuboCan/HuboDescription.hpp"
#include <iostream>


int main(int, char* [])
{
    HuboCan::HuboDescription desc;

    desc.receiveInfo();

    std::cout << "======= I AM THE RECEIVER ========" << std::endl;

    std::cout << desc.getJointTable() << std::endl << std::endl;

    std::cout << desc.getJmcTable() << std::endl;

    return 0;
}
