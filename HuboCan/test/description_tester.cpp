
#include "HuboDescription.hpp"


int main(int argc, char* argv[])
{
    HuboCan::HuboDescription desc;
    desc.parseFile("../HuboCan/misc/Hubo2Plus.dd");

    return 0;
}
