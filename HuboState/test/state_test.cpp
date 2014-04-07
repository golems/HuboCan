#include "../State.hpp"

using namespace HuboState;

int main(int argc, char* argv[])
{
    HuboCan::HuboDescription desc;
    desc.parseFile("../HuboCan/misc/Hubo2Plus.dd");

    State state(desc);

    std::cout << state.joints << std::endl;


    return 0;
}
