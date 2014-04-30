#include "../State.hpp"

using namespace HuboState;

int main(int argc, char* argv[])
{
    State state;

    state.update();

    std::cout << state.joints << std::endl;


    return 0;
}
