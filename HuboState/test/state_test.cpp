#include "../State.hpp"

using namespace HuboState;

int main(int, char* [])
{
    State state;

    state.update();

    std::cout << state.joints << std::endl;


    return 0;
}
