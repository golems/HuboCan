
#include "../Player.hpp"
#include "HuboRT/Daemonizer.hpp"


int main(int argc, char* argv[])
{
    HuboPath::Player player;
    HuboRT::Daemonizer rt;

    rt.redirect_signals();

    while( player.step() && rt.good() )
    {
        std::cout << player.current_element().references[4] << std::endl;
    }



    return 0;
}
