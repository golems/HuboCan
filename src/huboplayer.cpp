
#include "HuboPath/Player.hpp"
#include "HuboRT/Daemonizer.hpp"


int main(int, char* [])
{
    HuboRT::Daemonizer rt;
    if(!rt.begin("player", 40))
    {
        return 1;
    }
    
    HuboPath::Player player;
    
    while( player.step() && rt.good() )
    {
        player.report_state();
        // Debugging can go here if desired
    }
    
    player.report_state();
    
    return 0;
}