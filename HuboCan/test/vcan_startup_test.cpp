
#include "../SocketCanPump.hpp"

int main(int, char* [])
{
    HuboCan::SocketCanPump(200, 1e6, 2, 1000, true);
    
    return 0;
}
