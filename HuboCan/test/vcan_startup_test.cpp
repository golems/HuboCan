
#include "../SocketCanPump.hpp"

int main(int, char* [])
{
    HuboCan::SocketCanPump canpump(200, 1e6, 2, true);
    
    return 0;
}
