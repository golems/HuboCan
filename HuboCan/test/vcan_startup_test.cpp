
#include "../SocketCanPump.hpp"

int main(int argc, char* argv[])
{
    HuboCan::SocketCanPump canpump(200, 1e6, 2, true);
    
    return 0;
}
