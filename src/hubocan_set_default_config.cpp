
#include "HuboRT/Manager.hpp"

int main(int argc, char* argv[])
{
    std::string robot_type = "virtual_Hubo2Plus";
    for(int i=1; i<argc; ++i)
    {
        robot_type = argv[i];
    }

    HuboRT::Manager mgr;

    mgr.load_config(robot_type);

    return 0;
}
