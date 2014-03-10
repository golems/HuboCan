
#include "HuboRT/Manager.hpp"
#include <iostream>



int main(int argc, char* argv[])
{
    HuboRT::Manager mgr;

    mgr.stop_all_processes();

    return 0;
}
