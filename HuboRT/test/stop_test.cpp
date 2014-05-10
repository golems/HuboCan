
#include "HuboRT/Manager.hpp"
#include <iostream>



int main(int, char* [])
{
    HuboRT::Manager mgr;

    mgr.shut_down();

/*
    for(int i=0; i<argc; ++i)
    {
        if(strcmp(argv[i], "-k")==0)
        {
            mgr.kill_all_processes();
            return 0;
        }
    }

    mgr.stop_all_processes();
*/

    return 0;
}
