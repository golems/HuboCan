
#include "../HuboData.hpp"

using namespace HuboState;

int main(int, char* [])
{
    HuboData<hubo_joint_state_t> test_data;
    std::vector<std::string> joint_names;
    joint_names.push_back("RSP");
    joint_names.push_back("RSR");
    joint_names.push_back("RSY");
    joint_names.push_back("REP");
    joint_names.push_back("RWY");
    joint_names.push_back("LSP");
    joint_names.push_back("LSR");
    joint_names.push_back("LSY");
    joint_names.push_back("LEP");
    joint_names.push_back("LWY");

    test_data.initialize(joint_names, HUBO_JOINT_SENSOR_CHANNEL);
    
    std::vector<hubo_joint_state_t> vec;
    for(size_t i=0; i<10; ++i)
    {
        hubo_joint_state_t js;
        memset(&js, 0, sizeof(hubo_joint_state_t));
        js.position = i;
        vec.push_back(js);
//        test_data[i].position = i;
    }
    
    test_data.set_data(vec);
    
    test_data.send_data(0);
    
    std::vector<hubo_joint_state_t> getvec = test_data.get_data(false);
    
    for(size_t i=0; i<getvec.size(); ++i)
    {
        std::cout << getvec[i].position << "\t";
    }
    std::cout << std::endl;
    
    for(size_t i=0; i<test_data.size(); ++i)
    {
        std::cout << test_data[i].position << "\t";
    }
    std::cout << std::endl;
    
    std::cout << "Bytes:" << std::endl;
    for(size_t i=0; i<get_data_size<hubo_joint_state_t>(test_data._raw_data); ++i)
    {
        std::cout << (unsigned int)(test_data._raw_data[i]) << "\t";
        if( (i+1)%10 == 0 )
        {
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;

    std::cout << test_data << std::endl;
    
    return 0;
}
