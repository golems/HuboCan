
#include "../HuboData.hpp"

using namespace HuboState;

int main(int argc, char* argv[])
{
    HuboData<hubo_joint_state_t> test_data;
    test_data.initialize(10, HUBO_JOINT_SENSOR_CHANNEL);
    hubo_data* saved_data = initialize_data<hubo_joint_state_t>(10);
    
    memcpy(saved_data, test_data._raw_data, get_data_size<hubo_joint_state_t>(saved_data));
//    saved_data[30] = 7;
    
    std::vector<hubo_joint_state_t> vec;
    for(size_t i=0; i<10; ++i)
    {
        hubo_joint_state_t js;
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
    
    for(size_t i=0; i<test_data.array_count(); ++i)
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
    
//    std::cout << "Checking data" << std::endl;
//    for(size_t i=0; i<get_data_size<hubo_joint_state_t>(saved_data); ++i)
//    {
//        if( test_data._raw_data[i] != saved_data[i])
//        {
//            std::cout << "BYTE MISMATCH (" << i << "): " << test_data._raw_data[i]
//                         << " | " << saved_data[i] << std::endl;
//        }
//    }
    
//    for(size_t i=0; i<get_data_component_count(test_data._raw_data); ++i)
//    {
//        hubo_joint_state_t* jsp = get_data_component<hubo_joint_state_t>(test_data._raw_data, i);
//        std::cout << jsp->position << "\t";
//    }
//    std::cout << std::endl;
    
    return 0;
}
