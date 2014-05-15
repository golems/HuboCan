
#include "../HuboData.hpp"

using namespace HuboState;

int main(int, char* [])
{
    HuboData<hubo_joint_state_t> rec_data;
    rec_data.verbose = true;
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

    rec_data.initialize(joint_names, HUBO_JOINT_SENSOR_CHANNEL);
    
    std::vector<hubo_joint_state_t> vec;
    for(size_t i=0; i<10; ++i)
    {
        hubo_joint_state_t js;
        js.position = 10-i;
        vec.push_back(js);
//        rec_data[i].position = 10-i;
    }
    
    rec_data.set_data(vec);
    
    
    
    
    rec_data.receive_data(5);
    vec = rec_data.get_data(false);
    
    
    for(size_t i=0; i<vec.size(); ++i)
    {
        std::cout << vec[i].position << "\t";
    }
    std::cout << std::endl;
    
    for(size_t i=0; i<rec_data.size(); ++i)
    {
        std::cout << rec_data[i].position << "\t";
    }
    std::cout << std::endl;
    
    for(size_t i=0; i<get_data_component_count(rec_data._raw_data); ++i)
    {
        hubo_joint_state_t* jsp = get_data_component<hubo_joint_state_t>(rec_data._raw_data, i);
        std::cout << jsp->position << "\t";
    }
    std::cout << std::endl;
    
    std::cout << "Bytes:" << std::endl;
    for(size_t i=0; i<get_data_size<hubo_joint_state_t>(rec_data._raw_data); ++i)
    {
        std::cout << (unsigned int)(rec_data._raw_data[i]) << "\t";
        if( (i+1)%10 == 0 )
        {
            std::cout << std::endl;
        }
    }
    
    return 0;
}
