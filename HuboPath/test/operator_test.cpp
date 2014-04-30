
#include "../Operator.hpp"

int main(int argc, char* argv[])
{
    HuboPath::Operator op;

    int test_number = 1;
    for(int c=1; c < argc; ++c)
    {
        test_number = atoi(argv[c]);
    }

    StringArray mapping;
    mapping.push_back("WST"); // 0
    mapping.push_back("RSP"); // 1
    mapping.push_back("LKP"); // 2
    mapping.push_back("RHP"); // 3
    mapping.push_back("LEP"); // 4
    mapping.push_back("RSR"); // 5
    mapping.push_back("REP"); // 6

    op.setJointIndices(mapping);

    Eigen::VectorXd vec;
    vec.resize(mapping.size());
    vec.setZero();

    if(test_number == 0)
    {
        vec.setZero();
        op.addWaypoint(vec);
    }
    else if(test_number == 1)
    {
        for(size_t i=0; i<30; ++i)
        {
            vec[2] = i*M_PI/180.0;
            op.addWaypoint(vec);
        }
    }
    else if(test_number == 2)
    {
        vec[1] = 45*M_PI/180.0;
        vec[6] = -30*M_PI/180.0;

        vec[3] = 20*M_PI/180.0;
        vec[2] = 30*M_PI/180.0;
    }
    else
    {
        std::cout << "We do not go up to a test #" << test_number << "!" << std::endl;
        return 1;
    }

    op.interpolate();
    std::cout << op.getCurrentTrajectory() << std::endl;
    op.sendNewTrajectory();

    return 0;
}
