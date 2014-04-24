
#include "../Operator.hpp"

int main(int argc, char* argv[])
{
    HuboPath::Operator op;

    IndexArray mapping;
    mapping.push_back(0);
    mapping.push_back(2);
    mapping.push_back(4);
    mapping.push_back(1);
    mapping.push_back(3);
    mapping.push_back(6);

    op.setJointIndices(mapping);

    Eigen::VectorXd vec;
    vec.resize(mapping.size());
    vec.setZero();

    for(size_t i=0; i<30; ++i)
    {
        vec[2] = i*M_PI/180.0;
        op.addWaypoint(vec);
    }

    op.sendNewTrajectory();

    return 0;
}
