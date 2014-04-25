
#include "../interpolation/Trajectory.h"
#include "../hubo_path.hpp"

int main(int argc, char* argv[])
{
    std::list<Eigen::VectorXd> waypoints;

    Eigen::VectorXd point(4);
    point << 0, 0, 0, 0;
    waypoints.push_back(point);

    point << 3, -4, 6, 10;
    waypoints.push_back(point);

    point << -6, 2, 2, -2;
    waypoints.push_back(point);

    point << 0, 0, 0, 0;
    waypoints.push_back(point);

    std::cout << "constructing path" << std::endl;

    optimal_interpolation::Path path(waypoints, 0.01);

    std::cout << "constructed path" << std::endl;

    Eigen::VectorXd vel(4);
    vel << 1, 1, 1, 3;
    Eigen::VectorXd accel(4);
    accel << 0.8, 0.8, 0.8, 0.8;

    std::cout << "constructing trajectory" << std::endl;

    optimal_interpolation::Trajectory traj(path, vel, accel);

    std::cout << "constructed trajectory" << std::endl;

    std::cout << "Duration: " << traj.getDuration() << std::endl;

    HuboPath::Trajectory htraj;
    for(int i=0; i<point.size(); ++i)
        htraj.claim_joint(i);

    size_t traj_count = traj.getDuration()*200;

    Eigen::VectorXd next_waypoint;
    hubo_path_element_t elem;
    memset(&elem, 0, sizeof(elem));
    double dt = 1.0/200.0;
    size_t checker=0;
    for(size_t i=0; i<traj_count; ++i)
    {
        next_waypoint = traj.getPosition(i*dt);

        for(int j=0; j<point.size(); ++j)
        {
            elem.references[j] = next_waypoint[j];
        }
        htraj.push_back(elem);

        if( traj.getPathSegmentIndex(i*dt) > checker )
        {
            std::cout << "switch detected at " << i*dt << " ("
                      << traj.getPathSegmentIndex(i*dt)<< ")" << std::endl;
            ++checker;
        }
    }

//    std::cout << htraj << std::endl;

    return 0;
}
