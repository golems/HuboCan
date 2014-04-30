
#include "../Trajectory.hpp"
#include "../interpolation/Trajectory.h"

bool HuboPath::Trajectory::interpolate(hubo_path_interp_t type)
{
    params.interp = type;
    return interpolate();
}

bool HuboPath::Trajectory::interpolate()
{
    if( HUBO_PATH_RAW == params.interp )
    {
        return true;
    }
    else if( HUBO_PATH_OPTIMIZE == params.interp )
    {
        return _optimal_interpolation();
    }
    else if( HUBO_PATH_SPLINE == params.interp )
    {
        // TODO: fill in the guts of the spline interpolation
//        return _spline_interpolation();
        return _optimal_interpolation();
    }
    else if( HUBO_PATH_DENSIFY == params.interp )
    {
        return _densify();
    }

    return false;
}

bool HuboPath::Trajectory::_optimal_interpolation()
{
    if(!desc.okay())
    {
        std::cout << "This trajectory does not have a HuboDescription loaded!\n"
                  << " -- We cannot interpolate using HUBO_PATH_OPTIMIZE!" << std::endl;
        return false;
    }

    IndexArray joint_mapping;
    for(size_t i=0; i<HUBO_PATH_JOINT_MAX_SIZE; ++i)
    {
        if( ((params.bitmap >> i) & 0x01) == 1)
        {
            joint_mapping.push_back(i);
        }
    }

    std::list<Eigen::VectorXd> waypoints;
    Eigen::VectorXd next_point(joint_mapping.size());
    for(size_t i=0; i<elements.size(); ++i)
    {
        for(size_t j=0; j<joint_mapping.size(); ++j)
        {
            next_point[j] = elements[i].references[j];
        }
        waypoints.push_back(next_point);
    }

    Eigen::VectorXd velocities(joint_mapping.size());
    Eigen::VectorXd accelerations(joint_mapping.size());
    for(size_t j=0; j<joint_mapping.size(); ++j)
    {
        velocities[j] = desc.joints[joint_mapping[j]]->info.nominal_speed;
        accelerations[j] = desc.joints[joint_mapping[j]]->info.nominal_accel;
    }

    double tolerance = params.tolerance;
    if(tolerance == 0)
    {
        tolerance = 0.01;
    }

    optimal_interpolation::Path optimal_path(waypoints, tolerance);

    optimal_interpolation::Trajectory optimal_traj(optimal_path, velocities, accelerations);

    if(!optimal_traj.isValid())
    {
        std::cout << "The trajectory interpolator for HUBO_PATH_OPTIMIZE has returned invalid!"
                  << std::endl;
        return false;
    }

    double frequency = desc.params.frequency;
    if( 0 == frequency )
    {
        std::cout << "Warning: Your Trajectory object's HuboDescription"
                     "contained a frequency parameter of 0\n"
                     " -- We will default this to 200" << std::endl;
        frequency = 200;
    }

    double dt = 1.0/frequency;
    size_t traj_count = optimal_traj.getDuration()*frequency;
    elements.clear();
    elements.resize(traj_count);
    for(size_t i=0; i<traj_count; ++i)
    {
        next_point = optimal_traj.getPosition(i*dt);

        for(size_t j=0; j<joint_mapping.size(); ++j)
        {
            elements[i].references[joint_mapping[j]] = next_point[j];
        }
    }

    params.interp = HUBO_PATH_RAW;

    return true;
}

bool HuboPath::Trajectory::_spline_interpolation()
{
    // TODO
    return false;
}

bool HuboPath::Trajectory::_densify()
{
    // TODO
    return false;
}
