#ifndef KDLPlanner_H
#define KDLPlanner_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <cmath>

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

class KDLPlanner
{

public:
    ///////////////////////////
    //      Constructors     //
    ///////////////////////////
    KDLPlanner();
    KDLPlanner(double _maxVel, double _maxAcc);
    KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);

    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius, double _accDuration = 0.5);

    ///////////////////////////
    // Create trajectories   //
    ///////////////////////////
    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    trajectory_point compute_circular_trajectory(double s, double s_dot, double s_ddot);
    trajectory_point compute_linear_trajectory(double s, double s_dot, double s_ddot);

    trajectory_point circular_traj_cubic(const double t);
    trajectory_point circular_traj_trapezoidal(const double t);

    trajectory_point linear_traj_cubic(const double t);
    trajectory_point linear_traj_trapezoidal(const double t);

    KDL::Trajectory* getTrajectory();

    //////////////////////////////////
    trajectory_point compute_trajectory(double time);

    ///////////////////////////
    // Trapezoidal and Cubic //
    ///////////////////////////
    void trapezoidal_vel(double t, double t_c, double &s, double &s_dot, double &s_ddot);
    void cubic_polynomial(double t, double &s, double &s_dot, double &s_ddot);

private:

    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;

    //////////////////////////////////
    double trajDuration_;
    double accDuration_;
    double trajRadius_;
    Eigen::Vector3d trajInit_, trajEnd_;
    trajectory_point p;

};

#endif
