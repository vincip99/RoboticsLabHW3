#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController();
    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);

    Eigen::VectorXd look_at_point_control(KDL::Frame pose_in_camera_frame, 
                                        KDL::Frame camera_frame,
                                        KDL::Jacobian camera_jacobian,
                                        Eigen::VectorXd q0_dot);

/*     Eigen::VectorXd idCntr_look_at_point(double prev_angle, double dt, KDL::Frame &_desPos,
                                      KDL::Frame pose_in_camera_frame, KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo); */

private:

    KDLRobot* robot_;

};

#endif
