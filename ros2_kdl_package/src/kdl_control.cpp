#include "kdl_control.h"

KDLController::KDLController(){}

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    /*return u*/
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis(); // + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    // Calculate proportional and derivative gains
    Eigen::Matrix<double,6,6> Kp = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,6> Kd = Eigen::Matrix<double,6,6>::Zero();

    // Set gains for position and orientation
    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();

    // Update the robot's state using its Jacobian and dynamics
    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;
    Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);
    Eigen::Matrix<double,7,7> B = robot_->getJsim();

    // Compute desired and actual end-effector positions and orientations
    Eigen::Vector3d x_des(_desPos.p.data);
    Eigen::Vector3d x_e(robot_->getEEFrame().p.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_des(_desPos.M.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
    R_des = matrixOrthonormalization(R_des);
    R_e = matrixOrthonormalization(R_e);

    // Compute desired and actual velocities
    Eigen::Vector3d x_dot_des(_desVel.vel.data);
    Eigen::Vector3d x_dot_e(robot_->getEEVelocity().vel.data);
    Eigen::Matrix<double,3,1> omega_des(_desVel.rot.data);
    Eigen::Matrix<double,3,1> omega_e(robot_->getEEVelocity().rot.data);

    // Compute desired accelerations (linear and rotational)
    Eigen::Matrix<double,6,1> x_ddot_des = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,3,1> alpha_des(_desAcc.vel.data);
    Eigen::Matrix<double,3,1> alpha_r_des(_desAcc.rot.data); 

    // Compute linear errors
    Eigen::Matrix<double,3,1> e_p = computeLinearError(x_des, x_e);
    Eigen::Matrix<double,3,1> e_dot_p = computeLinearError(x_dot_des, x_dot_e);
    // Compute Rotational errors
    Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_des, R_e);
    Eigen::Matrix<double,3,1> e_dot_o = computeOrientationVelocityError(omega_des, omega_e, R_des, R_e);

    // Combine position and orientation errors into a single vector
    Eigen::Matrix<double,6,1> x_tilde = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,6,1> x_dot_tilde = Eigen::Matrix<double,6,1>::Zero();
    x_tilde << e_p, e_o;
    x_dot_tilde << e_dot_p, e_dot_o;
    x_ddot_des << alpha_des, alpha_r_des;

    // Inverse dynamics
    Eigen::Matrix<double,6,1> y = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,6,1> J_dot_q_dot = robot_->getEEJacDotqDot()*robot_->getJntVelocities();
    y << (x_ddot_des + Kd*x_dot_tilde + Kp*x_tilde - J_dot_q_dot);

    // Return the computed joint torques (Optionally include gravity compensation)
    return B*(Jpinv*y) + robot_->getCoriolis(); //+ robot_->getGravity();
    
}
