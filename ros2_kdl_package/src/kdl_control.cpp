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

Eigen::VectorXd KDLController::look_at_point_control(KDL::Frame pose_in_camera_frame, KDL::Frame camera_frame,
                                        KDL::Jacobian camera_jacobian, Eigen::VectorXd q0_dot)
{
    //////////////////////
    // Compute L matrix //
    //////////////////////

    // Convert the camera rotation to Eigen and build the 6x6 spatial rotation matrix
    Matrix6d R = spatialRotation(camera_frame.M);

    // Compute the direction vector s
    Eigen::Vector3d c_P_o = toEigen(pose_in_camera_frame.p);
    Eigen::Vector3d s = c_P_o / c_P_o.norm();

    // Interaction matrix L
    Eigen::Matrix<double, 3, 6> L = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix3d L_11 = (-1 / c_P_o.norm()) * (Eigen::Matrix3d::Identity() - s * s.transpose());
    L.block<3, 3>(0, 0) = L_11;
    L.block<3, 3>(0, 3) = skew(s);
    L = L * R;

    ///////////////////////
    // Compute Jacobians //
    ///////////////////////

    Eigen::MatrixXd J_c = camera_jacobian.data; // Camera Jacobian in the camera frame
    Eigen::MatrixXd LJ = L * J_c;              // Combined matrix L * J_c
    Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse(); // Moore-Penrose pseudoinverse of L * J_c

    // Compute null-space projector N
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J_c.cols(), J_c.cols());
    Eigen::MatrixXd N = I - (LJ_pinv * LJ);

    /////////////////////////
    // Compute Joint Velocities
    /////////////////////////

    Eigen::Vector3d s_d(0, 0, 1); // Desired unit vector pointing forward
    double k = 10;              // Gain for the primary task
    Eigen::VectorXd joint_velocities = k * LJ_pinv * s_d + N * q0_dot;

    Eigen::Vector3d s_error = s - s_d;
    std::cout << s_error.norm() << std::endl;

    // Return computed joint velocities
    return joint_velocities;

}
