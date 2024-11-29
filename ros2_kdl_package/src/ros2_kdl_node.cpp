// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using Float = std_msgs::msg::Float64;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {   
            //////////////////////////////////////////
            // Retreive urdf and create roboy obj   //
            //////////////////////////////////////////

            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "velocity"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);

            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            declare_parameter("traj_type", "linear");
            declare_parameter("s_type", "trapezoidal");
            
            get_parameter("traj_type", traj_type_);
            RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());

            if (!(traj_type_ == "linear" || traj_type_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            get_parameter("s_type", s_type_);
            RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());

            if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            declare_parameter("cmd_type", "op_id");
            get_parameter("cmd_type", cmd_type_);
            RCLCPP_INFO(get_logger(),"Current control type is: '%s'", cmd_type_.c_str());

            if (!(cmd_type_ == "op_id" || cmd_type_ == "jnt_id"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }

            // Create ptr to KDLRobot object from urdf tree
            robot_ = std::make_shared<KDLRobot>(robot_tree);
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);
            joint_positions_.resize(nj); // joint positions array
            joint_velocities_.resize(nj); // joint velocities array
            joint_accelerations_.resize(nj);
            joint_torques_.resize(nj); 

            //////////////////////////////////////////////////////
            // Subscribes to jnt states and update robot values //
            //////////////////////////////////////////////////////

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data)); // Update with new joint pos and vel
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);  // Add the end effector frame
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            init_cart_vel_ = robot_->getEEVelocity();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // Initialize controller
            controller_ = KDLController(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];

            // Plan trajectory
            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0;
            double  traj_radius = 0.15;

            // Retrieve the first trajectory point
            trajectory_point p;
            if(traj_type_ == "linear"){
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
                if(s_type_ == "trapezoidal")
                {
                    p = planner_.linear_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p = planner_.linear_traj_cubic(t_);
                }
            } 
            else if(traj_type_ == "circular")
            {
                planner_ = KDLPlanner(traj_duration, init_position, traj_radius, acc_duration);
                if(s_type_ == "trapezoidal")
                {
                    p = planner_.circular_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p = planner_.circular_traj_cubic(t_);
                }
            }

            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(20), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint effort commands
                for (long int i = 0; i < joint_torques_.data.size(); ++i) {
                    desired_commands_[i] = joint_torques_(i);
                }

                errPublisher_ = this->create_publisher<Float>("/error_topic", 10);
                timer_e = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::error_publisher, this));
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = 1.5; // 
            int trajectory_len = 150; // 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_ += dt;

            if (t_ < total_time){

                // Retrieve the trajectory point
                trajectory_point p;
                if(traj_type_ == "linear"){
                    if(s_type_ == "trapezoidal")
                    {
                        p = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    if(s_type_ == "trapezoidal")
                    {
                        p = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p = planner_.circular_traj_cubic(t_);
                    }
                }

                // Compute EE frame acctual position and velocity
                KDL::Frame cartpos = robot_->getEEFrame();
                KDL::Twist cartvel = robot_->getEEVelocity();

                // Compute desired Frame position, velocity and acceleration at current p
                KDL::Frame desPos; desPos.M = cartpos.M; desPos.p = toKDL(p.pos);   // x_des
                KDL::Twist desVel; desVel.rot = cartvel.rot; desVel.vel = toKDL(p.vel);   // x_dot_des
                KDL::Twist desAcc; desAcc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());   // X_ddot_des

                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;
                error_norm = error.norm();

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else if(cmd_interface_ == "velocity"){

                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                    
                }
                else if(cmd_interface_ == "effort"){

                    if(cmd_type_ == "jnt_id"){
                        // Define control gains for proportional (Kp) and derivative (Kd) terms
                        double Kp = 25;
                        double Kd = 5;

                        // Ensure proper orientation matrices for desired and current rotations
                        Eigen::Matrix<double,3,3,Eigen::RowMajor> R_des(init_cart_pose_.M.data);
                        Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
                        R_des = matrixOrthonormalization(R_des);
                        R_e = matrixOrthonormalization(R_e);

                        // Compute angular velocity errors
                        Eigen::Matrix<double,3,1> omega_des(init_cart_vel_.rot.data);
                        Eigen::Matrix<double,3,1> omega_e(robot_->getEEVelocity().rot.data);

                        // Compute velocity errors (linear and rotational)
                        Eigen::Vector3d error_dot = computeLinearError(p.vel, Eigen::Vector3d(cartvel.vel.data));
                        Eigen::Vector3d o_error_dot = computeOrientationVelocityError(omega_des, omega_e, R_des, R_e);
                        
                        // Compute joint accelerations using the Jacobian pseudo-inverse
                        Vector6d cartacc; cartacc << p.acc + 5*error_dot + 10*error, 5*o_error_dot + 10*o_error;
                        joint_accelerations_.data = pseudoinverse(robot_->getEEJacobian().data) * (cartacc
                                            - robot_->getEEJacDotqDot()*joint_velocities_.data);

                        // Integrate acceleration to obtain velocity and position
                        joint_velocities_.data = joint_velocities_.data + joint_accelerations_.data*dt;
                        joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;

                        // Inverse dynamics to compute control torques
                        joint_torques_.data = controller_.idCntr(joint_positions_, joint_velocities_, joint_accelerations_, Kp, Kd);
                    }
                    else if(cmd_type_ == "op_id"){
                        // define joint space inverse kinematics 
                        double Kpp = 150;
                        double Kpo = 10;
                        double Kdp = 20;
                        double Kdo = 10;
                        joint_torques_.data = controller_.idCntr(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);
                    }

                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_torques_.data.size(); ++i) {
                        desired_commands_[i] = joint_torques_(i);
                    }
                    
                    std::cout << "----------------- Joint states -------------------" << std::endl;
                    std::cout << "joint positions: " << joint_positions_.data.transpose() << std::endl;
                    std::cout << "joint velocities: " << joint_velocities_.data.transpose() << std::endl;
                    std::cout << "joint accelerations: " << joint_accelerations_.data.transpose() << std::endl;
                    std::cout << "torques: " << joint_torques_.data.transpose() <<std::endl;
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                // Send joint velocity commands
                if(!(cmd_interface_ == "effort")){
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_torques_(i);
                    }
                } else{
                    // Stop at the end of trajectory
                    double Kpp = 150;
                    double Kpo = 10;
                    double Kdp = 20;
                    double Kdo = 10;
                    KDL::Frame desPos; desPos = robot_->getEEFrame();   // x_des
                    KDL::Twist desVel; desVel = KDL::Twist(KDL::Vector::Zero(),KDL::Vector::Zero());   // x_dot_des
                    KDL::Twist desAcc; desAcc = KDL::Twist(KDL::Vector::Zero(),KDL::Vector::Zero());   // X_ddot_des
                    joint_torques_.data = controller_.idCntr(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_torques_(i);
                    }
                }
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        // joint state subscriber callback to read actual joint positions and save them 
        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            /*  for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            }
            std::cout<<"\n";
            for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
                 RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            }
            std::cout<<"\n";
            for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
                 RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            } */

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        }

        void error_publisher(){
            std_msgs::msg::Float64 error_msg;
            error_msg.data = error_norm;
            errPublisher_->publish(error_msg);
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::Publisher<Float>::SharedPtr errPublisher_; 
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr timer_e; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_accelerations_;
        
        KDL::JntArray joint_torques_;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        KDLController controller_;

        int iteration_;
        bool joint_state_available_;
        double t_;
        double error_norm;

        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;
        std::string cmd_type_;

        KDL::Frame init_cart_pose_;
        KDL::Twist init_cart_vel_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}