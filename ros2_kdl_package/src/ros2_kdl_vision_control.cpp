#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

class VisionControlNode : public rclcpp::Node {
public:
    VisionControlNode() : Node("ros2_kdl_vision_control"), 
    node_handle_(std::shared_ptr<VisionControlNode>(this)){

        /////////////////////////////
        //       Node arguments    //
        /////////////////////////////

        // declare cmd_interface parameter (position, velocity, effort)
        declare_parameter("cmd_interface", "effort"); // defaults to "position"
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
        {
            RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
        }

        declare_parameter("cmd_type", "jnt_id"); // defaults to "position"
        get_parameter("cmd_type", cmd_type_);

            RCLCPP_INFO(get_logger(),"Current control type is: '%s'", cmd_type_.c_str());

            if (!(cmd_type_ == "op_id" || cmd_type_ == "jnt_id"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

        // declare cmd_interface parameter (positioning, look-at-point)
        declare_parameter("task", "look-at-point"); // defaults to "positioning"
        get_parameter("task", task_);
        RCLCPP_INFO(get_logger(),"Current task selected is: '%s'", task_.c_str());

        if (!(task_ == "positioning" || task_ == "look-at-point" ))
        {
            RCLCPP_INFO(get_logger(),"Selected task is not valid!"); return;
        }

        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;
        aruco_pose_available_ = false;

        /////////////////////////////////////////////////
        // retreive robot urdf and create KDLRobot obj //
        /////////////////////////////////////////////////

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
        nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
        q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
        robot_->setJntLimits(q_min,q_max);

        joint_positions_.resize(nj); // joint positions array
        joint_velocities_.resize(nj); // joint velocities array
        joint_accelerations_.resize(nj);
        joint_efforts_.resize(nj);
        q_des.resize(nj);
        q_dot_des.resize(nj);
        q_ddot_des.resize(nj);

        //////////////////////////////////////////////////////
        // Subscribes to jnt states and update robot values //
        //////////////////////////////////////////////////////

        // Subscriber to jnt states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&VisionControlNode::joint_state_subscriber, this, std::placeholders::_1));

        // Wait for the joint_state topic
        while(!joint_state_available_){
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(node_handle_);
        }

        // Update KDLrobot object
/*         robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data)); // Update with new joint pos and vel
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);  // Add the end effector frame */
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

        KDL::Chain kdl_chain = robot_->getChain();
        fk_solver_ = new KDL::ChainFkSolverPos_recursive(kdl_chain);

        // Compute EE frame
        init_cart_pose_ = robot_->getEEFrame();
        init_cart_vel_ = robot_->getEEVelocity();
        J_cam = robot_->getEEJacobian();

        image_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&VisionControlNode::imageCallback, this, std::placeholders::_1));
        
        // Wait for the aruco pose topic
        while(!aruco_pose_available_){
            RCLCPP_INFO(this->get_logger(), "No image data received yet! ...");
            rclcpp::spin_some(node_handle_);
        }

        // Initialize controller
        controller_ = KDLController(*robot_);

        // Marker in spatial frame
        aruco_pose_base = init_cart_pose_ * pose_in_tool_frame;
 
        double roll, pitch, yaw;
        // Extract the roll, pitch, and yaw from the rotation matrix (aruco_pose_.M)
        pose_in_camera_frame.M.GetRPY(roll, pitch, yaw);
        RCLCPP_INFO(
            this->get_logger(),
            "Transformed Pose in camera frame:\nPosition: [x: %.2f, y: %.2f, z: %.2f]\nOrientation: [roll: %.2f, pitch: %.2f, yaw: %.2f]",
            pose_in_camera_frame.p.x(), pose_in_camera_frame.p.y(), pose_in_camera_frame.p.z(),
            roll, pitch, yaw
        );

        transformed_frame.M.GetRPY(roll, pitch, yaw);
        RCLCPP_INFO(
            this->get_logger(),
            "Transformed Pose in camera frame:\nPosition: [x: %.2f, y: %.2f, z: %.2f]\nOrientation: [roll: %.2f, pitch: %.2f, yaw: %.2f]",
            transformed_frame.p.x(), transformed_frame.p.y(), transformed_frame.p.z(),
            roll, pitch, yaw
        );

        pose_in_tool_frame.M.GetRPY(roll, pitch, yaw);
        RCLCPP_INFO(
            this->get_logger(),
            "Transformed Pose in tool frame:\nPosition: [x: %.2f, y: %.2f, z: %.2f]\nOrientation: [roll: %.2f, pitch: %.2f, yaw: %.2f]",
            pose_in_tool_frame.p.x(), pose_in_tool_frame.p.y(), pose_in_tool_frame.p.z(),
            roll, pitch, yaw
        );

        aruco_pose_base.M.GetRPY(roll, pitch, yaw);
        RCLCPP_INFO(
            this->get_logger(),
            "Transformed Pose in spatial frame:\nPosition: [x: %.2f, y: %.2f, z: %.2f]\nOrientation: [roll: %.2f, pitch: %.2f, yaw: %.2f]",
             aruco_pose_base.p.x(),  aruco_pose_base.p.y(),  aruco_pose_base.p.z(),
            roll, pitch, yaw
        );

        // EE's trajectory initial position
        Eigen::Vector3d init_position = toEigen(init_cart_pose_.p);
        std::cout << init_position <<std::endl;

        // EE's trajectory end position
        Eigen::Vector3d end_position;
        if (task_ == "positioning"){
            end_position = toEigen(aruco_pose_base.p);
            // Plan trajectory
            total_time = 1.5, acc_duration = 0.5;
            planner_ = KDLPlanner(total_time, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
        } else {
            end_position = toEigen(init_cart_pose_.p) + Eigen::Vector3d(0.0, 0.1, 0.0);
            total_time = 10.5, acc_duration = 0.5;
            planner_ = KDLPlanner(total_time, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
        }
        std::cout << end_position <<std::endl;

        // Retrieve the first trajectory point
        p = planner_.compute_trajectory(t_);

        // compute errors
        error = computeLinearError(p.pos, init_position);
        std::cout << "The error norm is : " << error.norm() << std::endl;
        error_norm = error.norm();


        
        if(cmd_interface_ == "velocity"){
            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                        std::bind(&VisionControlNode::cmd_publisher, this));
        
            // Set joint velocity commands
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = joint_velocities_(i);
            }
        }
        else if(cmd_interface_ == "effort"){
            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(20), 
                                        std::bind(&VisionControlNode::cmd_publisher, this));
        
            // Set joint effort commands
            for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                desired_commands_[i] = joint_efforts_(i);
            }
        }

        // Create msg and publish
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);

        // Error topic publisher
        errPublisher_ = this->create_publisher<Float>("/error_topic", 10);
        timer_e = this->create_wall_timer(std::chrono::milliseconds(100), 
                                std::bind(&VisionControlNode::error_publisher, this));

        RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");

    }

private:

    void cmd_publisher(){

        iteration_++;

        // Define trajectory
        int trajectory_len = total_time * 100;
        double dt = 1.0 / (trajectory_len / total_time);
        t_ += dt;
        // Desired camera orient
        s_d << 0, 0, 1;

        if (t_ < total_time){
            // Retrieve the trajectory point
            p = planner_.compute_trajectory(t_);

            // Compute EE frame acctual position and velocity
            KDL::Frame cartpos = robot_->getEEFrame();
            KDL::Twist cartvel = robot_->getEEVelocity();
            J_cam = robot_->getEEJacobian();

            Eigen::VectorXd q0_dot = Eigen::VectorXd::Zero(nj);

            // Compute desired Frame position, velocity and acceleration at current p
            // KDL::Frame desPos; desPos.M = transformed_frame.M *  KDL::Rotation::RotY(1.57); desPos.p = toKDL(p.pos);   // x_des
            KDL::Frame desPos; desPos.M = aruco_pose_base.M ; desPos.p = aruco_pose_base.p;   // x_des
            KDL::Twist desVel; desVel.rot = cartvel.rot * KDL::Vector::Zero(); desVel.vel = toKDL(p.vel);   // x_dot_des
            KDL::Twist desAcc; desAcc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());   // X_ddot_des

            // compute errors
            error = computeLinearError(Eigen::Vector3d(desPos.p.data), Eigen::Vector3d(cartpos.p.data));
            o_error = computeOrientationError(toEigen(cartpos.M), toEigen(desPos.M));

            if(cmd_interface_ == "velocity"){
                if(task_ == "positioning"){
                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
                    joint_positions_.data += joint_velocities_.data * dt;
                }
                else{
                    // Define the pose in the camera frame
                    KDL::Frame cartpos_camera = cartpos * KDL::Frame(
                        KDL::Rotation::RotY(-1.57) * KDL::Rotation::RotZ(-3.14)
                    );

                    // Transform the Jacobian J_cam into the camera frame
                    KDL::Jacobian J_cam_camera(J_cam.columns());
                    KDL::changeBase(J_cam, cartpos_camera.M, J_cam_camera);

                    joint_velocities_.data = controller_.look_at_point_control(pose_in_camera_frame, cartpos_camera, J_cam_camera, q0_dot);

                    // Compute errors
                    Eigen::Vector3d c_P_o = toEigen(pose_in_camera_frame.p);
                    Eigen::Vector3d s = c_P_o / c_P_o.norm();

                    Eigen::Vector3d s_error = s - s_d;
                    error_norm = s_error.norm();
                }
                
            }
            else if(cmd_interface_ == "effort"){
                
                // Define the pose in the camera frame
                KDL::Frame cartpos_camera = cartpos * KDL::Frame(
                    KDL::Rotation::RotY(-1.57) * KDL::Rotation::RotZ(-3.14)
                );
                // Transform the Jacobian J_cam into the camera frame
                KDL::Jacobian J_cam_camera(J_cam.columns());
                KDL::changeBase(J_cam, cartpos_camera.M, J_cam_camera);

                prev_joint_velocities_.data = joint_velocities_.data;

                Eigen::Vector3d c_P_o = toEigen(pose_in_camera_frame.p);
                Eigen::Vector3d s = c_P_o / c_P_o.norm();
                Eigen::Vector3d s_error = s - s_d;
                error_norm = s_error.norm();
                std::cout << error_norm << std::endl;

                // Desired joint velocities
                q_dot_des.data = controller_.look_at_point_control(pose_in_camera_frame, cartpos_camera, J_cam_camera, q0_dot);
                // Desired joint positions
                q_des.data = joint_positions_.data + q_dot_des.data * dt;
                // Desired joint acceleration
                q_ddot_des.data = (joint_velocities_.data - prev_joint_velocities_.data) / dt;

                double cos_theta = s.dot(s_d); // Dot product between vectors
                cos_theta = std::max(-1.0, std::min(1.0, cos_theta)); // Clamp to [-1, 1]
                // double angular_error = std::acos(cos_theta);
                double theta = std::acos(cos_theta);
                Eigen::Vector3d axis = s_d.cross(s);
                //Eigen::Vector3d orientation_error = cam_orient_error * s;
                KDL::Rotation R_des = KDL::Rotation::Rot(toKDL(axis), theta);

                // Vector6d cartvel; cartvel << p.vel + error, orientation_error;
                // Update desired joint velocities using Jacobian pseudoinverse
                //dq_desired.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;

                if(cmd_type_ == "jnt_id"){
                    // Compute torques using inverse dynamics
                    joint_efforts_.data = controller_.idCntr(q_des, q_dot_des, q_ddot_des, Kp, Kd);
                }
                else if(cmd_type_ == "op_id"){
/*                     // Angle between s and s_d
                    double dot_product = s.dot(s_d);
                    double magnitudes = s.norm() * s_d.norm();
                    // Clamp the value to avoid numerical errors outside the domain of acos
                    double cos_theta = std::clamp(dot_product / magnitudes, -1.0, 1.0);
                    // Compute the angle in radians
                    double angle = std::acos(cos_theta); */

                    // Vector6d cartacc; cartacc << p.acc + error / dt, 0, 0, 0;
                    desVel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]), KDL::Vector::Zero());
                    desAcc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]), KDL::Vector::Zero());
                    // desPos.M = desired_frame.M;
                    desPos.M = (robot_->getEEFrame().M)*R_des;
                    // desPos.p = transformed_frame.p; // Possibly multiply by Re

                    joint_efforts_.data = controller_.idCntr(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);
                }
            }

            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            if(cmd_interface_ == "velocity"){
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_(i);
                }
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
            
            // Send cmd 
            if(!(cmd_interface_ == "effort")){
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
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
                joint_efforts_.data = controller_.idCntr(desPos, desVel, desAcc, Kpp, Kpo, Kdp, Kdo);

                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_(i);
                }
            }
            
            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }
    }

    void imageCallback(const geometry_msgs::msg::PoseStamped& msg) {
        aruco_pose_available_ = true;

        // Extract pose information
        const auto position = msg.pose.position;
        const auto orientation = msg.pose.orientation;

        // Convert to KDL::Frame
        KDL::Vector kdl_position(position.x, position.y, position.z);
        KDL::Rotation kdl_rotation = KDL::Rotation::Quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        );

        // Set pose_in_camera_frame with the converted position and rotation
        pose_in_camera_frame.M = kdl_rotation;
        pose_in_camera_frame.p = kdl_position;

        // object detected into the end effector frame
        // pose_in_tool_frame.p = kdl_position;
        // pose_in_tool_frame.M = cam_R_ee * kdl_rotation;
        // Transform the pose from the camera frame to the tool frame
        KDL::Vector adjusted_position(
            kdl_position.data[0] + 0.03, // Add an offset to X
            kdl_position.data[1],        // Y remains the same
            kdl_position.data[2] - 0.20  // Subtract an offset from Z
        );

        // Construct the rotation matrix with an additional rotation about Y
        KDL::Rotation adjusted_rotation = kdl_rotation * KDL::Rotation::RotY(-1.57);
        pose_in_tool_frame = KDL::Frame(adjusted_rotation, adjusted_position);

        // Pose aligned 
        // Transformation matrix
        KDL::Rotation transform = KDL::Rotation::RotX(3.14) ; //* KDL::Rotation::RotY(1.57);
        
        // Apply the transformation
        transformed_frame = KDL::Frame(transform) * pose_in_camera_frame;

    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
        joint_state_available_ = true;
        for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }

    void error_publisher(){
        std_msgs::msg::Float64 error_msg;
        error_msg.data = error_norm;
        errPublisher_->publish(error_msg);
    }

    /////////////////////////////////
    // nodes
    /////////////////////////////////
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::Publisher<Float>::SharedPtr errPublisher_;
    rclcpp::TimerBase::SharedPtr timer_e; 
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::TimerBase::SharedPtr subTimer_;
    rclcpp::Node::SharedPtr node_handle_;

    ///////////////////////////////
    // Robot
    ///////////////////////////////
    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_accelerations_;
    KDL::JntArray joint_efforts_;

    KDL::JntArray q_des;
    KDL::JntArray q_dot_des;
    KDL::JntArray q_ddot_des;

    KDL::Twist desVel;
    KDL::Twist desAcc;
    KDL::Frame desPos;

    KDL::Jacobian J_cam;

    KDL::JntArray prev_joint_velocities_;

    KDL::JntArray q;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;
    KDLController controller_;

    unsigned int nj;
    Eigen::Vector3d s_d; // Desired direction in camera frame
    bool joint_state_available_;
    bool aruco_pose_available_;

    int iteration_;
    double t_;
    double error_norm;
    double total_time;
    double acc_duration;

    std::string task_;
    std::string cmd_interface_;
    std::string cmd_type_;

    KDL::Frame init_cart_pose_;
    KDL::Twist init_cart_vel_;

    KDL::Frame aruco_pose_base;
    KDL::Frame pose_in_camera_frame;
    KDL::Frame pose_in_tool_frame;

    trajectory_point p;

    Eigen::Vector3d error;
    Eigen::Vector3d o_error;

    // Define RPY angles of camera frame with respect ee frame
    double roll = 0.0;      // Roll (phi)
    double pitch = -1.57;   // Pitch (theta)
    double yaw = 3.14;      // Yaw (psi)

    // Create a KDL rotation matrix from RPY
    KDL::Rotation cam_R_ee = KDL::Rotation::RPY(roll, pitch, yaw);
    KDL::Frame transformed_frame;
    KDL::ChainFkSolverPos_recursive* fk_solver_;

    ////////////////////
    // Gains 
    ////////////////////
    // define joint space inverse kinematics 
    double Kp = 10;
    double Kd = 1;

    // define op space inverse kinematics 
    double Kpp = 150;
    double Kpo = 10;
    double Kdp = 20;
    double Kdo = 10;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionControlNode>());
    rclcpp::shutdown();
    return 0;
}
