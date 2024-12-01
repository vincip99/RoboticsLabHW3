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
        declare_parameter("cmd_interface", "velocity"); // defaults to "position"
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
        {
            RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
        }

        // declare cmd_interface parameter (positioning, look-at-point)
        declare_parameter("task", "positioning"); // defaults to "positioning"
        get_parameter("task", task_);

        RCLCPP_INFO(get_logger(),"Current task selected is: '%s'", task_.c_str());

        if (!(task_ == "positioning" || task_ == "look-at-point" ))
        {
            RCLCPP_INFO(get_logger(),"Selected task is not valid!"); return;
        }

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

        //////////////////////////////////////////////////////
        // Subscribes to jnt states and update robot values //
        //////////////////////////////////////////////////////

        // Subscriber to jnt states
        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;
        aruco_pose_available_ = false;

        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&VisionControlNode::joint_state_subscriber, this, std::placeholders::_1));

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
        std::cout << init_cart_pose_ <<std::endl;

        // Compute IK
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);

        image_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&VisionControlNode::imageCallback, this, std::placeholders::_1));
        
        // Wait for the aruco pose topic
        while(!aruco_pose_available_){
            RCLCPP_INFO(this->get_logger(), "No image data received yet! ...");
            rclcpp::spin_some(node_handle_);
        }

        // Initialize controller
        controller_ = KDLController(*robot_);

        aruco_pose_ = init_cart_pose_ * pose_in_camera_frame;

        //aruco_pose_.p = kdl_position; // Assign position to the KDL::Frame
        //aruco_pose_.M = kdl_rotation; // Assign orientation to the KDL::Frame

        double qx, qy, qz, qw;
        // Extract the quaternion from the rotation matrix (aruco_pose_.M)
        aruco_pose_.M.GetQuaternion(qx, qy, qz, qw);
        RCLCPP_INFO(
            this->get_logger(),
            "Transformed Pose in spatial frame:\nPosition: [x: %.2f, y: %.2f, z: %.2f]\nOrientation: [x: %.2f, y: %.2f, z: %.2f, w: %.2f]",
            aruco_pose_.p.x(), aruco_pose_.p.y(), aruco_pose_.p.z(),
            qx, qy, qz, qw
        );

        // EE's trajectory initial position
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));
        // EE's trajectory end position 
        Eigen::Vector3d end_position(Eigen::Vector3d(aruco_pose_.p.data));

        // Plan trajectory
        double traj_duration = 1.5, acc_duration = 0.5, t = 0.0;
        planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile

        // Retrieve the first trajectory point
        trajectory_point p = planner_.compute_trajectory(t);

        // compute errors
        Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));

        if(cmd_interface_ == "position"){
            // Create cmd publisher
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                        std::bind(&VisionControlNode::cmd_publisher, this));
        
            // Send joint position commands
            for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                desired_commands_[i] = joint_positions_(i);
            }
        }
        else if(cmd_interface_ == "velocity"){
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
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
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
        t_+=dt;

        if (t_ < total_time){

            // Retrieve the trajectory point
            trajectory_point p = planner_.compute_trajectory(t_); 

            // Compute EE frame
            KDL::Frame cartpos = robot_->getEEFrame();

            // Compute desired Frame
            KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p.pos); 

            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
            std::cout << "The error norm is : " << error.norm() << std::endl;

            if(cmd_interface_ == "position"){
                // Next Frame
                KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                // Compute IK
                joint_positions_ = joint_positions_;
                robot_->getInverseKinematics(nextFrame, joint_positions_);
            }
            else if(cmd_interface_ == "velocity"){
                // Compute differential IK
                Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                if(task_ == "positioning"){
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                }
                else if(task_ == "look-at-point"){
                    joint_velocities_.data = controller_.look_at_point_control(aruco_pose_, init_cart_pose_,robot_->getEEJacobian());
                }
                
            }
            else if(cmd_interface_ == "effort"){
                joint_efforts_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
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
            
            // Send joint velocity commands
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = 0.0;
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

        RCLCPP_INFO(
            this->get_logger(),
            "Received PoseStamped message in camera frame:\nPosition: [x: %.2f, y: %.2f, z: %.2f]\nOrientation: [x: %.2f, y: %.2f, z: %.2f, w: %.2f]",
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w
        );

        // Convert to KDL::Frame
        KDL::Vector kdl_position(position.x, position.y, position.z);
        KDL::Rotation kdl_rotation = KDL::Rotation::Quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        );
        pose_in_camera_frame.M = kdl_rotation;
        pose_in_camera_frame.p = kdl_position;
        
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
        // 
        joint_state_available_ = true;
        for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }


    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr image_sub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::TimerBase::SharedPtr subTimer_;
    rclcpp::Node::SharedPtr node_handle_;

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_accelerations_;
    KDL::JntArray joint_efforts_;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;
    KDLController controller_;

    unsigned int nj;
    bool joint_state_available_;
    bool aruco_pose_available_;

    int iteration_;
    double t_;
    double error_norm;

    std::string task_;
    std::string cmd_interface_;

    KDL::Frame init_cart_pose_;
    KDL::Twist init_cart_vel_;

    KDL::Frame aruco_pose_;
    KDL::Frame pose_in_camera_frame;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionControlNode>());
    rclcpp::shutdown();
    return 0;
}
