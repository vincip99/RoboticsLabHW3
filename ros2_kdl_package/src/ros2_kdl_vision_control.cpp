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

class VisionControlNode : public rclcpp::Node {
public:
    VisionControlNode() : Node("ros2_kdl_vision_control"), 
    node_handle_(std::shared_ptr<VisionControlNode>(this)){
        /////////////////////////////
        //       Node arguments     //
        /////////////////////////////
        // declare cmd_interface parameter (position, velocity)
        declare_parameter("cmd_interface", "position"); // defaults to "position"
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
        {
            RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
        }

        // declare cmd_interface parameter (positioning, look-at-point)
        declare_parameter("task", "positioning"); // defaults to "position"
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
        joint_torques_.resize(nj);

        //////////////////////////////////////////////////////
        // Subscribes to jnt states and update robot values //
        //////////////////////////////////////////////////////
        // Subscriber to jnt states
        joint_state_available_ = false;

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&VisionControlNode::imageCallback, this, std::placeholders::_1));

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

        // Compute IK
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);

        // Initialize controller
        controller_ = KDLController(*robot_);

        // EE's trajectory initial position (just an offset)
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));
        // EE's trajectory end position (just opposite y)
        Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];

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
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                        std::bind(&Iiwa_pub_sub::cmd_publisher, this));
        
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
                joint_positions_cmd_ = joint_positions_;
                robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
            }
            else if(cmd_interface_ == "velocity"){
                // Compute differential IK
                Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                joint_velocities_cmd.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
            }
            else if(cmd_interface_ == "effort"){
                joint_efforts_cmd.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
            }

            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            if(cmd_interface_ == "position"){
                // Set joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
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

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS2 image to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;

        // Detect Aruco marker
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids);

        if (!marker_ids.empty()) {
            // Calculate pose and control commands
            auto twist = geometry_msgs::msg::Twist();
            if (task_ == "positioning") {
                // Align camera position with the marker
                twist = alignCameraToMarker(marker_corners);
            } else if (task_ == "look-at-point") {
                // Orient camera to "look-at-point"
                twist = lookAtPoint(marker_corners);
            }

            // Publish velocity command
            cmd_pub_->publish(twist);
        }
    }

    geometry_msgs::msg::Twist alignCameraToMarker(const std::vector<std::vector<cv::Point2f>> &marker_corners) {
        geometry_msgs::msg::Twist twist;
        // Implement positioning logic
        // Example: Move along X and Y axes to center the marker in the frame
        return twist;
    }

    geometry_msgs::msg::Twist lookAtPoint(const std::vector<std::vector<cv::Point2f>> &marker_corners) {
        geometry_msgs::msg::Twist twist;
        // Implement orientation logic
        // Example: Rotate to align the camera with a given marker orientation
        return twist;
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
        // 
        joint_state_available_ = true;
        for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_; 
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

    unsigned int nj;
    bool joint_state_available_;

    int iteration_;
    
    double t_;
    double error_norm;

    std::string task_;
    std::string cmd_interface_;

    KDL::Frame init_cart_pose_;
    KDL::Twist init_cart_vel_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionControlNode>());
    rclcpp::shutdown();
    return 0;
}
