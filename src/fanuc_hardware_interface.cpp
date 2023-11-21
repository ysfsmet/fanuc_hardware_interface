#include <sstream>
#include <ros/console.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <fanuc_hardware_interface/fanuc_hardware_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <industrial_msgs/CmdJointTrajectory.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace fanuc_hardware_interface
{
    FanucHardwareInterface::FanucHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/fanuc_hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &FanucHardwareInterface::update, this);
    }

    FanucHardwareInterface::~FanucHardwareInterface() {
        trajectory_pub.shutdown();
        joint_path_cmd_client.shutdown();
    }

    void FanucHardwareInterface::joint_states_callback(const sensor_msgs::JointStateConstPtr &state)
    {
        for (int i = 0; i < num_joints_; i++) {
            joint_position_[i] = state->position[i];
            joint_velocity_[i] = state->velocity[i];
            joint_effort_[i] = state->effort[i];
        }
    }

    void FanucHardwareInterface::init() {
        // Get joint names
        nh_.getParam("/fanuc_hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {
             // Create joint state interface
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
             joint_state_interface_.registerHandle(jointStateHandle);

            // Create position joint interface
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            JointLimits limits;
                SoftJointLimits softLimits;
            getJointLimits(joint_names_[i], nh_, limits);
            PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
            positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
            position_joint_interface_.registerHandle(jointPositionHandle);

            // Create effort joint interface
            JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(jointEffortHandle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&effort_joint_interface_);
        registerInterface(&positionJointSoftLimitsInterface);

        trajectory_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
        joint_path_cmd_client = nh_.serviceClient<industrial_msgs::CmdJointTrajectory>("joint_path_command");
    }

    void FanucHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void FanucHardwareInterface::read() {
        // for (int i = 0; i < num_joints_; i++) {
        //     joint_position_[i] = ROBOT.getJoint(joint_names_[i]).read();
        // }
    }

    void FanucHardwareInterface::write(ros::Duration elapsed_time) {
        positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
       
        industrial_msgs::CmdJointTrajectory jointCmd;
        trajectory_msgs::JointTrajectory traj;
        trajectory_msgs::JointTrajectoryPoint goal;

        traj.header.frame_id = "base_link";
        traj.joint_names.resize(num_joints_);
        traj.points.resize(1);

        goal.positions.resize(num_joints_);
        goal.velocities.resize(num_joints_);
        goal.effort.resize(num_joints_);

        traj.points[0].positions.resize(num_joints_);
        // Set joint names
        for(int i = 0; i < num_joints_; i++){
            traj.joint_names[i] = joint_names_[i];
        }

        traj.header.stamp = ros::Time::now();
        // Trajectory Points
        for (int i = 0; i < num_joints_; i++)
        {
            goal.positions[i] = joint_position_command_[i];
            goal.velocities[i] = joint_velocity_command_[i];
            goal.effort[i] = joint_effort_command_[i];
        }
        goal.time_from_start = ros::Duration(0.1);
        
        traj.points[0] = goal;

        jointCmd.request.trajectory = traj;
        if(joint_path_cmd_client.call(jointCmd)){
            ROS_INFO_NAMED("fanuc_hardware_interface", "Command Result: %d", (int)jointCmd.response.code.SUCCESS);
        }
        else{
            ROS_ERROR_NAMED("fanuc_hardware_interface", "Failed call to joint_path_command service" );
        }
    }
}
