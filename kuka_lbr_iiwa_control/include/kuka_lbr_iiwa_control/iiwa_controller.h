#pragma once

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>


#include <string>

class IiwaController
{
public:
    IiwaController();
    ~IiwaController();

    void setDefaultPose();

    trajectory_msgs::JointTrajectory constructTrajMsg() const;
    void printCurrentJointState();

private:
    const std::string m_robot_description;
    const std::string m_tr_topic_name;
    robot_model_loader::RobotModelLoader m_robot_model_loader;
    robot_model::RobotModelPtr m_kinematic_model;
    
    robot_state::RobotStatePtr m_kinematic_state;

    std::vector<double> m_joint_values;
};