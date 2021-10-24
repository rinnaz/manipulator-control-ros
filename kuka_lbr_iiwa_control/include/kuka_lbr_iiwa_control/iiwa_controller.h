#pragma once

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/dynamics_solver/dynamics_solver.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include "rnrt_msgs/JointGravity.h"

#include <string>
#include <utility>

class IiwaController
{
public:
    IiwaController();
    ~IiwaController();

    void setDefaultPose();

    trajectory_msgs::JointTrajectory constructTrajMsg() const;
    void printCurrentJointState();
    void printCurrentEePose();
    void setJointStates(const std::vector<double> &joints_goal);\
    void updateKinematicState();

    void callbackJointStates(const sensor_msgs::JointState &pose);
    void updateEePose();
    void setGravityMode(const bool& mode);

private:
    bool isInGravitymode;
    std::mutex m_mutex;
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    ros::Publisher  m_pub_gravity;

    const std::string m_robot_description;
    const std::string m_tr_topic_name;
    const robot_model_loader::RobotModelLoader m_robot_model_loader;
    const robot_model::RobotModelPtr m_kinematic_model;
    const robot_state::RobotStatePtr m_kinematic_state;
    const robot_state::JointModelGroup *m_joint_model_group;
    const std::vector<std::string> &m_joint_names;
    const std::vector<double> m_zeros;
    std::vector<double> m_torques;

    dynamics_solver::DynamicsSolverPtr m_dynamic_solver;
    geometry_msgs::Vector3 m_gravity = []{
                                            geometry_msgs::Vector3 ret;
                                            ret.x = 0.0;
                                            ret.y = 0.0;
                                            ret.z = -9.8;
                                            return ret;
                                        }();
    std::vector<geometry_msgs::Wrench> m_wrenches;

    std::vector<double> m_joint_values_current;
    std::vector<double> m_joint_values_target;
    Eigen::Isometry3d m_end_effector_state;
};