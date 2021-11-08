#include "kuka_lbr_iiwa_control/iiwa_controller.h"

IiwaController::IiwaController()
    : m_robot_description{"/robot_description"},
      m_robot_model_loader{m_robot_description},
      m_kinematic_model{m_robot_model_loader.getModel()},
      m_kinematic_state{new robot_state::RobotState(m_kinematic_model)},
      m_joint_model_group{m_kinematic_model->getJointModelGroup("iiwa_arm")},
      m_joint_names{m_joint_model_group->getJointModelNames()},
      m_joint_values_current{0.,0.,0.,0.,0.,0.,0.},
      isInGravitymode{false},
      m_dynamic_solver{nullptr},
      m_zeros(7, 0.0),
      m_torques(7, 0.0)
{
    ROS_INFO("Model frame: %s", m_kinematic_model->getModelFrame().c_str());

    m_sub = m_nh.subscribe("/kuka_lbr_iiwa_14_r820/joint_states", 
                           10 , 
                           &IiwaController::callbackJointStates, 
                           this);

    m_pub_gravity = m_nh.advertise<rnrt_msgs::JointEffortFeedForward>("/kuka_lbr_iiwa_14_r820/tr_controller/effort_feed_forward", 1);

    geometry_msgs::Wrench temp_wrench;
    geometry_msgs::Vector3 temp_v3;
    temp_v3.x = 0.0;
    temp_v3.y = 0.0;
    temp_v3.z = 0.0;

    temp_wrench.force = temp_v3;
    temp_wrench.torque = temp_v3;

    for(auto i : m_joint_values_current)
    {
        m_wrenches.push_back(temp_wrench);
    }

    m_dynamic_solver = std::make_shared<dynamics_solver::DynamicsSolver>(m_kinematic_model,
                                                                         "iiwa_arm",
                                                                         m_gravity);
    updateKinematicState();
}

IiwaController::~IiwaController() {}

void IiwaController::setDefaultPose()
{
    m_kinematic_state->setToDefaultValues();
}

void IiwaController::printCurrentJointState()
{
    std::vector<double> joint_values;
    m_kinematic_state->copyJointGroupPositions(m_joint_model_group, joint_values);

    ROS_INFO("Printing joint values...");

    for (auto i{0}; i < m_joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", m_joint_names[i].c_str(), joint_values[i]);
    }
}

void IiwaController::setJointStates(const std::vector<double> &joints)
{
    m_joint_values_current = joints;
}

void IiwaController::updateKinematicState()
{
    const std::scoped_lock lock(m_mutex);
    m_kinematic_state->setJointGroupPositions(m_joint_model_group, 
                                              m_joint_values_current);

    if (m_kinematic_state->satisfiesBounds())
    {
        updateEePose();
    }else{
        ROS_INFO("Current state is not valid");
    }
}

trajectory_msgs::JointTrajectory IiwaController::constructTrajMsg() const
{
    trajectory_msgs::JointTrajectory res;

    return res;
}

void IiwaController::callbackJointStates(const sensor_msgs::JointState &pose)
{
    const std::scoped_lock lock(m_mutex);
    setJointStates(pose.position);

    if (isInGravitymode){
        rnrt_msgs::JointEffortFeedForward msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_link";
        msg.name = m_joint_names;

        m_dynamic_solver->getTorques(m_joint_values_current,
                                     m_zeros,
                                     m_zeros,
                                     m_wrenches,
                                     m_torques);

        msg.effort_feed_forward = m_torques;

        m_pub_gravity.publish(msg);
    }
}

void IiwaController::updateEePose()
{
    auto end_effector_state = m_kinematic_state->getGlobalLinkTransform("tool0");
    m_end_effector_state = end_effector_state;
}

void IiwaController::printCurrentEePose()
{
    ROS_INFO_STREAM("Translation: \n" << m_end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << m_end_effector_state.rotation() << "\n");
}

void IiwaController::setGravityMode(const bool& mode)
{
    isInGravitymode = mode;
}