#include "kuka_lbr_iiwa_control/iiwa_controller.h"

IiwaController::IiwaController()
: m_robot_description { "/robot_description" },
  m_robot_model_loader { m_robot_description },
  m_kinematic_model { m_robot_model_loader.getModel() },
  m_kinematic_state { new robot_state::RobotState(m_kinematic_model) }
{
    ROS_INFO("Model frame: %s", m_kinematic_model->getModelFrame().c_str());
}

IiwaController::~IiwaController(){}

void IiwaController::setDefaultPose()
{
    m_kinematic_state->setToDefaultValues();

}


void IiwaController::printCurrentJointState()
{
    const robot_state::JointModelGroup* joint_model_group = m_kinematic_model->getJointModelGroup("iiwa_arm");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    m_kinematic_state->copyJointGroupPositions(joint_model_group, m_joint_values);
    
    ROS_INFO("Printing joint values...\n");

    m_kinematic_state->printStateInfo();
    m_kinematic_state->printStatePositions();       

    for(auto i { 0 }; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), m_joint_values[i]);
    }
}

trajectory_msgs::JointTrajectory IiwaController::constructTrajMsg() const
{
    trajectory_msgs::JointTrajectory res;

    return res;
}
