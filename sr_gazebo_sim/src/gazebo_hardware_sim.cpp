
#include "sr_gazebo_sim/gazebo_hardware_sim.h"

#include <string>

using boost::ptr_unordered_map;
using namespace ros_ethercat_model;


namespace sr_gazebo_sim
{

SrGazeboHWSim::SrGazeboHWSim() : DefaultRobotHWSim(), fake_state_(NULL)
{
}

bool SrGazeboHWSim::isFourFingersJoints(const std::string joint_name, const unsigned joint_index) const
{
    bool result = false;

    if (joint_name.size() > 3)
    {
        if ('F' == joint_name[joint_name.size() - 4] || 'M' == joint_name[joint_name.size() - 4] ||
                'R' == joint_name[joint_name.size() - 4] || 'L' == joint_name[joint_name.size() - 4])
        {

            if (joint_index < 10)
            {
                char index_character = '0' + joint_index;
                if (index_character == joint_name[joint_name.size() - 1])
                {
                    result = true;
                }
            }
        }
    }

    return result;
}

template <class T>
void SrGazeboHWSim::fixJointName(std::vector<T> *items, const std::string old_joint_name,
                                 const std::string new_joint_name) const
{
    if (items->size() > 0)
    {
        items->at(0).name_ = new_joint_name;

        std::string xml_data = items->at(0).xml_element_;
        for (size_t index = xml_data.find(old_joint_name, 0); std::string::npos != index;
             index = xml_data.find(old_joint_name, index))
        {
            xml_data.replace(index, old_joint_name.length(), new_joint_name);
            index += new_joint_name.length();

        }
        items->at(0).xml_element_ = xml_data;
    }
}

void SrGazeboHWSim::addFakeTransmissionsForJ0(
        std::vector<transmission_interface::TransmissionInfo> *transmissions) const
{
    for (unsigned i = transmissions->size() - 1; i != static_cast<unsigned>(-1); --i)
    {
        if (transmissions->at(i).joints_.size() > 0)
        {
            const std::string joint_name = transmissions->at(i).joints_[0].name_;

            if (this->isFourFingersJoints(joint_name, 3))
            {
                std::string new_joint_name = joint_name;
                new_joint_name[new_joint_name.size() - 1] = '2';

                transmission_interface::TransmissionInfo new_transmission = transmissions->at(i);

                fixJointName(&new_transmission.joints_, joint_name, new_joint_name);
                fixJointName(&new_transmission.actuators_, joint_name, new_joint_name);
                transmissions->push_back(new_transmission);
            }

        }
    }
}

void SrGazeboHWSim::initializeFakeRobotState(const urdf::Model *const urdf_model)
{
    this->fake_state_.robot_model_ = *urdf_model;

    for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator it = urdf_model->joints_.begin();
         it != urdf_model->joints_.end();
         ++it)
    {
        // we are only loading joints that can be controlled
        if (it->second->type == urdf::Joint::PRISMATIC || it->second->type == urdf::Joint::REVOLUTE)
        {
            this->fake_state_.joint_states_[it->first].joint_ = it->second;
        }
    }


    for (ptr_unordered_map<std::string, JointState>::iterator jit = this->fake_state_.joint_states_.begin();
         jit != this->fake_state_.joint_states_.end();
         ++jit)
    {
        jit->second->calibrated_ = true;
    }

    registerInterface(&this->fake_state_);
    ROS_INFO_STREAM("registered fake state");
}

bool SrGazeboHWSim::initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions)
{

    this->addFakeTransmissionsForJ0(&transmissions);

    bool result = gazebo_ros_control::DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model,
                                                                 transmissions);
    if (result)
    {

        this->initializeFakeRobotState(urdf_model);
    }

    return result;
}

void SrGazeboHWSim::readSim(ros::Time time, ros::Duration period)
{
    gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);

    for(unsigned j = 0; j < n_dof_; ++j)
    {
        const std::string joint_name = joint_names_[j];
        if (NULL != this->fake_state_.getJointState(joint_name))
        {
            this->fake_state_.getJointState(joint_name)->position_ = this->joint_position_[j];
            this->fake_state_.getJointState(joint_name)->velocity_ = this->joint_velocity_[j];
            this->fake_state_.getJointState(joint_name)->effort_ = this->joint_effort_[j];
        }
    }

    this->fake_state_.current_time_ = time;
}

void SrGazeboHWSim::writeSim(ros::Time time, ros::Duration period)
{
    for (unsigned j = 0; j < n_dof_; ++j)
    {
        std::string joint_name = joint_names_[j];

        if (this->isFourFingersJoints(joint_name, 2))
        {
            joint_name[joint_name.size() - 1] = '1';
        }

        if (NULL != this->fake_state_.getJointState(joint_name))
        {
            this->joint_position_[j] = this->fake_state_.getJointState(joint_name)->position_;
            this->joint_position_command_[j] = this->fake_state_.getJointState(joint_name)->commanded_position_;

            this->joint_velocity_[j] = this->fake_state_.getJointState(joint_name)->velocity_;
            this->joint_velocity_command_[j] = this->fake_state_.getJointState(joint_name)->commanded_velocity_;

            this->joint_effort_[j] = this->fake_state_.getJointState(joint_name)->effort_;
            this->joint_effort_command_[j] = this->fake_state_.getJointState(joint_name)->commanded_effort_;
        }
    }

    gazebo_ros_control::DefaultRobotHWSim::writeSim(time, period);
}


}

PLUGINLIB_EXPORT_CLASS(sr_gazebo_sim::SrGazeboHWSim, gazebo_ros_control::RobotHWSim)
