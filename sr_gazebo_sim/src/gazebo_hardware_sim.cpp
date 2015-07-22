
#include "sr_gazebo_sim/gazebo_hardware_sim.h"

#include <string>

using boost::ptr_unordered_map;

namespace sr_gazebo_sim
{

const std::string SrGazeboHWSim::j0_transmission_name = "sr_mechanism_model/J0Transmission";
const std::string SrGazeboHWSim::simple_transmission_name = "sr_mechanism_model/SimpleTransmission";

SrGazeboHWSim::SrGazeboHWSim() : DefaultRobotHWSim(), fake_state_(NULL)
{
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
        std::vector<transmission_interface::TransmissionInfo> *transmissions)
{
    for (size_t i = 0; i < transmissions->size(); ++i)
    {
        if (j0_transmission_name == transmissions->at(i).type_)
        {
            if (transmissions->at(i).joints_.size() > 0)
            {
                const std::string joint_name = transmissions->at(i).joints_[0].name_;
                std::string second_joint_name = joint_name;
                second_joint_name[second_joint_name.size() - 1] = '2';
                this->j2_j1_joints_[second_joint_name] = joint_name;
            }
        }
    }

    for (size_t i = transmissions->size() - 1; i != static_cast<size_t>(-1); --i)
    {
        if (transmissions->at(i).joints_.size() > 0)
        {
            const std::string joint_name = transmissions->at(i).joints_[0].name_;

            if ((joint_name.size() > 0) && ('3' == joint_name[joint_name.size() - 1]))
            {
                std::string second_joint_name = joint_name;
                second_joint_name[second_joint_name.size() - 1] = '2';

                if (this->j2_j1_joints_.count(second_joint_name) > 0)
                {
                    transmission_interface::TransmissionInfo new_transmission = transmissions->at(i);

                    fixJointName(&new_transmission.joints_, joint_name, second_joint_name);
                    fixJointName(&new_transmission.actuators_, joint_name, second_joint_name);
                    transmissions->push_back(new_transmission);
                }
            }
        }
    }
}

bool SrGazeboHWSim::is_hand_joint(const std::vector<transmission_interface::TransmissionInfo> &transmissions,
                                  const std::string &joint_name) const
{
    // TODO: Reimplement this function. It is using simple assumption that hand joint has one of
    // two types of transmission from sr_mechanism_model package.
    bool result = false;
    for (size_t i = 0; i < transmissions.size(); ++i)
    {
        if ((transmissions[i].joints_.size() > 0) && ((j0_transmission_name == transmissions[i].type_) ||
             (simple_transmission_name == transmissions[i].type_)))
        {
            if (transmissions[i].joints_[0].name_ == joint_name)
            {
                result = true;
            }
        }
    }

    return result;
}

void SrGazeboHWSim::initializeFakeRobotState(const urdf::Model *const urdf_model,
                                             const std::vector<transmission_interface::TransmissionInfo> &transmissions)
{
    this->fake_state_.robot_model_ = *urdf_model;

    for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator it = urdf_model->joints_.begin();
         it != urdf_model->joints_.end();
         ++it)
    {
        if (this->is_hand_joint(transmissions, it->first))
        {
            this->fake_state_.joint_states_[it->first].joint_ = it->second;
            this->fake_state_.joint_states_[it->first].calibrated_ = true;
        }
    }

    registerInterface(&this->fake_state_);
    ROS_INFO_STREAM("Registered fake state");
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

        this->initializeFakeRobotState(urdf_model, transmissions);
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
        if (this->j2_j1_joints_.count(joint_name) > 0)
        {
            joint_name = this->j2_j1_joints_[joint_name];
            // TODO: Add here logic to calculate position of J2 based on values for J1 joint

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
