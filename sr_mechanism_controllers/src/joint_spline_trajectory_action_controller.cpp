/**
 * @file   joint_spline_trajectory_action_controller.cpp
 * @author Guillaume Walck (UPMC) & Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Mar  4 13:08:22 2011
 *
 * @brief  Implement an actionlib server to execute a
 * control_msgs::JointTrajectoryAction. Follows the
 * given trajectory with the arm.
 *
 *
 */

#include "sr_mechanism_controllers/joint_spline_trajectory_action_controller.hpp"
#include <controller_manager_msgs/ListControllers.h>
#include <sr_utilities/getJointState.h>
#include <std_msgs/Float64.h>
#include <sr_robot_msgs/sendupdate.h>

namespace shadowrobot
{
// These functions are pulled from the spline_smoother package.
// They've been moved here to avoid depending on packages that aren't
// mature yet.


static inline void generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;

  for (int i=1; i<=n; i++)
  {
    powers[i] = powers[i-1]*x;
  }
}

static void getQuinticSplineCoefficients(double start_pos, double start_vel, double start_acc,
    double end_pos, double end_vel, double end_acc, double time, std::vector<double>& coefficients)
{
  coefficients.resize(6);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.5*end_acc;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
  }
  else
  {
    double T[6];
    generatePowers(5, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5*start_acc;
    coefficients[3] = (-20.0*start_pos + 20.0*end_pos - 3.0*start_acc*T[2] + end_acc*T[2] -
                       12.0*start_vel*T[1] - 8.0*end_vel*T[1]) / (2.0*T[3]);
    coefficients[4] = (30.0*start_pos - 30.0*end_pos + 3.0*start_acc*T[2] - 2.0*end_acc*T[2] +
                       16.0*start_vel*T[1] + 14.0*end_vel*T[1]) / (2.0*T[4]);
    coefficients[5] = (-12.0*start_pos + 12.0*end_pos - start_acc*T[2] + end_acc*T[2] -
                       6.0*start_vel*T[1] - 6.0*end_vel*T[1]) / (2.0*T[5]);
  }
}
/**
 * \brief Samples a quintic spline segment at a particular time
 */
static void sampleQuinticSpline(const std::vector<double>& coefficients, double time,
                                double& position, double& velocity, double& acceleration)
{
  // create powers of time:
  double t[6];
  generatePowers(5, time, t);

  position = t[0]*coefficients[0] +
             t[1]*coefficients[1] +
             t[2]*coefficients[2] +
             t[3]*coefficients[3] +
             t[4]*coefficients[4] +
             t[5]*coefficients[5];

  velocity = t[0]*coefficients[1] +
             2.0*t[1]*coefficients[2] +
             3.0*t[2]*coefficients[3] +
             4.0*t[3]*coefficients[4] +
             5.0*t[4]*coefficients[5];

  acceleration = 2.0*t[0]*coefficients[2] +
                 6.0*t[1]*coefficients[3] +
                 12.0*t[2]*coefficients[4] +
                 20.0*t[3]*coefficients[5];
}
static void getCubicSplineCoefficients(double start_pos, double start_vel,
                                       double end_pos, double end_vel, double time, std::vector<double>& coefficients)
{
  coefficients.resize(4);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.0;
    coefficients[3] = 0.0;
  }
  else
  {
    double T[4];
    generatePowers(3, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[1] - end_vel*T[1]) / T[2];
    coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[1] + end_vel*T[1]) / T[3];
  }
}

JointTrajectoryActionController::JointTrajectoryActionController() :
  nh_tilde("~"),use_sendupdate(false)
{
  using namespace XmlRpc;
  action_server = boost::shared_ptr<JTAS> (new JTAS("/r_arm_controller/joint_trajectory_action",
                  boost::bind(&JointTrajectoryActionController::execute_trajectory, this, _1),
                  false ));
                //  boost::bind(&JointTrajectoryActionController::cancelCB, this, _1) ));

  std::vector<std::string> joint_labels;

/*  // Gets all of the joints
  XmlRpc::XmlRpcValue joint_names;

  if (!nh.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", nh.getNamespace().c_str());
    return;
  }

  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", nh.getNamespace().c_str());
    return ;
  }

  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpcValue &name_value = joint_names[i];

    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
                nh.getNamespace().c_str());
      return ;
    }

    joint_labels.push_back((std::string)name_value);
  }
*/
    // This the internal order of the joints
    joint_labels.push_back("ShoulderJRotate");
    joint_labels.push_back("ShoulderJSwing");
    joint_labels.push_back("ElbowJSwing");
    joint_labels.push_back("ElbowJRotate");
    joint_labels.push_back("WRJ2");
    joint_labels.push_back("WRJ1");

    // fillup a joint_state_idx_map
    for(unsigned int i=0;i<joint_labels.size();i++)
    {
        joint_state_idx_map[joint_labels[i]]=i;
    }

  //look for controllers and build controller name to joint map
  if( ros::service::waitForService("controller_manager/list_controllers",20000) )
  {
    use_sendupdate=false;
    ros::ServiceClient controller_list_client = nh.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");
    controller_manager_msgs::ListControllers controller_list;
    std::string controlled_joint_name;

	// query the list
    controller_list_client.call(controller_list);
	// build the map
    for (unsigned int i=0; i<controller_list.response.controller.size() ; i++ )
    {
      if(controller_list.response.controller[i].state=="running")
      {
        if (nh.getParam("/"+controller_list.response.controller[i].name+"/joint", controlled_joint_name))
        {
          ROS_DEBUG("controller %d:%s controls joint %s\n",i,controller_list.response.controller[i].name.c_str(),controlled_joint_name.c_str());
          jointControllerMap[controlled_joint_name]= controller_list.response.controller[i].name ;
        }
      }
    }

    //build controller publisher to joint map
    for(unsigned int i=0; i < joint_labels.size(); i ++)
    {
      std::string controller_name=jointControllerMap[joint_labels[i]];

      if(controller_name.compare("")!=0) //if exist, set idx to controller number + 1
      {
        controller_publishers.push_back(nh.advertise<std_msgs::Float64>("/"+jointControllerMap[joint_labels[i]]+"/command", 2));
        jointPubIdxMap[joint_labels[i]]=controller_publishers.size(); //we want the index to be above zero all the time
      }
      else // else put a zero in order to detect when this is empty
      {
        ROS_WARN("Could not find a controller for joint %s",joint_labels[i].c_str());
        jointPubIdxMap[joint_labels[i]]=0; //we want the index to be above zero all the time
      }
    }
  }
  else
  {
    ROS_WARN("controller_manager not found, switching back to sendupdates");
    use_sendupdate=true;
    sr_arm_target_pub = nh.advertise<sr_robot_msgs::sendupdate>("/sr_arm/sendupdate", 2);
    sr_hand_target_pub = nh.advertise<sr_robot_msgs::sendupdate>("/srh/sendupdate", 2);
  }

  ROS_INFO("waiting for getJointState");
  if( ros::service::waitForService("getJointState",20000))
  {
    // open persistent link to joint_state service
    joint_state_client = nh.serviceClient<sr_utilities::getJointState>("getJointState",true);
  }
  else
  {
    ROS_ERROR("Cannot access service: Check if joint_state service is launched");
    ros::shutdown();
    exit(-1);
  }
  ROS_INFO("Got getJointState");
  joint_names_=joint_labels;

  q.resize(joint_names_.size());
  qd.resize(joint_names_.size());
  qdd.resize(joint_names_.size());

  desired_joint_state_pusblisher = nh.advertise<sensor_msgs::JointState> ("/desired_joint_states", 2);

  command_sub = nh.subscribe("command", 1, &JointTrajectoryActionController::commandCB, this);
  ROS_INFO("Listening to commands");

  action_server->start();
  ROS_INFO("Action server started");
}

void JointTrajectoryActionController::updateJointState()
{
  sr_utilities::getJointState getState;
  sensor_msgs::JointState joint_state_msg;
  if(joint_state_client.call(getState))
  {
    joint_state_msg=getState.response.joint_state;
    if(joint_state_msg.name.size()>0)
    {
      //fill up the lookup map with updated positions
      for(unsigned int i=0;i<joint_state_msg.name.size();i++)
      {
        joint_state_map[joint_state_msg.name[i]]=joint_state_msg.position[i];
      }
    }
  }
}

bool JointTrajectoryActionController::getPosition(std::string joint_name, double &position)
{
  std::map<std::string, double>::iterator iter = joint_state_map.find(joint_name);
  if (iter != joint_state_map.end())
  {
    position = iter->second;
    return true;
  }
  else
  {
    ROS_ERROR("Joint %s not found",joint_name.c_str());
    return false;
  }
}


void JointTrajectoryActionController::execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  bool success = true;

  ros::Time time = ros::Time::now() - ros::Duration(0.05);
  last_time_ = time;
  ROS_DEBUG("Figuring out new trajectory at %.3lf, with data from %.3lf",
          time.toSec(), goal->trajectory.header.stamp.toSec());

  boost::shared_ptr<SpecifiedTrajectory> new_traj_ptr(new SpecifiedTrajectory);
  SpecifiedTrajectory &traj = *new_traj_ptr;


  // Finds the end conditions of the final segment
  std::vector<double> prev_positions(joint_names_.size());
  std::vector<double> prev_velocities(joint_names_.size());
  std::vector<double> prev_accelerations(joint_names_.size());

  updateJointState();

  ROS_DEBUG("Initial conditions for new set of splines:");
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    double position;
    if(getPosition(joint_names_[i],position))
      prev_positions[joint_state_idx_map[joint_names_[i]]]=position;
    else
    {
      ROS_ERROR("Cannot get joint_state, not executing trajectory");
      return;
    }
    prev_velocities[i]=0.0;
    prev_accelerations[i]=0.0;

    ROS_DEBUG("    %.2lf, %.2lf, %.2lf  (%s)", prev_positions[i], prev_velocities[i],
              prev_accelerations[i], joint_names_[i].c_str());
  }
  // ------ Tacks on the new segments
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;

  std::vector<double> durations(goal->trajectory.points.size());
  if (goal->trajectory.points.size() > 0)
    durations[0] = goal->trajectory.points[0].time_from_start.toSec();
  for (size_t i = 1; i < goal->trajectory.points.size(); ++i)
    durations[i] = (goal->trajectory.points[i].time_from_start - goal->trajectory.points[i-1].time_from_start).toSec();

  // no continuous joints so do not check if we should wrap

  // extract the traj
  for (size_t i = 0; i < goal->trajectory.points.size(); ++i)
  {
    Segment seg;

    if(goal->trajectory.header.stamp == ros::Time(0.0))
      seg.start_time = (time + goal->trajectory.points[i].time_from_start).toSec() - durations[i];
    else
      seg.start_time = (goal->trajectory.header.stamp + goal->trajectory.points[i].time_from_start).toSec() - durations[i];
    seg.duration = durations[i];
    seg.splines.resize(joint_names_.size());

    // Checks that the incoming segment has the right number of elements.
    if (goal->trajectory.points[i].accelerations.size() != 0 && goal->trajectory.points[i].accelerations.size() != joint_names_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the accelerations", (int)i, (int)goal->trajectory.points[i].accelerations.size());
      return;
    }
    if (goal->trajectory.points[i].velocities.size() != 0 && goal->trajectory.points[i].velocities.size() != joint_names_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the velocities", (int)i, (int)goal->trajectory.points[i].velocities.size());
      return;
    }
    if (goal->trajectory.points[i].positions.size() != joint_names_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the positions", (int)i, (int)goal->trajectory.points[i].positions.size());
      return;
    }

    // Re-orders the joints in the command to match the internal joint order.
    accelerations.resize(goal->trajectory.points[i].accelerations.size());
    velocities.resize(goal->trajectory.points[i].velocities.size());
    positions.resize(goal->trajectory.points[i].positions.size());
    for (size_t j = 0; j < goal->trajectory.joint_names.size(); ++j)
    {
      if (!accelerations.empty()) accelerations[ joint_state_idx_map[goal->trajectory.joint_names[j]] ] = goal->trajectory.points[i].accelerations[j];
      if (!velocities.empty()) velocities[ joint_state_idx_map[goal->trajectory.joint_names[j]] ] = goal->trajectory.points[i].velocities[j];
      if (!positions.empty()) positions[ joint_state_idx_map[goal->trajectory.joint_names[j]] ] = goal->trajectory.points[i].positions[j];
    }

    // Converts the boundary conditions to splines.
    for (size_t j = 0; j < joint_names_.size(); ++j)
    {
      if (prev_accelerations.size() > 0 && accelerations.size() > 0)
      {
        getQuinticSplineCoefficients(
          prev_positions[j], prev_velocities[j], prev_accelerations[j],
          positions[j], velocities[j], accelerations[j],
          durations[i],
          seg.splines[j].coef);
      }
      else if (prev_velocities.size() > 0 && velocities.size() > 0)
      {
        getCubicSplineCoefficients(
          prev_positions[j], prev_velocities[j],
          positions[j], velocities[j],
          durations[i],
          seg.splines[j].coef);
        seg.splines[j].coef.resize(6, 0.0);
      }
      else
      {
        seg.splines[j].coef[0] = prev_positions[j];
        if (durations[i] == 0.0)
          seg.splines[j].coef[1] = 0.0;
        else
          seg.splines[j].coef[1] = (positions[j] - prev_positions[j]) / durations[i];
        seg.splines[j].coef[2] = 0.0;
        seg.splines[j].coef[3] = 0.0;
        seg.splines[j].coef[4] = 0.0;
        seg.splines[j].coef[5] = 0.0;
      }
    }
    // Pushes the splines onto the end of the new trajectory.

    traj.push_back(seg);

    // Computes the starting conditions for the next segment

    prev_positions = positions;
    prev_velocities = velocities;
    prev_accelerations = accelerations;
  }

  // ------ Commits the new trajectory

  if (!new_traj_ptr)
  {
    ROS_ERROR("The new trajectory was null!");
    return;
  }

  ROS_DEBUG("The new trajectory has %d segments", (int)traj.size());

  std::vector<sr_robot_msgs::joint> joint_vector_traj;
  unsigned int controller_pub_idx=0;
  //only one of these 2 will be used
  std_msgs::Float64 target_msg;
  sr_robot_msgs::sendupdate sendupdate_msg_traj;

  //initializes the joint names
  //TODO check if traj only contains joint that we control
  //joint_names_ = internal order. not goal->trajectory.joint_names;
  joint_vector_traj.clear();

  for(unsigned int i = 0; i < joint_names_.size(); ++i)
  {
    sr_robot_msgs::joint joint;
    joint.joint_name = joint_names_[i];
    joint_vector_traj.push_back(joint);
  }

  if(use_sendupdate)
  {
    sendupdate_msg_traj.sendupdate_length = joint_vector_traj.size();
    ROS_DEBUG("Trajectory received: %d joints / %d msg length", (int)goal->trajectory.joint_names.size(), sendupdate_msg_traj.sendupdate_length);
  }

  ros::Rate tmp_rate(1.0);

//  std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = goal->trajectory.points;
//  trajectory_msgs::JointTrajectoryPoint trajectory_step;

  //loop through the steps
  ros::Duration sleeping_time(0.0);
  ROS_DEBUG("Entering the execution loop");

	last_time_ = ros::Time::now();
  while(ros::ok())
  {
    ros::Time time = ros::Time::now();
    ros::Duration dt = time - last_time_;
    last_time_ = time;

    // ------ Finds the current segment
    ROS_DEBUG("Find current segment");

    // Determines which segment of the trajectory to use.  (Not particularly realtime friendly).
    int seg = -1;
    while (seg + 1 < (int)traj.size() && traj[seg+1].start_time < time.toSec())
    {
      ++seg;
    }

    // if the last trajectory is already in the past, stop the servoing
    if( (traj[traj.size()-1].start_time+traj[traj.size()-1].duration) < time.toSec())
    {
		   ROS_DEBUG("trajectory is finished %f<%f",(traj[traj.size()-1].start_time+traj[traj.size()-1].duration),time.toSec());
       break;
    }

    if (seg == -1)
    {
      if (traj.size() == 0)
        ROS_ERROR("No segments in the trajectory");
      else
        ROS_ERROR("No earlier segments.  First segment starts at %.3lf (now = %.3lf)", traj[0].start_time, time.toSec());
		  success = false;
      break;
    }

    // ------ Trajectory Sampling
    ROS_DEBUG("Sample the trajectory");

    for (size_t i = 0; i < q.size(); ++i)
    {
      sampleSplineWithTimeBounds(traj[seg].splines[i].coef, traj[seg].duration,
                                 time.toSec() - traj[seg].start_time,
                                 q[i], qd[i], qdd[i]);
    }
    ROS_DEBUG("Sampled the trajectory");
    //check if preempted
    if (action_server->isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Joint Trajectory Action Preempted");
      // set the action state to preempted
      action_server->setPreempted();
      success = false;
      break;
    }
    ROS_DEBUG("Update the targets");
    //update the targets and publish target joint_states
    sensor_msgs::JointState desired_joint_state_msg;
    for(unsigned int i = 0; i < joint_names_.size(); ++i)
    {
      desired_joint_state_msg.name.push_back(joint_names_[i]);
      desired_joint_state_msg.position.push_back(q[i]);
      desired_joint_state_msg.velocity.push_back(qd[i]);
      desired_joint_state_msg.effort.push_back(0.0);
      if(!use_sendupdate)
      {
        if((controller_pub_idx=jointPubIdxMap[ joint_names_[i] ])>0) // if a controller exist for this joint
        {
          target_msg.data=q[i];
          controller_publishers.at(controller_pub_idx-1).publish(target_msg);
        }
      }
      else
      {
        joint_vector_traj[i].joint_target = q[i] * 57.3;
        ROS_DEBUG("traj[%s]: %f", joint_vector_traj[i].joint_name.c_str(), joint_vector_traj[i].joint_target);
      }
    }
    ROS_DEBUG("Targets updated");

    desired_joint_state_msg.header.stamp = ros::Time::now();
    desired_joint_state_pusblisher.publish(desired_joint_state_msg);

    if(use_sendupdate)
    {
      sendupdate_msg_traj.sendupdate_list = joint_vector_traj;
      sr_arm_target_pub.publish(sendupdate_msg_traj);
      sr_hand_target_pub.publish(sendupdate_msg_traj);
    }

    ROS_DEBUG("Now sleep and loop");
    sleeping_time.sleep();
    sleeping_time = ros::Duration(0.1);
    ROS_DEBUG("redo loop");
  }

  control_msgs::FollowJointTrajectoryResult joint_trajectory_result;

  if(success)
    action_server->setSucceeded(joint_trajectory_result);
  else
    action_server->setAborted(joint_trajectory_result);
}

  void JointTrajectoryActionController::sampleSplineWithTimeBounds(
  const std::vector<double>& coefficients, double duration, double time,
  double& position, double& velocity, double& acceleration)
{
  if (time < 0)
  {
    double _;
    sampleQuinticSpline(coefficients, 0.0, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else if (time > duration)
  {
    double _;
    sampleQuinticSpline(coefficients, duration, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else
  {
    sampleQuinticSpline(coefficients, time,
                        position, velocity, acceleration);
  }
}

void JointTrajectoryActionController::commandCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  bool success = true;

  ros::Time time = ros::Time::now()-ros::Duration(0.05);
  last_time_ = time;

  ROS_ERROR("Figuring out new trajectory at %.3lf, with data from %.3lf with %zu waypoints",
          time.toSec(), msg->header.stamp.toSec(), msg->points.size());

  boost::shared_ptr<SpecifiedTrajectory> new_traj_ptr(new SpecifiedTrajectory);
  SpecifiedTrajectory &traj = *new_traj_ptr;


  // Finds the end conditions of the final segment
  std::vector<double> prev_positions(joint_names_.size());
  std::vector<double> prev_velocities(joint_names_.size());
  std::vector<double> prev_accelerations(joint_names_.size());

  updateJointState();

  ROS_DEBUG("Initial conditions for new set of splines:");
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    double position;
    if(getPosition(joint_names_[i],position))
      prev_positions[i]=position;
    else
    {
      ROS_ERROR("Cannot get joint_state, not executing trajectory");
      return;
    }
    prev_velocities[i]=0.0;
    prev_accelerations[i]=0.0;

    ROS_DEBUG("    %.2lf, %.2lf, %.2lf  (%s)", prev_positions[i], prev_velocities[i],
              prev_accelerations[i], joint_names_[i].c_str());
  }
  // ------ Tacks on the new segments
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;

  std::vector<double> durations(msg->points.size());
  if (msg->points.size() > 0)
    durations[0] = msg->points[0].time_from_start.toSec();
  for (size_t i = 1; i < msg->points.size(); ++i)
    durations[i] = (msg->points[i].time_from_start - msg->points[i-1].time_from_start).toSec();

  // no continuous joints so do not check if we should wrap

  // extract the traj
  for (size_t i = 0; i < msg->points.size(); ++i)
  {
    Segment seg;
		ROS_DEBUG("Current time %f and header time %f",msg->header.stamp.toSec(),ros::Time(0.0).toSec());
    if(msg->header.stamp == ros::Time(0.0))
    {
      seg.start_time = (time + msg->points[i].time_from_start).toSec() - durations[i];
      ROS_DEBUG("Segment %zu start time A %f,time_from_start %f, duration, %f", i, seg.start_time,msg->points[i].time_from_start.toSec(), durations[i]);
    }
    else
    {
      seg.start_time = (msg->header.stamp + msg->points[i].time_from_start).toSec() - durations[i];
      ROS_DEBUG("Segment start time B %f",seg.start_time);
    }
    seg.duration = durations[i];
    seg.splines.resize(joint_names_.size());

    // Checks that the incoming segment has the right number of elements.
    if (msg->points[i].accelerations.size() != 0 && msg->points[i].accelerations.size() != joint_names_.size())
    {
      ROS_DEBUG("Command point %d has %d elements for the accelerations", (int)i, (int)msg->points[i].accelerations.size());
      return;
    }
    if (msg->points[i].velocities.size() != 0 && msg->points[i].velocities.size() != joint_names_.size())
    {
      ROS_DEBUG("Command point %d has %d elements for the velocities", (int)i, (int)msg->points[i].velocities.size());
      return;
    }
    if (msg->points[i].positions.size() != joint_names_.size())
    {
      ROS_DEBUG("Command point %d has %d elements for the positions", (int)i, (int)msg->points[i].positions.size());
      return;
    }

     // Re-orders the joints in the command to match the internal joint order.
    accelerations.resize(msg->points[i].accelerations.size());
    velocities.resize(msg->points[i].velocities.size());
    positions.resize(msg->points[i].positions.size());
    for (size_t j = 0; j < joint_names_.size(); ++j)
    {
      if (!accelerations.empty()) accelerations[j] = msg->points[i].accelerations[j];
      if (!velocities.empty()) velocities[j] = msg->points[i].velocities[j];
      if (!positions.empty()) positions[j] = msg->points[i].positions[j];
    }

    // Converts the boundary conditions to splines.
    for (size_t j = 0; j < joint_names_.size(); ++j)
    {
      if (prev_accelerations.size() > 0 && accelerations.size() > 0)
      {
        getQuinticSplineCoefficients(
          prev_positions[j], prev_velocities[j], prev_accelerations[j],
          positions[j], velocities[j], accelerations[j],
          durations[i],
          seg.splines[j].coef);
      }
      else if (prev_velocities.size() > 0 && velocities.size() > 0)
      {
        getCubicSplineCoefficients(
          prev_positions[j], prev_velocities[j],
          positions[j], velocities[j],
          durations[i],
          seg.splines[j].coef);
        seg.splines[j].coef.resize(6, 0.0);
      }
      else
      {
        seg.splines[j].coef[0] = prev_positions[j];
        if (durations[i] == 0.0)
          seg.splines[j].coef[1] = 0.0;
        else
          seg.splines[j].coef[1] = (positions[j] - prev_positions[j]) / durations[i];
        seg.splines[j].coef[2] = 0.0;
        seg.splines[j].coef[3] = 0.0;
        seg.splines[j].coef[4] = 0.0;
        seg.splines[j].coef[5] = 0.0;
      }
    }
    // Pushes the splines onto the end of the new trajectory.

    traj.push_back(seg);

    // Computes the starting conditions for the next segment

    prev_positions = positions;
    prev_velocities = velocities;
    prev_accelerations = accelerations;
  }

  // ------ Commits the new trajectory

  if (!new_traj_ptr)
  {
    ROS_ERROR("The new trajectory was null!");
    return;
  }

  ROS_DEBUG("The new trajectory has %d segments", (int)traj.size());

  std::vector<sr_robot_msgs::joint> joint_vector_traj;
  unsigned int controller_pub_idx=0;
  //only one of these 2 will be used
  std_msgs::Float64 target_msg;
  sr_robot_msgs::sendupdate sendupdate_msg_traj;

  //initializes the joint names
  //TODO check if traj only contains joint that we control
  //joint_names_ = goal->trajectory.joint_names;
  joint_vector_traj.clear();

  for(unsigned int i = 0; i < joint_names_.size(); ++i)
  {
    sr_robot_msgs::joint joint;
    joint.joint_name = joint_names_[i];
    joint_vector_traj.push_back(joint);
  }

  if(use_sendupdate)
  {
    sendupdate_msg_traj.sendupdate_length = joint_vector_traj.size();
    ROS_DEBUG("Trajectory received: %d joints / %d msg length", (int)msg->joint_names.size(), sendupdate_msg_traj.sendupdate_length);
  }

  ros::Rate tmp_rate(1.0);

//  std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = goal->trajectory.points;
//  trajectory_msgs::JointTrajectoryPoint trajectory_step;

  //loop through the steps
  ros::Duration sleeping_time(0.0);
  ROS_DEBUG("Entering the execution loop");

  while(ros::ok())
  {
    ros::Time time = ros::Time::now();
    ros::Duration dt = time - last_time_;
    last_time_ = time;

    // ------ Finds the current segment
    ROS_DEBUG("Find current segment");

    // Determines which segment of the trajectory to use.  (Not particularly realtime friendly).
    int seg = -1;
    while (seg + 1 < (int)traj.size() && traj[seg+1].start_time < time.toSec())
    {
      ++seg;
    }

    // if the last trajectory is already in the past, stop the servoing
    if( (traj[traj.size()-1].start_time+traj[traj.size()-1].duration) < time.toSec())
    {
      ROS_DEBUG("trajectory is finished %f<%f",(traj[traj.size()-1].start_time+traj[traj.size()-1].duration),time.toSec());
       break;
    }

    if (seg == -1)
    {
      if (traj.size() == 0)
        ROS_ERROR("No segments in the trajectory");
      else
        ROS_ERROR("No earlier segments.  First segment starts at %.3lf (now = %.3lf)", traj[0].start_time, time.toSec());
      return;
    }

    // ------ Trajectory Sampling
    ROS_DEBUG("Sample the trajectory");

    for (size_t i = 0; i < q.size(); ++i)
    {
      sampleSplineWithTimeBounds(traj[seg].splines[i].coef, traj[seg].duration,
                                 time.toSec() - traj[seg].start_time,
                                 q[i], qd[i], qdd[i]);
    }
    ROS_DEBUG("Sampled the trajectory");
    //check if preempted
    if (!ros::ok())
    {
      ROS_INFO("Joint Trajectory Stopping");
      // set the action state to preempted
      //action_server->setPreempted();
      success = false;
      break;
    }
    ROS_DEBUG("Update the targets");
    //update the targets and publish target joint_states
    sensor_msgs::JointState desired_joint_state_msg;
    for(unsigned int i = 0; i < joint_names_.size(); ++i)
    {
      desired_joint_state_msg.name.push_back(joint_names_[i]);
      desired_joint_state_msg.position.push_back(q[i]);
      desired_joint_state_msg.velocity.push_back(qd[i]);
      desired_joint_state_msg.effort.push_back(0.0);
      if(!use_sendupdate)
      {
        if((controller_pub_idx=jointPubIdxMap[ joint_names_[i] ])>0) // if a controller exist for this joint
        {
          target_msg.data=q[i];
          controller_publishers.at(controller_pub_idx-1).publish(target_msg);
        }
      }
      else
      {
        joint_vector_traj[i].joint_target = q[i] * 57.3;
        ROS_DEBUG("traj[%s]: %f", joint_vector_traj[i].joint_name.c_str(), joint_vector_traj[i].joint_target);
      }
    }
    ROS_DEBUG("Targets updated");

    desired_joint_state_msg.header.stamp = ros::Time::now();
    desired_joint_state_pusblisher.publish(desired_joint_state_msg);

    if(use_sendupdate)
    {
      sendupdate_msg_traj.sendupdate_list = joint_vector_traj;
      sr_arm_target_pub.publish(sendupdate_msg_traj);
      sr_hand_target_pub.publish(sendupdate_msg_traj);
    }

    ROS_DEBUG("Now sleep and loop");
    sleeping_time.sleep();
    sleeping_time = ros::Duration(0.1);
    ROS_DEBUG("redo loop");
  }

  return;
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_joint_trajectory_action_controller");

  ros::AsyncSpinner spinner(1); //Use 1 thread
  spinner.start();
  shadowrobot::JointTrajectoryActionController jac;
  ros::spin();
  //ros::waitForShutdown();

  return 0;
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/



