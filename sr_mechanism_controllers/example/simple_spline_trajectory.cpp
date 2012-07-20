// http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class ShadowTrajectory
{
private:
  // Action client for the joint trajectory action
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  ShadowTrajectory()
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~ShadowTrajectory()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now()- ros::Duration(0.01);
    traj_client_->sendGoal(goal);
  }

  //! Wait for currently running trajectory to finish
  void waitTrajectory() {
    while(!getState().isDone() && ros::ok()) { usleep(50000); }
  }

  //! Generates a simple trajectory to move the arm.
  /*! Note that this trajectory contains three waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal arm_movement()
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("ShoulderJRotate");
    goal.trajectory.joint_names.push_back("ShoulderJSwing");
    goal.trajectory.joint_names.push_back("ElbowJSwing");
    goal.trajectory.joint_names.push_back("ElbowJRotate");
    goal.trajectory.joint_names.push_back("WRJ2");
    goal.trajectory.joint_names.push_back("WRJ1");

    // Set number of waypoints in this goal trajectory
    goal.trajectory.points.resize(3);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(6);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 1.57;
    goal.trajectory.points[ind].positions[3] = -0.78;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;

    // Points also have velocities
    goal.trajectory.points[ind].velocities.resize(6);
    goal.trajectory.points[ind].velocities[0] = 0.0;
    goal.trajectory.points[ind].velocities[1] = 0.0;
    goal.trajectory.points[ind].velocities[2] = 0.0;
    goal.trajectory.points[ind].velocities[3] = 0.0;
    goal.trajectory.points[ind].velocities[4] = 0.0;
    goal.trajectory.points[ind].velocities[5] = 0.0;

    // To be reached 4.0 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(10.0);

    // 2nd trajectory point
    ind += 1;
    goal.trajectory.points[ind].positions.resize(6);
    goal.trajectory.points[ind].positions[0] = 0.4;
    goal.trajectory.points[ind].positions[1] = 0.78;
    goal.trajectory.points[ind].positions[2] = 0.78;
    goal.trajectory.points[ind].positions[3] = 0.78;
    goal.trajectory.points[ind].positions[4] = 0.1;
    goal.trajectory.points[ind].positions[5] = 0.1;

    // Points also have velocities
    goal.trajectory.points[ind].velocities.resize(6);
    goal.trajectory.points[ind].velocities[0] = 0.3;
    goal.trajectory.points[ind].velocities[1] = 0.0;
    goal.trajectory.points[ind].velocities[2] = 0.0;
    goal.trajectory.points[ind].velocities[3] = 0.0;
    goal.trajectory.points[ind].velocities[4] = 0.0;
    goal.trajectory.points[ind].velocities[5] = 0.0;

    // To be reached 4.0 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(16.0);
    // 3rd trajectory point
    ind += 1;
    goal.trajectory.points[ind].positions.resize(6);
    goal.trajectory.points[ind].positions[0] = 0.6;
    goal.trajectory.points[ind].positions[1] = 0.48;
    goal.trajectory.points[ind].positions[2] = 0.78;
    goal.trajectory.points[ind].positions[3] = 0.78;
    goal.trajectory.points[ind].positions[4] = 0.1;
    goal.trajectory.points[ind].positions[5] = 0.1;

    // Points also have velocities
    goal.trajectory.points[ind].velocities.resize(6);
    goal.trajectory.points[ind].velocities[0] = 0.0;
    goal.trajectory.points[ind].velocities[1] = 0.0;
    goal.trajectory.points[ind].velocities[2] = 0.0;
    goal.trajectory.points[ind].velocities[3] = 0.0;
    goal.trajectory.points[ind].velocities[4] = 0.0;
    goal.trajectory.points[ind].velocities[5] = 0.0;

    // To be reached 4.0 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(20.0);

    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }

};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "shadow_trajectory_driver");

  ShadowTrajectory sj;
  sj.startTrajectory(sj.arm_movement());
  sj.waitTrajectory();
}

// vim: sw=2:ts=2
