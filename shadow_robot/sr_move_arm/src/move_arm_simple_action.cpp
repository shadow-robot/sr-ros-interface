/*
enum MoveArmState {
  PLANNING,
  START_CONTROL,
  VISUALIZE_PLAN,
  MONITOR,
  PAUSE
};


    action_server_.reset(new actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction>(root_handle_, "move_" + group_name, boost::bind(&MoveArm::execute, this, _1)));


void execute(const move_arm_msgs::MoveArmGoalConstPtr& goal)
  {
    motion_planning_msgs::GetMotionPlan::Request req;	    
    moveArmGoalToPlannerRequest(goal,req);	    
    original_request_ = req;
    ROS_INFO("Received new goal");
    ros::Rate move_arm_rate(move_arm_frequency_);
    move_arm_action_result_.contacts.clear();
    move_arm_action_result_.error_code.val = 0;
    move_arm_stats_.time_to_execution = ros::Time::now().toSec();
    move_arm_stats_.time_to_result = ros::Time::now().toSec();
    while(private_handle_.ok())
    {	    	    
      if (action_server_->isPreemptRequested())
      {
        move_arm_stats_.preempted = true;
        if(publish_stats_)
          publishStats();
        move_arm_stats_.time_to_execution = ros::Time::now().toSec();
        move_arm_stats_.time_to_result = ros::Time::now().toSec();
        if(action_server_->isNewGoalAvailable())
        {
          move_arm_action_result_.contacts.clear();
          move_arm_action_result_.error_code.val = 0;
          moveArmGoalToPlannerRequest((action_server_->acceptNewGoal()),req);
          ROS_INFO("Received new goal, will preempt previous goal");
          original_request_ = req;
          if (controller_status_ == QUEUED || controller_status_ == ACTIVE)
            stopTrajectory();
          state_ = PLANNING;
        }
        else               //if we've been preempted explicitly we need to shut things down
        {
          ROS_INFO("The move arm action was preempted by the action client. Preempting this goal.");
          if (controller_status_ == QUEUED || controller_status_ == ACTIVE)
            stopTrajectory();
          resetStateMachine();
          action_server_->setPreempted();
          return;
        }
      }

      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      bool done = executeCycle(req);


      if(done)
      {
        if(publish_stats_)
          publishStats();
        return;
      }

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG("Full control cycle time: %.9f\n", t_diff.toSec());

      move_arm_rate.sleep();
    }	    
    //if the node is killed then we'll abort and return
    ROS_INFO("Node was killed, aborting");
    action_server_->setAborted(move_arm_action_result_);
  }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm");
  
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start();
  ros::NodeHandle nh("~");
  std::string group;
  nh.param<std::string>("group", group, std::string());
  ROS_INFO("Move arm operating on group %s",group.c_str());    
  move_arm::MoveArm move_arm(group);
  if(!move_arm.configure())
  {
    ROS_ERROR("Could not configure move arm, exiting");
    ros::shutdown();
    return 1;
  }
  ROS_INFO("Move arm action started");
  ros::waitForShutdown();
    
  return 0;
}
*/
