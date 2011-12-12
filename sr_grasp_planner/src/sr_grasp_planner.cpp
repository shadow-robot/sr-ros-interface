/**
 * @file   sr_grasp_planner.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Mar 22 10:37:43 2011
 *
 * @brief  Plans grasp for an object that was not recognized.
 *
 *
 */

#include "sr_grasp_planner/sr_grasp_planner.hpp"
#include <math.h>
#include <tf/tf.h>
#include <sstream>

#include <boost/assign/std/vector.hpp>

using namespace boost::assign;

namespace shadowrobot
{
  const double SrGraspPlanner::default_approach_distance = 0.2;
  const unsigned short SrGraspPlanner::default_number_of_computed_grasps = 20;

  SrGraspPlanner::SrGraspPlanner()
  {
    std::vector<std::string> names;
    names += "FFJ0", "FFJ3", "FFJ4",
      "MFJ0", "MFJ3", "MFJ4",
      "RFJ0", "RFJ3", "RFJ4",
      "LFJ0", "LFJ3", "LFJ4", "LFJ5",
      "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
      "WRJ1", "WRJ2";

    pregrasp.name = names;
    grasp.name = names;

    //TODO: change those for a proper grasp / pregrasp
    std::vector<double> pregrasp_position;
    pregrasp_position += 0.0, 0.0, 0.0,     //FF
      0.0, 0.0, 0.0,                        //MF
      0.0, 0.0, 0.0,                        //RF
      0.0, 0.0, 0.0, 0.0,                   //LF
      0.0, 0.0, 0.0, 0.0, 0.0,              //TH
      -45.0, 10.0;                             //WR

    pregrasp.position = pregrasp_position;

    std::vector<double> grasp_position;
    grasp_position += 115.0, 90.0, 0.0,     //FF
      180.0, 90.0, 0.0,                     //MF
      180.0, 90.0, 0.0,                     //RF
      180.0, 90.0, 0.0, 0.0,                //LF
      45.0, 30.0, 0.0, 48.0, 35.0,             //TH
      -45.0, 10.0;                            //WR

    grasp.position = grasp_position;

    //initialize the tf listener and broadcaster
    tf_listener = boost::shared_ptr<tf::TransformListener>( new tf::TransformListener() );
    tf_broadcaster = boost::shared_ptr<tf::TransformBroadcaster>( new tf::TransformBroadcaster() );
  }

  SrGraspPlanner::~SrGraspPlanner()
  {}

  std::vector<object_manipulation_msgs::Grasp> SrGraspPlanner::compute_list_of_grasps(object_manipulation_msgs::ClusterBoundingBox bounding_box)
  {
    //get the main axis
    Eigen::Vector3d main_axis = get_main_axis(bounding_box);

    std::vector<object_manipulation_msgs::Grasp> possible_grasps;

    //compute the grasps: they're placed on a cylinder surounding the
    //biggest axis
    for (unsigned short i = 0; i < default_number_of_computed_grasps; ++i)
    {
      object_manipulation_msgs::Grasp tmp_grasp;

      tmp_grasp.pre_grasp_posture = pregrasp;
      tmp_grasp.grasp_posture = grasp;

      geometry_msgs::Pose grasp_pose;
      //the default grasp comes from above the main axis
      if (main_axis[2] == 1)
      {
        tmp_grasp.grasp_pose = compute_pose(i, true, bounding_box);
      }
      else
      {
        tmp_grasp.grasp_pose = compute_pose(i, false, bounding_box);
      }

      tmp_grasp.desired_approach_distance = static_cast<double>(i);

      possible_grasps.push_back(tmp_grasp);
    }

    return possible_grasps;
  }

  geometry_msgs::Pose SrGraspPlanner::compute_pose(unsigned int index_pose, bool is_vertical,
                                                   object_manipulation_msgs::ClusterBoundingBox bounding_box)
  {
    /*
      tf::Quaternion object_rotation(bounding_box.pose_stamped.pose.orientation.x,
      bounding_box.pose_stamped.pose.orientation.y,
      bounding_box.pose_stamped.pose.orientation.z,
      bounding_box.pose_stamped.pose.orientation.w);
    */

    if( index_pose == 0)
    {
      if( is_vertical )
        ROS_INFO("Computing grasps for a vertical object");
      else
        ROS_INFO("Computing grasps for an horizontal object");
    }

    //transform between base_link and the bottom-middle of the bounding box,
    // with no rotation
    tf::Quaternion tmp_quat;
    tmp_quat.setRPY(0.0,0.0,0.0);

    tf::Transform base_to_bottom_middle_tf;
    base_to_bottom_middle_tf.setOrigin( tf::Vector3(bounding_box.pose_stamped.pose.position.x + bounding_box.dimensions.x / 2.0,
                                                    bounding_box.pose_stamped.pose.position.y + bounding_box.dimensions.y / 2.0,
                                                    bounding_box.pose_stamped.pose.position.z + bounding_box.dimensions.z / 2.0) );
    base_to_bottom_middle_tf.setRotation( tmp_quat );

    double object_height = bounding_box.dimensions.z;

    std::stringstream tf_name_base_to_bot_mid;
    tf_name_base_to_bot_mid << "bot_mid_obj_" << index_pose;
    tf_broadcaster->sendTransform( tf::StampedTransform(base_to_bottom_middle_tf,
                                                        ros::Time::now(),
                                                        bounding_box.pose_stamped.header.frame_id,
                                                        tf_name_base_to_bot_mid.str()) );

    //now rotate correctly depending on if we're trying to
    // grasp an horizontal or a vertical object.
    tf::StampedTransform bottom_middle_to_base_orientation_tf;
    //get correct rotation: we want to rotate to have the same
    // orientation as the base_link
    if( tf_listener->waitForTransform(tf_name_base_to_bot_mid.str(), "/base_link",
                                      ros::Time(), ros::Duration(1.0) ) )
    {
      tf_listener->lookupTransform(tf_name_base_to_bot_mid.str(), "/base_link",
                                   ros::Time(0),
                                   bottom_middle_to_base_orientation_tf);
    }
    else
    {
      ROS_FATAL_STREAM("Couldn't get the transform between /base_link and " << tf_name_base_to_bot_mid.str() );
      ROS_BREAK();
    }


    //but we don't want to translate to the base_link, just to rotate:
    bottom_middle_to_base_orientation_tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );


    //if it's an horizontal object, we need to grasp around the long axis
    // so we need one more rotation, to set the z axis of the frame
    // so we rotate from 90 degrees around y
    if( !is_vertical )
    {
      tmp_quat.setRPY( 0.0, 1.57079633, 0.0 );
      tf::Quaternion current_quat = bottom_middle_to_base_orientation_tf.getRotation();

      tmp_quat += current_quat;
      bottom_middle_to_base_orientation_tf.setRotation( tmp_quat );
    }

    std::stringstream tf_name_bot_mid_to_base_or;
    tf_name_bot_mid_to_base_or << "bot_mid_obj_base_or_" << index_pose;
    tf_broadcaster->sendTransform( tf::StampedTransform(bottom_middle_to_base_orientation_tf,
                                                        ros::Time::now(),
                                                        tf_name_base_to_bot_mid.str(),
                                                        tf_name_bot_mid_to_base_or.str()) );


    //Now we need to rotate of the correct angle
    tf::Transform bottom_middle_to_correct_orientation_tf;
    tmp_quat.setRPY(0, 0, 2.0 * M_PI * double(index_pose) / double(default_number_of_computed_grasps) );
    bottom_middle_to_correct_orientation_tf.setRotation( tmp_quat );
    bottom_middle_to_correct_orientation_tf.setOrigin( tf::Vector3(0.0, 0.0, object_height / 2.0) );

    std::stringstream tf_name_bot_mid_to_correct_or;
    tf_name_bot_mid_to_correct_or << "bot_mid_obj_correct_or_" << index_pose;
    tf_broadcaster->sendTransform( tf::StampedTransform(bottom_middle_to_correct_orientation_tf,
                                                        ros::Time::now(),
                                                        tf_name_bot_mid_to_base_or.str(),
                                                        tf_name_bot_mid_to_correct_or.str()) );

    //then just translate a given value along X axis
    // to the default approach distance
    tf::Transform bottom_middle_to_pregrasp_tf;
    bottom_middle_to_pregrasp_tf.setOrigin( tf::Vector3( default_approach_distance, 0.0, 0.0 ) );
    tmp_quat.setRPY(0.0,0.0,0.0);
    bottom_middle_to_pregrasp_tf.setRotation( tmp_quat );

    std::stringstream tf_name_bot_mid_to_preg;
    tf_name_bot_mid_to_preg << "mid_obj_preg_" << index_pose;
    tf_broadcaster->sendTransform( tf::StampedTransform(bottom_middle_to_pregrasp_tf,
                                                        ros::Time::now(),
                                                        tf_name_bot_mid_to_correct_or.str(),
                                                        tf_name_bot_mid_to_preg.str()) );

    //Finally, we get a pose for the grasp in the base_link
    // frame, and we return this.
    geometry_msgs::PoseStamped grasp_pose_base_frame, grasp_pose_pose_frame;
    grasp_pose_pose_frame.header.stamp = ros::Time::now();
    grasp_pose_pose_frame.header.frame_id = tf_name_bot_mid_to_preg.str();
    grasp_pose_pose_frame.pose.position.x = 0.0;
    grasp_pose_pose_frame.pose.position.y = 0.0;
    grasp_pose_pose_frame.pose.position.z = 0.0;

    grasp_pose_pose_frame.pose.orientation.x = 0.0;
    grasp_pose_pose_frame.pose.orientation.y = 0.0;
    grasp_pose_pose_frame.pose.orientation.z = 0.0;
    grasp_pose_pose_frame.pose.orientation.w = 1.0;


    bool success = false;
    for(unsigned int i = 0; i < 5000; ++i)
    {
      tf_broadcaster->sendTransform( tf::StampedTransform(base_to_bottom_middle_tf,
                                                          ros::Time::now(),
                                                          bounding_box.pose_stamped.header.frame_id,
                                                          tf_name_base_to_bot_mid.str()) );
      ros::spinOnce();

      tf_broadcaster->sendTransform( tf::StampedTransform(bottom_middle_to_base_orientation_tf,
                                                          ros::Time::now(),
                                                          tf_name_base_to_bot_mid.str(),
                                                          tf_name_bot_mid_to_base_or.str()) );
      ros::spinOnce();
      tf_broadcaster->sendTransform( tf::StampedTransform(bottom_middle_to_correct_orientation_tf,
                                                          ros::Time::now(),
                                                          tf_name_bot_mid_to_base_or.str(),
                                                          tf_name_bot_mid_to_correct_or.str()) );
      ros::spinOnce();
      tf_broadcaster->sendTransform( tf::StampedTransform(bottom_middle_to_pregrasp_tf,
                                                          ros::Time::now(),
                                                          tf_name_bot_mid_to_correct_or.str(),
                                                          tf_name_bot_mid_to_preg.str()) );
      ros::spinOnce();


      try
      {
        tf_listener->transformPose("/base_link", grasp_pose_pose_frame,
                                   grasp_pose_base_frame);
        success = true;
        break;
      }
      catch(tf::TransformException exc)
      {
        continue;
      }
    }
    if( !success )
    {
      ROS_WARN_STREAM("Couldn't get the transform from the base_link to the grasp.");
    }

    return grasp_pose_base_frame.pose;
  }

  Eigen::Vector3d SrGraspPlanner::get_main_axis(object_manipulation_msgs::ClusterBoundingBox bbox)
  {
    Eigen::Vector3d main_axis;
    double max = 0;
    if( fabs(bbox.dimensions.x) > max )
    {
      max = bbox.dimensions.x;
      main_axis[0] = 1.0;
    }
    if( fabs(bbox.dimensions.y) > max )
    {
      max = bbox.dimensions.y;
      main_axis[1] = 1.0;
    }
    if( fabs(bbox.dimensions.z) > max )
    {
      max = bbox.dimensions.z;
      main_axis[2] = 1.0;
    }

    return main_axis;
  }

}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
