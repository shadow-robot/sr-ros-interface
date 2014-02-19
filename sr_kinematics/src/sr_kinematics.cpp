// bsd license blah blah
// majority of this code comes from package urdf_tool/arm_kinematics
// written by David Lu!!
// the IK is our own computation the FK will be in a near future.

#include <cstring>
#include <cmath>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <urdf/model.h>
#include <string>

using std::string;

static const std::string IK_SERVICE = "get_ik";
static const std::string FK_SERVICE = "get_fk";
static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
static const std::string FK_INFO_SERVICE = "get_fk_solver_info";

#define IK_EPS	1e-5

//taken from PR2_kinematics_utils... should we link to this or ?
bool solveQuadratic(const double &a, const double &b, const double &c, double *x1, double *x2)
{
  double discriminant = b * b - 4 * a * c;
  if (fabs(a) < IK_EPS)
  {
    *x1 = -c / b;
    *x2 = *x1;
    return true;
  }
  //ROS_DEBUG("Discriminant: %f\n",discriminant);
  if (discriminant >= 0)
  {
    *x1 = (-b + sqrt(discriminant)) / (2 * a);
    *x2 = (-b - sqrt(discriminant)) / (2 * a);
    return true;
  }
  else if (fabs(discriminant) < IK_EPS)
  {
    *x1 = -b / (2 * a);
    *x2 = -b / (2 * a);
    return true;
  }
  else
  {
    *x1 = -b / (2 * a);
    *x2 = -b / (2 * a);
    return false;
  }
}

class Kinematics
{
public:
  Kinematics();
  bool init();

private:
  ros::NodeHandle nh, nh_private;
  std::string root_name, finger_base_name, tip_name;
  KDL::JntArray joint_min, joint_max;
  KDL::Chain chain;
  unsigned int num_joints;
  std::vector<urdf::Pose> link_offset;
  std::vector<std::string> link_offset_name;
  std::vector<urdf::Vector3> knuckle_axis;

  KDL::ChainFkSolverPos_recursive* fk_solver;

  double epsilon;
  double length_middle;
  double length_proximal;
  unsigned int J5_idx_offset; //LF has an additional joint (1 when LF, 0 otherwise)

  ros::ServiceServer ik_service, ik_solver_info_service;
  ros::ServiceServer fk_service, fk_solver_info_service;

  tf::TransformListener tf_listener;

  moveit_msgs::KinematicSolverInfo info;

  bool loadModel(const std::string xml);
  bool readJoints(urdf::Model &robot_model);
  int getJointIndex(const std::string &name);
  int getKDLSegmentIndex(const std::string &name);

  /**
   * @brief This is the basic IK service method that will compute and return an IK solution.
   * @param A request message. See service definition for GetPositionIK for more information on this message.
   * @param The response message. See service definition for GetPositionIK for more information on this message.
   */
  bool getPositionIK(moveit_msgs::GetPositionIK::Request &request, moveit_msgs::GetPositionIK::Response &response);

  /**
   * @brief This is the basic kinematics info service that will return information about the kinematics node.
   * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
   * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
   */
  bool getIKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                       moveit_msgs::GetKinematicSolverInfo::Response &response);

  /**
   * @brief This is the basic kinematics info service that will return information about the kinematics node.
   * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
   * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
   */
  bool getFKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                       moveit_msgs::GetKinematicSolverInfo::Response &response);

  /**
   * @brief This is the basic forward kinematics service that will return information about the kinematics node.
   * @param A request message. See service definition for GetPositionFK for more information on this message.
   * @param The response message. See service definition for GetPositionFK for more information on this message.
   */
  bool getPositionFK(moveit_msgs::GetPositionFK::Request &request, moveit_msgs::GetPositionFK::Response &response);
};

Kinematics::Kinematics() :
    nh_private("~"), epsilon(1e-2), length_proximal(.0), length_middle(.0), num_joints(0),
    J5_idx_offset(0), fk_solver(NULL)
{
}

bool Kinematics::init()
{
  // Get URDF XML
  std::string urdf_xml, full_urdf_xml;
  nh.param("urdf_xml", urdf_xml, std::string("robot_description"));
  nh.searchParam(urdf_xml, full_urdf_xml);
  ROS_DEBUG("Reading xml file from parameter server");
  std::string result;
  if (!nh.getParam(full_urdf_xml, result))
  {
    ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  // Get Root and Tip From Parameter Service
  if (!nh_private.getParam("root_name", root_name))
  {
    ROS_FATAL("GenericIK: No root name found on parameter server");
    return false;
  }
  if (root_name != "palm")
  {
    ROS_FATAL("Current solver can only resolve to root frame = palm");
    return false;
  }

  if (!nh_private.getParam("tip_name", tip_name))
  {
    ROS_FATAL("GenericIK: No tip name found on parameter server");
    return false;
  }

  if (tip_name.find("distal") == string::npos)
  {
    ROS_FATAL("Current solver can only resolve to one of the distal frames");
    return false;
  }

  if (tip_name.find("lfdistal") != string::npos)
  {
    ROS_INFO("Computing LF IK not considering J5");
    J5_idx_offset = 1;
  }
  else
    J5_idx_offset = 0;

  if (tip_name.find("thdistal") != string::npos)
  {
    ROS_FATAL("Current solver cannot resolve to the thdistal frame");
    return false;
  }

  finger_base_name = tip_name.substr(0, 2);
  finger_base_name.append("knuckle");
  ROS_INFO("base_finger name %s", finger_base_name.c_str());

  // Load and Read Models
  if (!loadModel(result))
  {
    ROS_FATAL("Could not load models!");
    return false;
  }

  // Get Solver Parameters
  int maxIterations;

  nh_private.param("maxIterations", maxIterations, 1000);
  nh_private.param("epsilon", epsilon, 1e-2);

  // Build Solvers
  fk_solver = new KDL::ChainFkSolverPos_recursive(chain); //keep the standard arm_kinematics fk_solver although we have ours.

  ROS_INFO("Advertising services");
  fk_service = nh_private.advertiseService(FK_SERVICE, &Kinematics::getPositionFK, this);
  ik_service = nh_private.advertiseService(IK_SERVICE, &Kinematics::getPositionIK, this);
  ik_solver_info_service = nh_private.advertiseService(IK_INFO_SERVICE, &Kinematics::getIKSolverInfo, this);
  fk_solver_info_service = nh_private.advertiseService(FK_INFO_SERVICE, &Kinematics::getFKSolverInfo, this);

  return true;
}

bool Kinematics::loadModel(const std::string xml)
{
  urdf::Model robot_model;
  KDL::Tree tree;

  if (!robot_model.initString(xml))
  {
    ROS_FATAL("Could not initialize robot model");
    return -1;
  }
  if (!kdl_parser::treeFromString(xml, tree))
  {
    ROS_ERROR("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(root_name, tip_name, chain))
  {
    ROS_ERROR("Could not initialize chain object");
    return false;
  }

  if (!readJoints(robot_model))
  {
    ROS_FATAL("Could not read information about the joints");
    return false;
  }

  return true;
}

bool Kinematics::readJoints(urdf::Model &robot_model)
{
  num_joints = 0;
  // get joint maxs and mins
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
  boost::shared_ptr<const urdf::Joint> joint;

  bool length_proximal_done = false;
  bool length_middle_done = false;
  urdf::Vector3 length;

  while (link && link->name != root_name)
  {
    //extract knuckle to palm_offset.
    if (link->name.find("knuckle") != string::npos)
    {
      if (link->getParent()->name == "palm") // FF,MF,RF
      {
        link_offset.push_back(link->parent_joint->parent_to_joint_origin_transform);
        link_offset_name.push_back(link->name);
        knuckle_axis.push_back(link->parent_joint->axis);
      }
      else // LF
      {
        //temp store the first offset
        urdf::Pose first_offset = link->parent_joint->parent_to_joint_origin_transform;
        std::string link_name = link->name;
        //check for palm in parent of parent
        if (link->getParent()->getParent()->name == "palm")
        {
          urdf::Pose second_offset = link->getParent()->parent_joint->parent_to_joint_origin_transform;
          urdf::Pose combined_offset(first_offset);
          //combine only position (so the pose between palm and lfknuckle may be wrong regarding rotations)
          combined_offset.position.x += second_offset.position.x;
          combined_offset.position.y += second_offset.position.y;
          combined_offset.position.z += second_offset.position.z;
          link_offset.push_back(combined_offset);
          link_offset_name.push_back(link_name);
          knuckle_axis.push_back(link->parent_joint->axis);
        }
        else
        { //stop at level2; structure must be false.
          ROS_ERROR("Could not find palm parent for this finger");
          return false;
        }
      }
    }

    // extract proximal length
    if (!length_proximal_done)
    {
      if (link->name.find("middle") != string::npos)
      {
        length = link->parent_joint->parent_to_joint_origin_transform.position;
        length_proximal = sqrt(length.x * length.x + length.y * length.y + length.z * length.z);
        length_proximal_done = true;
      }
    }

    // extract middle length
    if (!length_middle_done)
    {
      if (link->name.find("distal") != string::npos)
      {
        length = link->parent_joint->parent_to_joint_origin_transform.position;
        length_middle = sqrt(length.x * length.x + length.y * length.y + length.z * length.z);
        length_middle_done = true;
      }
    }

    joint = robot_model.getJoint(link->parent_joint->name);
    if (!joint)
    {
      ROS_ERROR("Could not find joint: %s", link->parent_joint->name.c_str());
      return false;
    }
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      ROS_INFO("adding joint: [%s]", joint->name.c_str());
      num_joints++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }

  joint_min.resize(num_joints);
  joint_max.resize(num_joints);
  info.joint_names.resize(num_joints);
  info.link_names.resize(num_joints);
  info.limits.resize(num_joints);

  link = robot_model.getLink(tip_name);
  unsigned int i = 0;
  while (link && i < num_joints)
  {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      ROS_INFO("getting bounds for joint: [%s]", joint->name.c_str());

      float lower, upper;
      int hasLimits;
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        lower = joint->limits->lower;
        upper = joint->limits->upper;
        hasLimits = 1;
      }
      else
      {
        lower = -M_PI;
        upper = M_PI;
        hasLimits = 0;
      }
      int index = num_joints - i - 1;

      joint_min.data[index] = lower;
      joint_max.data[index] = upper;
      info.joint_names[index] = joint->name;
      info.link_names[index] = link->name;
      info.limits[index].joint_name = joint->name;
      info.limits[index].has_position_limits = hasLimits;
      info.limits[index].min_position = lower;
      info.limits[index].max_position = upper;
      i++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  return true;
}

int Kinematics::getJointIndex(const std::string &name)
{
  for (unsigned int i = 0; i < info.joint_names.size(); i++)
  {
    if (info.joint_names[i] == name)
      return i;
  }
  return -1;
}

int Kinematics::getKDLSegmentIndex(const std::string &name)
{
  int i = 0;
  while (i < (int)chain.getNrOfSegments())
  {
    if (chain.getSegment(i).getName() == name)
    {
      return i + 1;
    }
    i++;
  }
  return -1;
}

bool Kinematics::getPositionIK(moveit_msgs::GetPositionIK::Request &request,
                               moveit_msgs::GetPositionIK::Response &response)
{

  // get the 3D position of requested goal (forget about orientation, which is not relevant)
  geometry_msgs::PointStamped point_msg_in;
  point_msg_in.header.frame_id = request.ik_request.pose_stamped.header.frame_id;
  point_msg_in.header.stamp = ros::Time::now() - ros::Duration(1);
  point_msg_in.point = request.ik_request.pose_stamped.pose.position;

  tf::Stamped<tf::Point> transform;
  tf::Stamped<tf::Point> transform_root;
  tf::Stamped<tf::Point> transform_finger_base;
  tf::pointStampedMsgToTF(point_msg_in, transform);

  urdf::Pose knuckle_offset;
  urdf::Vector3 J4_axis;
  float J4_dir;
  tf::Point p; //local req_point coordinates
  tf::Point pbis; //with different coordinate system

  // IK computation variables
  double l1 = length_proximal, l2 = length_middle;
  double L = 0.0;
  double x1, x2;
  double thetap, thetam, l2p;
  double alpha2;
  float b, c, delta = 0.01;

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(num_joints);
  jnt_pos_out.resize(num_joints);

  //Convert F to our root_frame
  ROS_DEBUG("sr_kin: Get point in root frame");
  try
  {
    tf_listener.transformPoint(root_name, transform, transform_root);
  }
  catch (...)
  {
    ROS_ERROR("Could not transform IK pose to frame: %s", root_name.c_str());
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return false;
  }
  ROS_DEBUG("root x,y,z:%f,%f,%f", transform_root.x(), transform_root.y(), transform_root.z());

  //look for knuckle corresponding to distal phalanx
  ROS_DEBUG("sr_kin: Looking for J4_axis direction and knuckle offset");
  for (unsigned int i = 0; i < link_offset_name.size(); i++)
  {
    if (link_offset_name[i] == finger_base_name)
    {
      knuckle_offset = link_offset[i];
      ROS_DEBUG("knuckle offset is %f,%f,%f", knuckle_offset.position.x, knuckle_offset.position.y,
                knuckle_offset.position.z);
      J4_axis = knuckle_axis[i];
      //ROS_DEBUG("J4 axis is %f,%f,%f",J4_axis.x,J4_axis.y,J4_axis.z);
      ROS_DEBUG("J4 is %s", J4_axis.y > 0 ? "positive" : "negative");
      break;
    }
  }

  // Change to a local computation frame (at knuckle pos but with palm orientation)
  ROS_DEBUG("sr_kin: Convert Frame to local computation frame");
  p.setValue(-knuckle_offset.position.x, -knuckle_offset.position.y, -knuckle_offset.position.z);
  p += transform_root;
  ROS_DEBUG("x,y,z:%f,%f,%f", p.x(), p.y(), p.z());

  // Change frame to the one where maths have been done :-)
  pbis = p;
  pbis.setValue(p.z(), p.x(), -p.y());
  ROS_DEBUG("p2bis %f,%f,%f", pbis.x(), pbis.y(), pbis.z());

  // because J4 are not the same direction for each finger
  // we need to change it according to knuckle axis direction
  ROS_DEBUG("sr_kin: Compute J4");
  J4_dir = J4_axis.y > 0 ? 1.0 : -1.0;

  jnt_pos_out(0) = 0; //to solve the case of LF;
  if (fabs(pbis.x()) - epsilon > 0.0)
  {
    jnt_pos_out(0 + J5_idx_offset) = J4_dir * atan(pbis.y() / pbis.x());
  }
  else
  {
    if (fabs(pbis.y()) - epsilon > 0.0)
    {
      float sign = pbis.y() >= 0 ? 1.0 : -1.0;
      jnt_pos_out(0 + J5_idx_offset) = J4_dir * sign * M_PI_2;
    }
    else
      jnt_pos_out(0 + J5_idx_offset) = 0;
  }

  L = pbis.length2();
  ROS_DEBUG("L %f", L);
  ROS_DEBUG("sr_kin: Solve IK quadratic system");
  if (!solveQuadratic(l1 * l2, l2 * l2 - 3 * l1 * l2, -(L) + l1 * l1, &x2, &x1)) //x1 x2 inversed is normal
  {
    ROS_DEBUG("No solution to quadratic eqn");
    return false;
  }
  ROS_DEBUG("x1,x2 %f,%f", x1, x2);

  // thetap et thetam are the 2 possibilities for each x1 or x2 in the isoceles triangle
  ROS_DEBUG("sr_kin: Check for acceptable solutions");
  if (x2 > 0)
  {
    thetam = acos(sqrt(x2) / 2);
    thetap = acos(-sqrt(x2) / 2);
    l2p = 2 * l2 * (sqrt(x2) / 2);
  }
  else
  {
    if (x1 > 0)
    {
      thetam = acos(sqrt(x1) / 2);
      thetap = acos(-sqrt(x1) / 2);
      l2p = 2 * l2 * (sqrt(x1) / 2);
    }
    else
    {
      thetap = 0;
      thetam = 0;
      l2p = 2 * l2;
    }
  }

  // alpha2 is the virtual angle of the second phalanx in a system with only 2 phalanxes
  ROS_DEBUG("sr_kin: Compute virtual ALPHA");
  ROS_DEBUG("thetap,thetam, l2p %f,%f,%f", thetap, thetam, l2p);
  if (thetap >= 0 && thetap <= M_PI_2)
  {
    alpha2 = 3 * thetap;
  }
  else
  {
    if (thetam <= M_PI_2)
      alpha2 = 3 * thetam;
    else
      alpha2 = 0;
  }

  b = l1 + l2p * cos(alpha2);
  c = l2p * sin(alpha2);

  ROS_DEBUG("sr_kin: Compute J3");
  ROS_DEBUG("alpha2,b,c %f,%f,%f", alpha2, b, c);
  if (pbis.x() < 0)
    jnt_pos_out(1 + J5_idx_offset) = M_PI_2 + atan2(sqrt(pbis.x() * pbis.x() + pbis.y() * pbis.y()), pbis.z())
        - atan2(c, b);
  else
    jnt_pos_out(1 + J5_idx_offset) = atan2(b * pbis.z() - c * sqrt(pbis.x() * pbis.x() + pbis.y() * pbis.y()),
                                           b * sqrt(pbis.x() * pbis.x() + pbis.y() * pbis.y()) + c * pbis.z());

  ROS_DEBUG("sr_kin: Compute J0=J1+J2");
  jnt_pos_out(2 + J5_idx_offset) = 2.0 * alpha2 / 3.0; // sum of the angles of the last 2 phalanxes divided by 2 to get individual phalanx angle in the coupled joint
  jnt_pos_out(3 + J5_idx_offset) = jnt_pos_out(2 + J5_idx_offset);

  for (unsigned int i = 0; i < num_joints; i++)
  {
    ROS_DEBUG("limit min-max %d, %f , %f", i, joint_min(i), joint_max(i));
    ROS_DEBUG("pos %d,%f", i, jnt_pos_out(i));
    if (jnt_pos_out(i) > joint_max(i) + delta || jnt_pos_out(i) < joint_min(i) - delta || isnan(jnt_pos_out(i)))
    {
      jnt_pos_out(0 + J5_idx_offset) = 0;
      jnt_pos_out(1 + J5_idx_offset) = 0;
      jnt_pos_out(2 + J5_idx_offset) = 0;
      jnt_pos_out(3 + J5_idx_offset) = 0;
      return 0;
    }
  }

  int ik_valid = 1;

  if (ik_valid >= 0)
  {
    response.solution.joint_state.name = info.joint_names;
    response.solution.joint_state.position.resize(num_joints);
    for (unsigned int i = 0; i < num_joints; i++)
    {
      response.solution.joint_state.position[i] = jnt_pos_out(i);
      ROS_DEBUG("IK Solution: %s %d: %f", response.solution.joint_state.name[i].c_str(), i, jnt_pos_out(i));
    }
    response.error_code.val = response.error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");
    response.error_code.val = response.error_code.NO_IK_SOLUTION;
    return true;
  }
  return true;
}

bool Kinematics::getIKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                                 moveit_msgs::GetKinematicSolverInfo::Response &response)
{
  response.kinematic_solver_info = info;
  return true;
}

bool Kinematics::getFKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                                 moveit_msgs::GetKinematicSolverInfo::Response &response)
{
  response.kinematic_solver_info = info;
  return true;
}

bool Kinematics::getPositionFK(moveit_msgs::GetPositionFK::Request &request,
                               moveit_msgs::GetPositionFK::Response &response)
{
  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  jnt_pos_in.resize(num_joints);
  for (unsigned int i = 0; i < num_joints; i++)
  {
    int tmp_index = getJointIndex(request.robot_state.joint_state.name[i]);
    if (tmp_index >= 0)
      jnt_pos_in(tmp_index) = request.robot_state.joint_state.position[i];
  }

  response.pose_stamped.resize(request.fk_link_names.size());
  response.fk_link_names.resize(request.fk_link_names.size());

  bool valid = true;
  for (unsigned int i = 0; i < request.fk_link_names.size(); i++)
  {
    int segmentIndex = getKDLSegmentIndex(request.fk_link_names[i]);
    ROS_DEBUG("End effector index: %d", segmentIndex);
    ROS_DEBUG("Chain indices: %d", chain.getNrOfSegments());
    if (fk_solver->JntToCart(jnt_pos_in, p_out, segmentIndex) >= 0)
    {
      tf_pose.frame_id_ = root_name;
      tf_pose.stamp_ = ros::Time();
      tf::poseKDLToTF(p_out, tf_pose);
      try
      {
        tf_listener.transformPose(request.header.frame_id, tf_pose, tf_pose);
      }
      catch (...)
      {
        ROS_ERROR("Could not transform FK pose to frame: %s", request.header.frame_id.c_str());
        response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
        return false;
      }
      tf::poseStampedTFToMsg(tf_pose, pose);
      response.pose_stamped[i] = pose;
      response.fk_link_names[i] = request.fk_link_names[i];
      response.error_code.val = response.error_code.SUCCESS;
    }
    else
    {
      ROS_ERROR("Could not compute FK for %s", request.fk_link_names[i].c_str());
      response.error_code.val = response.error_code.NO_IK_SOLUTION;
      valid = false;
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sr_kinematics");
  Kinematics k;
  if (k.init() < 0)
  {
    ROS_ERROR("Could not initialize kinematics node");
    return -1;
  }

  ros::spin();
  return 0;
}

