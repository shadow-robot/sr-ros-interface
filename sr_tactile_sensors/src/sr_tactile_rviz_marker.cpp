#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//#include <geometry_msgs/Point.h>

#include <string>

#include <boost/thread.hpp>

//messages
#include <std_msgs/Float64.h>

//a ros subscriber (will be instantiated later on)
ros::Subscriber sub[5];
ros::Publisher marker_pub;
std_msgs::Float64::_data_type data[5];
boost::mutex update_mutex;
float reduction_factor=10;
// Set our initial shape type to be a cube
uint32_t shape = visualization_msgs::Marker::ARROW;


void publish_marker(unsigned int id, std::string framesuffix, float force)
{
  if(force>0.01)
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/srh/position/"+framesuffix;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "touch";
    marker.id = id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point mypoint;
    // Set the pose of the marker.
    // Set start point
    float phalanx_thickness=0.007;
    float phalanx_length=0.01;
    mypoint.x= 0;
    mypoint.y= phalanx_thickness;
    mypoint.z= phalanx_length;
    marker.points.push_back(mypoint);

    // Set end point
    mypoint.x= 0;
    mypoint.y= (phalanx_thickness+force/20.0);
    mypoint.z= phalanx_length;
    marker.points.push_back(mypoint);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.0025;
    marker.scale.y = 0.004;
    marker.scale.z = 0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = (force)>0.0?(force):0.0;
    marker.color.g = (1.0f-force)>0.0?(1.0f-force):0.0;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);
  }
}


void callback_ff(const std_msgs::Float64ConstPtr& msg)
{
  update_mutex.lock();
  data[0] = msg->data;
  publish_marker(0,"ffdistal", msg->data/reduction_factor);
  update_mutex.unlock();
}
void callback_mf(const std_msgs::Float64ConstPtr& msg)
{
  update_mutex.lock();
  data[1] = msg->data;
  publish_marker(1,"mfdistal", msg->data/reduction_factor);
  update_mutex.unlock();
}
void callback_rf(const std_msgs::Float64ConstPtr& msg)
{
  update_mutex.lock();
  data[2] = msg->data;
  publish_marker(2,"rfdistal", msg->data/reduction_factor);
  update_mutex.unlock();
}
void callback_lf(const std_msgs::Float64ConstPtr& msg)
{
  update_mutex.lock();
  data[3] = msg->data;
  publish_marker(3,"lfdistal", msg->data/reduction_factor);
  update_mutex.unlock();
}
void callback_th(const std_msgs::Float64ConstPtr& msg)
{
  update_mutex.lock();
  data[4] = msg->data;
  publish_marker(4,"thdistal", msg->data/reduction_factor);
  update_mutex.unlock();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "sr_tactile_rviz_marker");
  ros::NodeHandle n;
  ros::Rate r(10);
  marker_pub = n.advertise<visualization_msgs::Marker>("sr_tactile_markers", 1);

  sub[0] = n.subscribe("/sr_tactile/touch/ff", 2,  callback_ff);
  sub[1] = n.subscribe("/sr_tactile/touch/mf", 2,  callback_mf);
  sub[2] = n.subscribe("/sr_tactile/touch/rf", 2,  callback_rf);
  sub[3] = n.subscribe("/sr_tactile/touch/lf", 2,  callback_lf);
  sub[4] = n.subscribe("/sr_tactile/touch/th", 2,  callback_th);

  std_msgs::Float64::_data_type cur_data[5] = {0};

  while (ros::ok())
  {
    update_mutex.lock();
    for (int i=0; i<5; i++)
      cur_data[i] = data[i];
    update_mutex.unlock();


    /*ROS_ERROR("TACTILE SENSOR READING: %f %f %f %f %f",
      cur_data[0],
      cur_data[1],
      cur_data[2],
      cur_data[3],
      cur_data[4]);*/
    r.sleep();
    ros::spinOnce();
  }
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
