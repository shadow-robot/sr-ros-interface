#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ContactsState.h>
//#include <geometry_msgs/Point.h>

#include <string>

#include <boost/thread.hpp>

//messages
#include <std_msgs/Float64.h>

#define COUNTDOWN_MAX	5
#define FF	0
#define MF	1
#define RF	2
#define LF	3
#define TH	4

//a ros subscriber (will be instantiated later on)
ros::Subscriber sub[5];
ros::Publisher marker_pub;
std_msgs::Float64::_data_type data[5];
boost::mutex update_mutex;
int colors[8][3]={{0,0,0},{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1},{1,1,1}};
// Set our initial shape type to be a cube
uint32_t shape = visualization_msgs::Marker::ARROW;
int countdown[5] = {COUNTDOWN_MAX,COUNTDOWN_MAX,COUNTDOWN_MAX,COUNTDOWN_MAX,COUNTDOWN_MAX} ;

void publish_marker_special(unsigned int id, std::string framesuffix, const::geometry_msgs::Vector3& force, int col)
{
		visualization_msgs::Marker marker;
                // Set the frame ID and timestamp.  See the TF tutorials for information on these.
                marker.header.frame_id = "/"+framesuffix;
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
                mypoint.x= (force.x/20.0);
                mypoint.y= (phalanx_thickness+force.y/20.0);
                mypoint.z= phalanx_length+force.z/20.0;
                marker.points.push_back(mypoint);

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker.scale.x = 0.0025;
                marker.scale.y = 0.004;
                marker.scale.z = 0;

		// Set the color -- be sure to set alpha to something non-zero!
                marker.color.r = colors[col][0];
                marker.color.g = colors[col][1];
                marker.color.b = colors[col][2];
                marker.color.a = 1.0;

		marker.lifetime = ros::Duration(.5);

                // Publish the marker
		marker_pub.publish(marker);
}
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

void callback_ff(const gazebo_msgs::ContactsState& msg)
{
  // Publish the marker
  // once every countdown
  update_mutex.lock();
  countdown[FF]--;
  if(countdown[FF]<=0)
  {
        countdown[FF]=COUNTDOWN_MAX;
  	for(unsigned int i=0;i<msg.states[0].wrenches.size();i++)
 	{
		const::geometry_msgs::Vector3& v = msg.states[0].wrenches[i].force;
    		publish_marker_special(FF,"ffdistal", v,i);
  	}
  }
  update_mutex.unlock();
}

void callback_mf(const gazebo_msgs::ContactsState& msg)
{
  // Publish the marker
  // once every countdown
  update_mutex.lock();
  countdown[MF]--;
  if(countdown[MF]<=0)
  {
        countdown[MF]=COUNTDOWN_MAX;
        for(unsigned int i=0;i<msg.states[0].wrenches.size();i++)
        {
                const::geometry_msgs::Vector3& v = msg.states[0].wrenches[i].force;
                publish_marker_special(MF,"mfdistal", v,i);
        }
  }
  update_mutex.unlock();
}

void callback_rf(const gazebo_msgs::ContactsState& msg)
{
  // Publish the marker
  // once every countdown
  update_mutex.lock();
  countdown[RF]--;
  if(countdown[RF]<=0)
  {
        countdown[RF]=COUNTDOWN_MAX;
        for(unsigned int i=0;i<msg.states[0].wrenches.size();i++)
        {
                const::geometry_msgs::Vector3& v = msg.states[0].wrenches[i].force;
                publish_marker_special(RF,"rfdistal", v,i);
        }
  }
  update_mutex.unlock();
}

void callback_lf(const gazebo_msgs::ContactsState& msg)
{
  // Publish the marker
  // once every countdown
  update_mutex.lock();
  countdown[LF]--;
  if(countdown[LF]<=0)
  {
        countdown[LF]=COUNTDOWN_MAX;
        for(unsigned int i=0;i<msg.states[0].wrenches.size();i++)
        {
                const::geometry_msgs::Vector3& v = msg.states[0].wrenches[i].force;
                publish_marker_special(LF,"lfdistal", v,i);
        }
  }
  update_mutex.unlock();
}

void callback_th(const gazebo_msgs::ContactsState& msg)
{
  // Publish the marker
  // once every countdown
  update_mutex.lock();
  countdown[TH]--;
  if(countdown[TH]<=0)
  {
        countdown[TH]=COUNTDOWN_MAX;
        for(unsigned int i=0;i<msg.states[0].wrenches.size();i++)
        {
                const::geometry_msgs::Vector3& v = msg.states[0].wrenches[i].force;
                publish_marker_special(TH,"thdistal", v,i);
        }
  }
  update_mutex.unlock();
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "bumper_markers");
  ros::NodeHandle n;
  ros::Rate r(10);
  marker_pub = n.advertise<visualization_msgs::Marker>("bumper_markers", 1);

  sub[0] = n.subscribe("/ffdistal_bumper/state", 2,  callback_ff);
  sub[1] = n.subscribe("/mfdistal_bumper/state", 2,  callback_mf);
  sub[2] = n.subscribe("/rfdistal_bumper/state", 2,  callback_rf);
  sub[3] = n.subscribe("/lfdistal_bumper/state", 2,  callback_lf);
  sub[4] = n.subscribe("/thdistal_bumper/state", 2,  callback_th);

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

