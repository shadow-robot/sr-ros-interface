/**
 * @file   motor_info_publisher.cpp
 * @author Nicolas Goossaert <nicolas@shadowrobot.com>
 * @date   Thu May 20 14:26:18 2010
 * 
 * @brief  This class reads and publishes data concerning the
 * different motors of the shadowhand. To publish those data, 
 * just call the publish() function. 
 * 
 * 
 */


//ROS include

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

//generic C/C++ include
#include <vector>
#include <string>
#include <sstream>

//header include
#include "sr_display/motor_info_publisher.h"
#include "sr_display/motor_data.h"

using namespace ros;
using namespace std;

namespace motor_info_publisher{

const int MotorInfoPublisher::number_hand_joints=24;

MotorInfoPublisher::MotorInfoPublisher() : n_tilde("~"), publish_rate(0.0){

	//set publish frequency
	double publish_freq;
	n_tilde.param("publish_frequency", publish_freq, 1.0);
	publish_rate = Rate(publish_freq);

	joints_names.resize(number_hand_joints);
	init_names();
	string prefix;
	string searched_param;
	string default_prefix="srh/";
	n_tilde.searchParam("joint_states_prefix", searched_param);
	n_tilde.param(searched_param, prefix, default_prefix);

	string full_topic = prefix + "shadowhand_data";

	shadowhand_data_sub = node.subscribe(full_topic, 10, &MotorInfoPublisher::jointsDataCallback, this);


	double text_x_position;
	n_tilde.param("text_x_position", text_x_position, -0.33);
	double text_y_position;
	n_tilde.param("text_y_position", text_y_position, -0.33);
	double text_z_position;
	n_tilde.param("text_z_position", text_z_position, 0.33);
	double text_scale;
	n_tilde.param("text_scale", text_scale, 0.2);
	double text_color_a;
	n_tilde.param("text_color_a", text_color_a, 0.7);

	datatest.clear();

	for ( int i(0); i<number_hand_joints; ++i){
		datatest.push_back(motor_data::MotorData(publish_freq));
	}

	motor_info_pub = node.advertise<visualization_msgs::Marker>("motor_info", 0);

}

MotorInfoPublisher::~MotorInfoPublisher(){
}

void MotorInfoPublisher::init_names(){

	joints_names[0]="ffdistal";
	joints_names[1]="ffmiddle";
	joints_names[2]="ffproximal";
	joints_names[3]="ffknuckle";
	joints_names[4]="mfdistal";
	joints_names[5]="mfmiddle";
	joints_names[6]="mfproximal";
	joints_names[7]="mfknuckle";
	joints_names[8]="rfdistal";
	joints_names[9]="rfmiddle";
	joints_names[10]="rfproximal";
	joints_names[11]="rfknuckle";
	joints_names[12]="lfdistal";
	joints_names[13]="lfmiddle";
	joints_names[14]="lfproximal";
	joints_names[15]="lfknuckle";
	joints_names[16]="lfmetacarpal";
	joints_names[17]="thdistal";
	joints_names[18]="thmiddle";
	joints_names[19]="thhub";
	joints_names[20]="thproximal";
	joints_names[21]="thbase";
	joints_names[22]="palm";
	joints_names[23]="wrist";
}



void MotorInfoPublisher::publish(){

	//ROS_INFO("publish on /motor_info");

	for ( int i(0); i<number_hand_joints; ++i){
		datatest[i].output_data.clear();
		datatest[i].return_markers();
	}

	//publish the whole vector

	for( unsigned int i = 0; i<datatest.size();++i){
		for( unsigned int j=0; j<datatest[i].output_data.size();++j){
			motor_info_pub.publish(datatest[i].output_data[j]);
		}
	}

	//	for( unsigned int i = 0; i<datatest.size();++i){
	//		motor_info_pub.publish(datatest[i].output_data[0]);
	//		motor_info_pub.publish(datatest[i].output_data[1]);
	//		motor_info_pub.publish(datatest[i].output_data[2]);
	//		motor_info_pub.publish(datatest[i].output_data[3]);
	//	}

	publish_rate.sleep();

}

void MotorInfoPublisher::jointsDataCallback(const sr_hand::joints_dataConstPtr& msg) {

	//ROS_ERROR("received");

	//joints: array containing all the joints of the hand
	int size = msg->joints_list_length;
	sr_hand::joint joints[size];

	//here the array is filled with the useful data (ie all data except FFJ0, MFJ0...)
	int cpt=0;
	for (int i(0); i<size; ++i){
		if(msg->joints_list[i].joint_name.at(msg->joints_list[i].joint_name.length()-1)!='0'){
			joints[i]=msg->joints_list[i];
			cpt++;
		}
	}
	size=cpt;

	//here the data are transformed
	for( int i(0); i<size; ++i){

		motor_data::MotorDataStruct data;

		data.motor_name = joints_names[i];
		data.motor_display = 1;
		data.error_flag = joints[i].error_flag;

		if(data.motor_name.find("palm")!=string::npos){
			data.motor_type=0;
		}
		else{
			if((data.motor_name.find("knuckle")!=string::npos)||(data.motor_name.find("hub")!=string::npos)||(data.motor_name.find("base")!=string::npos)||(data.motor_name.find("wrist")!=string::npos)){
				data.motor_type=1;
			}
			else{
				if(data.motor_name.find("proximal")!=string::npos){
					data.motor_type=2;
				}
				else{
					data.motor_type=3;
				}
			}
		}

		motor_data::MotorAttribute position;
		position.attr_name="position";
		position.attr_value=joints[i].joint_position;
		position.attr_display=0;
		position.attr_type=motor_data::POSITION;
		data.motor_attributes.push_back(position);

		motor_data::MotorAttribute target;
		target.attr_name="target";
		target.attr_value=joints[i].joint_target;
		target.attr_display=0;
		target.attr_type=motor_data::TARGET;
		data.motor_attributes.push_back(target);

		motor_data::MotorAttribute torque;
		torque.attr_name="force";
		torque.attr_value=joints[i].joint_torque;
		torque.attr_display=1;
		torque.attr_type=motor_data::FORCE;
		data.motor_attributes.push_back(torque);

		motor_data::MotorAttribute temperature;
		temperature.attr_name="temperature";
		temperature.attr_value=joints[i].joint_temperature;
		temperature.attr_display=1;
		temperature.attr_type=motor_data::TEMPERATURE;
		data.motor_attributes.push_back(temperature);

		motor_data::MotorAttribute current;
		current.attr_name="current";
		current.attr_value=joints[i].joint_current;
		current.attr_display=1;
		current.attr_type=motor_data::CURRENT;
		data.motor_attributes.push_back(current);

		datatest[i].setInputData(data);

	}

}

} // end namespace
