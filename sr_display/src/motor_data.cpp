/**
 * @file   motor_data.cpp
 * @author Nicolas Goossaert <nicolas@shadowrobot.com>
 * @date   Fri May 21 11:51:38 2010
 * 
 * @brief  This class transforms the input data concerning for
 * instance the temperature and the force on a motor, into Markers
 * (text marker and line between the text and the motor) ready to 
 * be published
 * 
 * 
 */


//ROS include
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

//generic C/C++ include
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

//header include
#include <sr_display/motor_data.h>

using namespace ros;
using namespace std;

namespace motor_data{

//Constructor with param: to get a custom MotorData

MotorData::MotorData(TextParam text, MotorParam motor, double publish_freq){

	publish_frequency = publish_freq;

	text_param.text_x_position = text.text_x_position;
	text_param.text_y_position = text.text_y_position;
	text_param.text_z_position = text.text_z_position;
	text_param.text_scale = text.text_scale;
	text_param.default_r = 1.0;
	text_param.default_g = 0.0;
	text_param.default_b = 0.0;
	text_param.text_color_a = text.text_color_a;

	motor_param.min_current = motor.min_current;
	motor_param.max_current = motor.max_current;
	motor_param.min_temperature = motor.min_temperature;
	motor_param.max_temperature = motor.max_temperature;
	motor_param.min_force = motor.min_force;
	motor_param.max_force = motor.max_force;


	//here are the attributes we want to display for each motor
	input_data.motor_name="Test Motor";
	MotorAttribute temperature, current;
	temperature.attr_name="temperature";
	current.attr_name="current";
	temperature.attr_value=20;
	current.attr_value=0;
	temperature.attr_display=current.attr_display=1;
	temperature.attr_type=TEMPERATURE;
	current.attr_type=CURRENT;
	input_data.motor_attributes.push_back(temperature);
	input_data.motor_attributes.push_back(current);
	input_data.motor_display=1;

}

//Constructor with struct: useful to update the markers

MotorData::MotorData(MotorDataStruct data_struct, double publish_freq){

	publish_frequency = publish_freq;

	//default text settings
	text_param.text_x_position = 0;
	text_param.text_y_position = 0.025;
	text_param.text_z_position = 0.025;
	text_param.text_scale = 0.005;
	text_param.default_r = 1.0;
	text_param.default_g = 0.0;
	text_param.default_b = 0.0;
	text_param.text_color_a = 1;

	//default motor settings
	motor_param.min_current = 0;
	motor_param.max_current = 1;
	motor_param.min_temperature = 20;
	motor_param.max_temperature = 80;
	motor_param.min_force  = 0;
	motor_param.max_force = 1;

	input_data.motor_name = data_struct.motor_name;
	input_data.motor_display = data_struct.motor_display;
	input_data.error_flag = data_struct.error_flag;
	for( unsigned int i(0); i<data_struct.motor_attributes.size(); ++i){
		MotorAttribute attribute;
		attribute.attr_name = data_struct.motor_attributes[i].attr_name;
		attribute.attr_value = data_struct.motor_attributes[i].attr_value;
		attribute.attr_display = data_struct.motor_attributes[i].attr_display;
		attribute.attr_type = data_struct.motor_attributes[i].attr_type;
		input_data.motor_attributes.push_back(attribute);
	}

}

//Default constructor: default settings

MotorData::MotorData(double publish_freq){

	publish_frequency = publish_freq;

	//default text settings
	text_param.text_x_position = 0;
	text_param.text_y_position = 0.025;
	text_param.text_z_position = 0.025;
	text_param.text_scale = 0.005;
	text_param.default_r = 1.0;
	text_param.default_g = 0.0;
	text_param.default_b = 0.0;
	text_param.text_color_a = 1;

	//default motor settings
	motor_param.min_current = 0;
	motor_param.max_current = 1;
	motor_param.min_temperature = 0;
	motor_param.max_temperature = 1;
	motor_param.min_force  = 0;
	motor_param.max_force = 1;

	//Test motor manual entry. To replace by the filling of the vector

	input_data.motor_name="palm";
	MotorAttribute temperature, current;
	temperature.attr_name="temperature";
	current.attr_name="current";
	temperature.attr_value=0;
	current.attr_value=1;
	temperature.attr_display=current.attr_display=0;
	temperature.attr_type=TEMPERATURE;
	current.attr_type=CURRENT;
	input_data.motor_attributes.push_back(temperature);
	input_data.motor_attributes.push_back(current);
	input_data.motor_display=0;

}

MotorDataStruct MotorData::getInputData(){
	return input_data;
}

void MotorData::setInputData(MotorDataStruct new_input_data){
	input_data.motor_name = new_input_data.motor_name;
	input_data.motor_display = new_input_data.motor_display;
	input_data.error_flag = new_input_data.error_flag;
	input_data.motor_attributes.clear();
	for (unsigned int i(0); i<new_input_data.motor_attributes.size(); ++i){
		motor_data::MotorAttribute ma;
		ma.attr_name = new_input_data.motor_attributes[i].attr_name;
		ma.attr_value = new_input_data.motor_attributes[i].attr_value;
		ma.attr_display = new_input_data.motor_attributes[i].attr_display;
		ma.attr_type = new_input_data.motor_attributes[i].attr_type;
		input_data.motor_attributes.push_back(ma);
	}
}

int MotorData::findAttributeFromName(MotorDataStruct motor, string s){
	int answer = 42;
	for(unsigned int i(0); i<motor.motor_attributes.size(); ++i){
		if(strcmp(motor.motor_attributes[i].attr_name.c_str(), s.c_str())){
			answer = i;
		}
	}
	return answer;
}

void MotorData::return_markers(){

	//Link between the joint and its info marker: its name and the graphic line bonding the name and the joint
	visualization_msgs::Marker motor_name, points, line;
	string link = "/srh/position/"+input_data.motor_name;
	motor_name.header.frame_id = points.header.frame_id = line.header.frame_id = link;
	motor_name.ns = points.ns = line.ns = input_data.motor_name;
	points.id = 0;
	line.id = 1;
	motor_name.id = 2;
	motor_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	points.type = visualization_msgs::Marker::POINTS;
	line.type = visualization_msgs::Marker::LINE_STRIP;
	motor_name.header.stamp = points.header.stamp = line.header.stamp = ros::Time::now();
	motor_name.action = points.action = line.action = visualization_msgs::Marker::ADD;
	geometry_msgs::Point p1;
	geometry_msgs::Point p2;
	motor_name.pose.orientation.x = points.pose.orientation.x = line.pose.orientation.x = 0.0;
	motor_name.pose.orientation.y = points.pose.orientation.y = line.pose.orientation.y = 0.0;
	motor_name.pose.orientation.z = points.pose.orientation.z = line.pose.orientation.z = 0.0;
	motor_name.pose.orientation.w = points.pose.orientation.w = line.pose.orientation.w = 1.0;
	line.scale.x = 0.0003;
	points.scale.x = 0.0003;
	points.scale.y = 0.0003;

	motor_name.pose.position.x = text_param.text_x_position;
	motor_name.pose.position.y = text_param.text_y_position;
	motor_name.pose.position.z = text_param.text_z_position;
	motor_name.scale.y = motor_name.scale.z = motor_name.scale.x = text_param.text_scale;

	motor_name.color.r = points.color.r = line.color.r = text_param.default_r;
	motor_name.color.g = points.color.g = line.color.g = text_param.default_g;
	motor_name.color.b = points.color.b = line.color.b = text_param.default_b;

	motor_name.color.a = points.color.a = line.color.a = text_param.text_color_a;


	//p1 and p2 are necessary to draw the line: they are the line's extremities
	p1.x = 0.0;
	p1.y = 0.0;
	p1.z = 0.0;
	p2.x = motor_name.pose.position.x;
	p2.y = motor_name.pose.position.y;
	p2.z = motor_name.pose.position.z;
	motor_name.text = input_data.motor_name;
	motor_name.lifetime = ros::Duration(1.1*publish_frequency);
	line.lifetime = ros::Duration(1.1*publish_frequency);
	points.points.push_back(p1);
	points.points.push_back(p2);
	line.points.push_back(p1);
	line.points.push_back(p2);

	output_data.push_back(line);
	output_data.push_back(motor_name);

	//filling of the vector<Markers> with the attributes of the Motor

	for( unsigned int i(0);i<input_data.motor_attributes.size();++i){
		visualization_msgs::Marker attribute;
		attribute.header.frame_id = motor_name.header.frame_id;
		attribute.ns=input_data.motor_name;
		attribute.id = output_data.back().id+1;
		attribute.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		attribute.header.stamp = ros::Time::now();
		attribute.action = visualization_msgs::Marker::ADD;
		attribute.pose.orientation.x = 0.0;
		attribute.pose.orientation.y = 0.0;
		attribute.pose.orientation.z = 0.0;
		attribute.pose.orientation.w = 1.0;
		attribute.scale.x = attribute.scale.y = attribute.scale.z = (motor_name.scale.z/1.5);
		attribute.pose.position.x = output_data.back().pose.position.x;
		attribute.pose.position.y = output_data.back().pose.position.y;
		attribute.pose.position.z = output_data.back().pose.position.z-attribute.scale.z;

		/*
		 * the following is the way to have color changing with the ratio between
		 * the current value of the attribute and its maximum value.
		 * Efficient, but quite messy, and needs to be modified each time we want to add a new
		 * attribute. If possible, it would be great to simplify and improve this part
		 */


		if(input_data.motor_attributes[i].attr_type==TEMPERATURE){

			if(input_data.motor_attributes[i].attr_value < motor_param.min_temperature+((motor_param.max_temperature-motor_param.min_temperature)/2)){
				attribute.color.r = 1+(input_data.motor_attributes[i].attr_value-(motor_param.min_temperature+((motor_param.max_temperature-motor_param.min_temperature)/2)))/(motor_param.max_temperature-(motor_param.min_temperature+((motor_param.max_temperature-motor_param.min_temperature)/2)));
			}
			else{
				attribute.color.r = 1;
			}
			if(input_data.motor_attributes[i].attr_value<motor_param.min_temperature+((motor_param.max_temperature-motor_param.min_temperature)/2)){
				attribute.color.g = 1;
			}
			else{
				attribute.color.g = 1+((motor_param.min_temperature+((motor_param.max_temperature-motor_param.min_temperature)/2))-input_data.motor_attributes[i].attr_value)/(motor_param.max_temperature-(motor_param.min_temperature+((motor_param.max_temperature-motor_param.min_temperature)/2)));
			}

		}

		else {

			if(input_data.motor_attributes[i].attr_type==CURRENT){

				if(input_data.motor_attributes[i].attr_value < motor_param.min_current+((motor_param.max_current-motor_param.min_current)/2)){
					attribute.color.r = 1+(input_data.motor_attributes[i].attr_value-(motor_param.min_current+((motor_param.max_current-motor_param.min_current)/2)))/(motor_param.max_current-(motor_param.min_current+((motor_param.max_current-motor_param.min_current)/2)));
				}
				else{
					attribute.color.r = 1;
				}
				if(input_data.motor_attributes[i].attr_value<motor_param.min_current+((motor_param.max_current-motor_param.min_current)/2)){
					attribute.color.g = 1;
				}
				else{
					attribute.color.g = 1+((motor_param.min_current+((motor_param.max_current-motor_param.min_current)/2))-input_data.motor_attributes[i].attr_value)/(motor_param.max_current-(motor_param.min_current+((motor_param.max_current-motor_param.min_current)/2)));
				}
			}

			else {

				if(input_data.motor_attributes[i].attr_type==FORCE){

					if(input_data.motor_attributes[i].attr_value < motor_param.min_force+((motor_param.max_force-motor_param.min_force)/2)){
						attribute.color.r = 1+(input_data.motor_attributes[i].attr_value-(motor_param.min_force+((motor_param.max_force-motor_param.min_force)/2)))/(motor_param.max_force-(motor_param.min_force+((motor_param.max_force-motor_param.min_force)/2)));
					}
					else{
						attribute.color.r = 1;
					}
					if(input_data.motor_attributes[i].attr_value<motor_param.min_force+((motor_param.max_force-motor_param.min_force)/2)){
						attribute.color.g = 1;
					}
					else{
						attribute.color.g = 1+((motor_param.min_force+((motor_param.max_force-motor_param.min_force)/2))-input_data.motor_attributes[i].attr_value)/(motor_param.max_force-(motor_param.min_force+((motor_param.max_force-motor_param.min_force)/2)));
					}
				}

				else {
					attribute.color.r=0;
					attribute.color.g=0;
					attribute.color.b=1;
				}
			}
		}

		attribute.color.b = 0;
		attribute.color.a = 1;
		stringstream s;
		s << input_data.motor_attributes[i].attr_name;
		s << " : ";
		s << input_data.motor_attributes[i].attr_value;
		string texte = s.str();
		attribute.text = texte;
		attribute.lifetime = ros::Duration(1.1*publish_frequency);

		/*
		 * Update of the color of the motor's name.
		 * The name and the line have the color of the attribute which is
		 * the closest from its maximum value.
		 * To change each time we add a new attribute.
		 */

		double force_ratio = input_data.motor_attributes[2].attr_value/(motor_param.max_force-motor_param.min_force);
		double temperature_ratio = input_data.motor_attributes[3].attr_value/(motor_param.max_temperature-motor_param.min_temperature);
		double current_ratio = input_data.motor_attributes[4].attr_value/(motor_param.max_current-motor_param.min_current);

		if(input_data.motor_attributes[i].attr_type==FORCE){
			if(force_ratio>=temperature_ratio&&force_ratio>=current_ratio){
				text_param.default_r=attribute.color.r;
				text_param.default_g=attribute.color.g;
				text_param.default_b=attribute.color.b;
			}
		}
		else{
			if(input_data.motor_attributes[i].attr_type==TEMPERATURE){
				if(temperature_ratio>=force_ratio&&temperature_ratio>=current_ratio){
					text_param.default_r=attribute.color.r;
					text_param.default_g=attribute.color.g;
					text_param.default_b=attribute.color.b;
				}
			}
			else{
				if(input_data.motor_attributes[i].attr_type==CURRENT){
					if(current_ratio>=temperature_ratio&&current_ratio>=force_ratio){
						text_param.default_r=attribute.color.r;
						text_param.default_g=attribute.color.g;
						text_param.default_b=attribute.color.b;
					}
				}
				else{
					text_param.default_r=0;
					text_param.default_g=0;
					text_param.default_b=1;
				}
			}
		}

		if(input_data.motor_attributes[i].attr_display==1){
			output_data.push_back(attribute);
		}
	}
	bool displays = false;
	for(unsigned int i(0); i<input_data.motor_attributes.size(); ++i){
		if(input_data.motor_attributes[i].attr_display==true){
			displays=true;
		}
	}
	if(displays==false || input_data.motor_display==0){
		output_data.pop_back();
		output_data.pop_back();
	}
}

}
