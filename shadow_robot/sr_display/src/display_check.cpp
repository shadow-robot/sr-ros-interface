/*
 * @file   display_check.cpp
 * @author Nicolas Goossaert <nicolas@shadowrobot.com>
 * @date   16 juin 2010
 *
 * @brief  This class is a controller for the "display"
 * attribute of each information that is possible to
 * display. It allows the user to see the informations
 * he wants to see and hide the other ones.
 *
 *
 */


//ROS include
#include <ros/ros.h>

//generic C/C++ include
#include <vector>
#include <string>
#include <sstream>

//header include
#include <sr_display/display_check.h>
#include <sr_display/motor_data.h>

using namespace ros;
using namespace std;

namespace display_check {

DisplayCheck::DisplayCheck(motor_info_publisher::MotorInfoPublisher* pub){
	mip = pub;

	service_true = node.advertiseService("set_display", &DisplayCheck::setDisplay, this);

} 
DisplayCheck::~DisplayCheck()
{
}


bool DisplayCheck::setDisplay(sr_display::display_check::Request  &req, sr_display::display_check::Response  &res){
	int joint = 0;
	for(unsigned int i(0); i<mip->joints_names.size(); ++i){
		if(strcmp(req.joint_name.c_str(), mip->joints_names[i].c_str())){
			joint = i;
		}
	}
	motor_data::MotorDataStruct data = mip->datatest[joint].getInputData();
	string attribute_name = req.attr_name.c_str();
	if(strcmp(req.joint_name.c_str(), "all")){
		if(req.display==0){
			data.motor_display = 0;
			for(unsigned int i(0); i<data.motor_attributes.size(); ++i){
				data.motor_attributes[i].attr_display=0;
			}
		}
		else{
			if(req.display==1){
				data.motor_display = 1;
				for(unsigned int i(0); i<data.motor_attributes.size(); ++i){
					data.motor_attributes[i].attr_display=1;
				}
			}
		}
	}
	else {
		if(req.display==0){
			data.motor_attributes[mip->datatest[joint].findAttributeFromName(data, attribute_name)].attr_display = 0;
		}
		else{
			if(req.display==1){
				data.motor_attributes[mip->datatest[joint].findAttributeFromName(data, attribute_name)].attr_display = 1;
			}
			else{
				ROS_ERROR("argh");
			}
		}
	}
	mip->datatest[joint].setInputData(data);
	res.success = 1;
	ROS_ERROR("toto");
	return true;
}

} //end namespace display_check
