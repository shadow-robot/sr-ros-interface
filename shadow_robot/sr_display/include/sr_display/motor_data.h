/**
 * @file   motor_data.h
 * @author Nicolas Goossaert <nicolas@shadowrobot.com>
 * @date   Fri May 21 10:59:41 2010
 * 
 * @brief  header of the motor_data.cpp file
 * This header contains the "MotorAttribute" and the
 * "MotorDataStruct" structures
 * 
 * 
 */

#ifndef MOTOR_DATA_H_
#define MOTOR_DATA_H_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <visualization_msgs/Marker.h>


using namespace ros;
using namespace std;

namespace motor_data{

enum DataTypes { TEMPERATURE, CURRENT, FORCE, POSITION, TARGET, NOTYPE };


/// A structure that defines an attribute of the motor (temperature, current...)
struct MotorAttribute {

	string attr_name;

	double attr_value;

	short attr_display;

	DataTypes attr_type;

}; //end struct MotorAttribute

/// A structure that defines a motor, with its name and all its attributes.
struct MotorDataStruct {

	string motor_name;

	vector<MotorAttribute> motor_attributes;

	short motor_display;

	//a bit dirty
	string error_flag;

}; //end struct MotorDataStruct



/// General text settings, if a custom text is needed
struct TextParam {
	double text_x_position;
	double text_y_position;
	double text_z_position;
	double text_scale;
	float default_r;
	float default_g;
	float default_b;
	float text_color_a;
};

/// General motor settings
struct MotorParam {
	double min_current;
	double max_current;
	double min_temperature;
	double max_temperature;
	double min_force;
	double max_force;
};

class MotorData
{
public:
	/**Precise Constructor
	 * @param textparam the list of parameters concerning the Text position, size and color
	 * @param motorparam the list of parameters concerning the Motor settings, particularly the minimum and maximum value of each measured value
	 * @param publish_freq markers publish frequency
	 */
	MotorData(TextParam textparam, MotorParam motorparam, double publish_freq);

	/**Update Constructor
	 * This constructor is used each time we want to update the data to display
	 * @param motor all the data about the motor
	 * @param publish_freq markers publish frequency
	 */
	MotorData(MotorDataStruct motor, double publish_freq);

	/**Default Constructor
	 * @param publish_freq markers publish frequency
	 */
	MotorData(double publish_freq);

	/**Destructor
	 *
	 */
	~MotorData(){};

	/**Updates the data to display
	 * @param newdata the up to date list of data to display
	 */
	void setInputData(MotorDataStruct newdata);

	/**Finds an attribute in the data structure by its name, and returns its index
	 * @param motor the data structure about the concerned motor
	 * @param attr_name the name of the attribute looked for
	 * @return the index of the attribute in the vector containing all the attributes of the motor
	 */
	int findAttributeFromName(MotorDataStruct motor, string attr_name);

	/**Returns the data structure containing all the informations about the motor
	 * @return the data structure
	 */
	MotorDataStruct getInputData();

	/**Transforms the data structure into a series of vizualisation_msgs::Marker and stores them in a vector
	 *
	 */
	void return_markers();

	/// The vector where are stored all the markers to be displayed
	vector<visualization_msgs::Marker> output_data;

private:

	/// The data structure containing all the informations about the motor
	MotorDataStruct input_data;

	/// General text settings
	TextParam text_param;

	/// General motor settings
	MotorParam motor_param;

	///markers publish frequency
	double publish_frequency;

}; // end class MotorInfoPublisher

} // end namespace

#endif
