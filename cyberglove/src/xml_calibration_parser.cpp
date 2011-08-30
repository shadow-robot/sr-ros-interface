/**
 * @file   xml_calibration_parser.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Apr 27 11:30:41 2010
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief  This is a simple xml parser, used to parse the calibration
 * file for the cyberglove.
 * A Calibration file must have this format:
 * <Cyberglove_calibration>
 *   <Joint name="G_ThumbRotate">
 *     <calib raw_value="0.0" calibrated_value="-60"/>
 *     <calib raw_value="0.42" calibrated_value="100"/>
 *   </Joint>
 * </Cyberglove_calibration>
 *
 * The calibration will be used by the glove node to stream coherent
 * angles (and not an uncalibrated sensor value between 0 and 1).
 *
 *
 */

//ROS include
#include <ros/ros.h>

#include "cyberglove/xml_calibration_parser.h"

#include <stdio.h>

namespace xml_calibration_parser{

  const float XmlCalibrationParser::lookup_precision = 1000.0f;
  const float XmlCalibrationParser::lookup_offset = 1.0f;

  /**
   * The constructor: parses the given file and stores the calibration
   * in the vector std::vector<JointCalibration> jointsCalibrations.
   *
   * @param path_to_calibration the path to the xml calibration
   * file. Please note that it is best to use ros parameters to set
   * the path in your code calling this constructor.
   */
  XmlCalibrationParser::XmlCalibrationParser(std::string path_to_calibration)
  {
    TiXmlDocument doc(path_to_calibration.c_str());
    bool loadOkay = doc.LoadFile();
    if (loadOkay)
      {
	ROS_DEBUG("loading calibration %s", path_to_calibration.c_str());
	parse_calibration_file( doc.RootElement() );

	build_calibration_table();
      }
    else
      {
	ROS_ERROR("Failed to load file \"%s\"", path_to_calibration.c_str());
      }
  }

  /**
   * Parses the calibration file and retreive the full calibration for
   * the cyberglove.
   *
   * @param pParent The parent node (Cyberglove_calibration)
   * containing all the xml tree.
   */
  void XmlCalibrationParser::parse_calibration_file( TiXmlNode* pParent )
  {
    if ( !pParent ) return;

    TiXmlElement* child = pParent->FirstChildElement("Joint");

    //no Joint elements => error
    if( !child )
      {
	ROS_ERROR( "The calibration file seems to be broken: there's no Joint elements." );
	return;
      }

    bool has_sibling = true;
    while( has_sibling )
      {
	// a Joint element was found
	XmlCalibrationParser::JointCalibration joint_calib;
	joint_calib.name = child->Attribute("name");

	joint_calib.calibrations = parse_joint_attributes(child);

	jointsCalibrations.push_back(joint_calib);

	//get the next Joint element
	child = child->NextSiblingElement("Joint");
	//no more Joint elements => stop
	if( !child )
	    has_sibling = false;
      }
  }

  /**
   * Parses the Joint element of the calibration file.
   *
   * @param pParent a Joint element
   *
   * @return the calibration values for this Joint.
   */
  std::vector<XmlCalibrationParser::Calibration>
  XmlCalibrationParser::parse_joint_attributes( TiXmlNode* pParent )
  {
    std::vector<XmlCalibrationParser::Calibration> calibrations;

    TiXmlElement* child = pParent->FirstChildElement("calib");

    //no Joint elements => error
    if( !child )
      {
	ROS_ERROR( "The calibration file seems to be broken: there's no calibration elements." );
	return calibrations;
      }
    bool has_sibling = true;

    while( has_sibling )
      {
	// a Joint element was found
	XmlCalibrationParser::Calibration calib;
	float fval;

	// get the raw-value
	if( child->QueryFloatAttribute("raw_value", &fval) == TIXML_SUCCESS )
	  calib.raw_value = fval;
	else
	  ROS_ERROR("The calibration file seems to be broken: there's no raw_value attribute.");

	//get the calibrated-value
	if( child->QueryFloatAttribute("calibrated_value", &fval) == TIXML_SUCCESS )
	  calib.calibrated_value = fval;
	else
	  ROS_ERROR("The calibration file seems to be broken: there's no calibrated_value attribute.");

	//add the calibration to the vector
	calibrations.push_back( calib );

	//get the next Joint element
	child = child->NextSiblingElement("calib");
	//no more Joint elements => stop
	if( !child )
	    has_sibling = false;
      }

    return calibrations;
  }


  /**
   * Transform the calibration values to a lookup table for fast
   * processing of the calibration process.
   * NB: the lookup table ranges from 0 to +lookup_offset
   * with a precision of 1/lookup_precision.
   *
   */
  int XmlCalibrationParser::build_calibration_table()
  {
    for (unsigned int index_calib = 0; index_calib < jointsCalibrations.size(); ++index_calib)
    {
      std::string name = jointsCalibrations[index_calib].name;
      std::cout << name << std::endl;

      std::vector<Calibration> calib = jointsCalibrations[index_calib].calibrations;

      std::vector<float> lookup_table((int)lookup_offset*(int)lookup_precision);

      if( calib.size() < 2 )
	ROS_ERROR("Not enough points were defined to set up the calibration.");

      //order the calibration vector by ascending values of raw_value
      //      ROS_ERROR("TODO: calibration vector not ordered yet");

      std::cout << "lookup table : ";

      //setup the lookup table
      for( unsigned int index_lookup = 0;
	   index_lookup < lookup_table.size() ;
	   ++ index_lookup )
	{
	  float value = compute_lookup_value(index_lookup, calib);
	  std::cout << index_lookup<<":"<<value << " ";
	  lookup_table[index_lookup] = value;
	}

      std::cout << std::endl;

      //add the values to the map
      joints_calibrations_map[name] = lookup_table;
      //joints_calibrations_map.insert(std::pair <std::string, std::vector<float> >(name, lookup_table));
    }

    return 0;
  }

  /**
   * return the value to store in the lookup table for a given index,
   * using the calibration information.
   *
   * @param index the index for which we compute the value
   *
   * @param calib the vector containing the calibration informations
   * (a list of raw_value <=> calibrated_value)
   *
   * @return the value to be stored in the lookup table
   */
  float XmlCalibrationParser::compute_lookup_value(int index, std::vector<XmlCalibrationParser::Calibration> calib)
  {
    float raw_pos = return_raw_position_from_index(index);

    if(calib.size() == 2)
      return linear_interpolate( raw_pos,
				 calib[0].raw_value,
				 calib[0].calibrated_value,
				 calib[1].raw_value,
				 calib[1].calibrated_value
			     );


    for( unsigned int index_calib = 0; index_calib < calib.size() - 1;
	 ++index_calib)
      {
	if(calib[index_calib].raw_value > raw_pos)
	  {
	    return linear_interpolate( raw_pos,
				       calib[index_calib].raw_value,
				       calib[index_calib].calibrated_value,
				       calib[index_calib+1].raw_value,
				       calib[index_calib+1].calibrated_value
				     );
	  }
      }

    //bigger than last calibrated value => extrapolate the value from
    //last 2 values
    //TODO: ca marche la formule si on est en dehors des points ? oui
    return linear_interpolate( raw_pos,
			       calib[calib.size()-1].raw_value,
			       calib[calib.size()-1].calibrated_value,
			       calib[calib.size()].raw_value,
			       calib[calib.size()].calibrated_value
			     );

  }

  float XmlCalibrationParser::get_calibration_value(float position, std::string joint_name)
  {
    mapType::iterator iter = joints_calibrations_map.find(joint_name);

    if( iter != joints_calibrations_map.end() )
      {
	//reads from the lookup table
	int index = return_index_from_raw_position(position);
	std::cout << index << std::endl;
	return iter->second[index];
      }
    else
      {
	ROS_ERROR("%s is not calibrated", joint_name.c_str());
	return 1.0f;
      }
  }

  float XmlCalibrationParser::linear_interpolate( float x ,
						  float x0, float y0,
						  float x1, float y1 )
  {

    //TODO ca marche qd c'est decroissant ca? oui
    float y = 0.0f;
    if( x1 - x0 == 0.0f )
      {
	ROS_WARN("Bad calibration: raw_calib[1] = raw_calib[0]");
	return 0.0f;
      }

    y = y0 + (x-x0)* ((y1-y0)/(x1-x0));
    return y;
  }


  int XmlCalibrationParser::return_index_from_raw_position(float raw_position)
  {
    if (raw_position < 0.0f)
      return 0;
    if(raw_position > 1.0f)
      return lookup_precision;
    return round(raw_position * lookup_precision);
  };

  int XmlCalibrationParser::round(float number)
  {
    //we only have positive numbers
    return (int)floor(number + 0.5);
    //return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
  }


  std::vector<XmlCalibrationParser::JointCalibration> XmlCalibrationParser::getJointsCalibrations()
  {
    return jointsCalibrations;
  }
}//end namespace
