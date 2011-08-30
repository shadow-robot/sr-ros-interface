/**
 * @file   xml_calibration_parser.h
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


#ifndef   	XML_CALIBRATION_PARSER_H_
# define   	XML_CALIBRATION_PARSER_H_

//xml parser library
#include <tinyxml/tinyxml.h>

//generic C/C++ include
#include <string>
#include <vector>
#include <map>

namespace xml_calibration_parser{

class XmlCalibrationParser
{
 public:
  XmlCalibrationParser(){};
  XmlCalibrationParser(std::string path_to_calibration);
  ~XmlCalibrationParser(){};

  float get_calibration_value(float position, std::string joint_name);
  
  struct Calibration
  {
    float raw_value;
    float calibrated_value;
  };

  struct JointCalibration
  {
    std::string name;
    std::vector<Calibration> calibrations;
  };
  
  
  std::vector<JointCalibration> getJointsCalibrations();

 protected:
  void parse_calibration_file( TiXmlNode* pParent );
  std::vector<Calibration> parse_joint_attributes( TiXmlNode* pParent );

  /// The vector containing the calibration
  std::vector<JointCalibration> jointsCalibrations;
  // use a map to easily access the value
  typedef std::map<std::string, std::vector<float> > mapType;
  mapType joints_calibrations_map;
  
  int build_calibration_table();

  std::vector<float> calibration_to_lookup_table(std::vector<Calibration> calib);

  float compute_lookup_value(int index, std::vector<Calibration> calib);

  float linear_interpolate( float x , 
			    float x0, float y0, 
			    float x1, float y1 );

  // consts for the lookup tables
  static const float lookup_precision;
  static const float lookup_offset;

  /** 
   * rounds the given number
   * 
   * @param number a float
   * 
   * @return the float rounded to the closest int
   */
  int round(float number);

  /** 
   * inline function to convert a raw position to a valid index for
   * our lookup table
   * 
   * @param raw_position the raw position (directly read from the glove)
   * 
   * @return the calibrated value
   */
  int return_index_from_raw_position(float raw_position);

  /** 
   * inline function to convert an index of our lookup table to a raw
   * position.
   * 
   * @param lookup_index the index in the lookup table
   * 
   * @return the corresponding raw position
   */
  static inline float return_raw_position_from_index(int lookup_index)
  {
    return ((float)lookup_index)/lookup_precision;
  };
  
}; // end class XmlCalibrationParser

} // end namespace
#endif 	    /* !XML_CALIBRATION_PARSER_H_ */
