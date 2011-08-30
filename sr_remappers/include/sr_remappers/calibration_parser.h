/**
* @file   calibration_parser.h
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Thu May 13 09:44:52 2010
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
* @brief This is where the calibration matrix is read from a file, stored and where the actual mapping take place.
*
*
*/

#ifndef   	CALIBRATION_PARSER_H_
# define   	CALIBRATION_PARSER_H_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

/**
* This is where the calibration matrix is read from a file, stored and where the actual mapping take place.
*/
class CalibrationParser {
public:

  /**
  * Default constructor, using the default path. Initialize a
  * calibration matrix loaded from the default calibration file.
  *
  */
  CalibrationParser();

  /**
  * Constructor initializing a calibration matrix loaded from the
  * given calibration file.
  *
  * @param path the path to the calibration file
  */
  CalibrationParser(std::string path);
  ~CalibrationParser(){};

  /**
  * multiplies the vector by the mapping matrix
  *
  * @param vector vector to be mapped
  *
  * @return mapped vector
  */
  std::vector<double> get_remapped_vector(std::vector<double>);

private:
  static const std::string default_path;

  /**
  * Open the given file and parses it into a matrix
  *
  * @param path path of the calibration file
  *
  * @return -1 if file not found, 0 if everything ok.
  */
  int init(std::string path);

  std::vector< std::vector<double> > calibration_matrix;

  inline double convertToDouble(std::string const& s)
  {
    std::istringstream i(s);
    double x;
    if (!(i >> x))
      ROS_ERROR("Bad calibration file: %s", s.c_str());
    return x;
  }
}; // end class


#endif 	    /* !CALIBRATION_PARSER_H_ */
