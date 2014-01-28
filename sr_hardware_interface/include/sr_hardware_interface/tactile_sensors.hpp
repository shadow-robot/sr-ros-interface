/**
 * @file   tactile_sensors.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Oct  5 14:57:27 2011
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
*
 * @brief  Contains the different tactile sensors structures.
 *
 *
 */

#ifndef _TACTILE_SENSORS_HPP_
#define _TACTILE_SENSORS_HPP_

#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include <sstream>

#include <ros/ros.h>

namespace tactiles
{
  class GenericTactileData
  {
  public:
    GenericTactileData() {};

    GenericTactileData(bool tactile_data_valid, int sample_frequency,
                       std::string manufacturer, std::string serial_number,
                       int software_version_current, int software_version_server,
                       bool software_version_modified, std::string pcb_version)
      : tactile_data_valid(tactile_data_valid), sample_frequency(sample_frequency),
        manufacturer(manufacturer), serial_number(serial_number),
        software_version_current(software_version_current),
        software_version_server(software_version_server),
        software_version_modified(software_version_modified),
        pcb_version(pcb_version)
    {};

    virtual ~GenericTactileData() {};

    bool tactile_data_valid;

    int which_sensor;
    int sample_frequency;
    std::string manufacturer;
    std::string serial_number;

    int software_version_current;
    int software_version_server;
    bool software_version_modified;

    /**
     * Parses the version string received
     *  from the tactiles and fill in the
     *  variables.
     *
     * @version The raw version string.
     */
    void set_software_version( std::string version )
    {
      //split the string to fill the different versions
      std::vector<std::string> splitted_string;
      boost::split(splitted_string, version, boost::is_any_of("\n"));

      ROS_DEBUG("Tactile version: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", static_cast<unsigned char>(version[0]), static_cast<unsigned char>(version[1]), static_cast<unsigned char>(version[2]), static_cast<unsigned char>(version[3]), static_cast<unsigned char>(version[4]), static_cast<unsigned char>(version[5]), static_cast<unsigned char>(version[6]), static_cast<unsigned char>(version[7]), static_cast<unsigned char>(version[8]), static_cast<unsigned char>(version[9]), static_cast<unsigned char>(version[10]), static_cast<unsigned char>(version[11]), static_cast<unsigned char>(version[12]), static_cast<unsigned char>(version[13]), static_cast<unsigned char>(version[14]), static_cast<unsigned char>(version[15]));
      if (splitted_string.size() >= 3)
      {
          software_version_current = convertToInt(splitted_string[0]);
          software_version_server = convertToInt(splitted_string[1]);

          if( splitted_string[2] == "No")
            software_version_modified = false;
          else
            software_version_modified = true;
      }
      else
      {
          ROS_ERROR("Incorrect tactile sensor version format");
          software_version_current = 0;
          software_version_server = 0;
          software_version_modified = false;
      }
    };

    /**
     * Formats the software version for the
     *  diagnostics.
     *
     * @return the formatted string
     */
    virtual std::string get_software_version()
    {
      //concatenate versions in a string.
      std::string full_version;

      std::stringstream ss;
      if( software_version_modified )
        ss << "current: " << software_version_current << " / server: " << software_version_server << " / MODIFIED";
      else
        ss << "current: " << software_version_current << " / server: " << software_version_server << " / not modified";

      full_version = ss.str();

      return full_version;
    };

    std::string pcb_version;

    inline double convertToInt(std::string const& s)
    {
      std::istringstream i(s);
      int x;
      if (!(i >> x))
        x = -1;
      return x;
    }
  };

  class PST3Data
    : public GenericTactileData
  {
  public:
    PST3Data()
      : GenericTactileData()
    {};

    PST3Data(const PST3Data& pst3)
      : GenericTactileData(pst3.tactile_data_valid, pst3.sample_frequency,
                           pst3.manufacturer, pst3.serial_number,
                           pst3.software_version_current,
                           pst3.software_version_server,
                           pst3.software_version_modified,
                           pst3.pcb_version),
        pressure(pst3.pressure), temperature(pst3.temperature),
        debug_1(pst3.debug_1), debug_2(pst3.debug_2),
        pressure_raw(pst3.pressure_raw), zero_tracking(pst3.zero_tracking), dac_value(pst3.dac_value)
    {};


    PST3Data(const GenericTactileData& gtd)
      : GenericTactileData(gtd.tactile_data_valid, gtd.sample_frequency,
                           gtd.manufacturer, gtd.serial_number,
                           gtd.software_version_current,
                           gtd.software_version_server,
                           gtd.software_version_modified,
                           gtd.pcb_version)
    {};

    ~PST3Data() {};
    int pressure;
    int temperature;

    int debug_1;
    int debug_2;

    int pressure_raw;
    int zero_tracking;

    int dac_value;

    /**
     * Formats the software version for the
     *  diagnostics.
     *
     * @return the formatted string
     */
    virtual std::string get_software_version()
    {
      //concatenate versions in a string.
      std::string full_version;

      std::stringstream ss;
      ss << "current: " << software_version_current;

      full_version = ss.str();

      return full_version;
    };
  };

  class BiotacData
    : public GenericTactileData
  {
  public:
    BiotacData()
      : GenericTactileData()
    {};

    BiotacData(const BiotacData& btac)
      : GenericTactileData(btac.tactile_data_valid, btac.sample_frequency,
                           btac.manufacturer, btac.serial_number,
                           btac.software_version_current,
                           btac.software_version_server,
                           btac.software_version_modified,
                           btac.pcb_version),
        pac0(btac.pac0), pac1(btac.pac1),
        pdc(btac.pdc), tac(btac.tac),
        tdc(btac.tdc)
    {
      for(unsigned int i =0; i<btac.electrodes.size() ;i++)
      {
        electrodes[i] = btac.electrodes[i];
      }
    };

    BiotacData(const GenericTactileData& gtd)
      : GenericTactileData(gtd.tactile_data_valid, gtd.sample_frequency,
                           gtd.manufacturer, gtd.serial_number,
                           gtd.software_version_current,
                           gtd.software_version_server,
                           gtd.software_version_modified,
                           gtd.pcb_version)
    {};

    ~BiotacData() {};

    int pac0; //always there, in word[0] and 1; int16u (2kHz)
    int pac1; //int16u

    int pdc; //int16u in word[2]

    int tac; //int16u in word[2]
    int tdc; //int16u in word[2]
    boost::array<short int, 19ul> electrodes; //int16u in word[2]
  };

  class UBI0Data
    : public GenericTactileData
  {
  public:
    UBI0Data()
      : GenericTactileData()
    {};

    UBI0Data(const UBI0Data& ubi0)
      : GenericTactileData(ubi0.tactile_data_valid, ubi0.sample_frequency,
                           ubi0.manufacturer, ubi0.serial_number,
                           ubi0.software_version_current,
                           ubi0.software_version_server,
                           ubi0.software_version_modified,
                           ubi0.pcb_version)
    {
      for(unsigned int i =0; i<ubi0.distal.size() ;i++)
      {
        distal[i] = ubi0.distal[i];
      }
      for(unsigned int i =0; i<ubi0.middle.size() ;i++)
      {
        middle[i] = ubi0.middle[i];
      }
      for(unsigned int i =0; i<ubi0.proximal.size() ;i++)
      {
        proximal[i] = ubi0.proximal[i];
      }
    };

    UBI0Data(const GenericTactileData& gtd)
      : GenericTactileData(gtd.tactile_data_valid, gtd.sample_frequency,
                           gtd.manufacturer, gtd.serial_number,
                           gtd.software_version_current,
                           gtd.software_version_server,
                           gtd.software_version_modified,
                           gtd.pcb_version)
    {};

    ~UBI0Data() {};


    boost::array<unsigned short int, 12ul> distal;
    boost::array<unsigned short int, 4ul> middle;
    boost::array<unsigned short int, 4ul> proximal;
  };

  class UBI0PalmData
  {
  public:
    UBI0PalmData()
    {};

    UBI0PalmData(const UBI0PalmData& ubi0)
    {
      for(unsigned int i =0; i<ubi0.palm.size() ;i++)
      {
        palm[i] = ubi0.palm[i];
      }
    };

    ~UBI0PalmData() {};

    boost::array<unsigned short int, 16ul> palm;
  };

  struct AllTactileData
  {
    BiotacData biotac;
    PST3Data pst;
    UBI0Data ubi0;
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
