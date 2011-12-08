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

namespace tactiles
{
  class GenericTactileData
  {
  public:
    GenericTactileData() {};

    GenericTactileData(bool tactile_data_valid, int sample_frequency,
                       std::string manufacturer, std::string serial_number,
                       int software_version, int pcb_version)
      : tactile_data_valid(tactile_data_valid), sample_frequency(sample_frequency),
        manufacturer(manufacturer), serial_number(serial_number),
        software_version(software_version), pcb_version(pcb_version)
    {};

    ~GenericTactileData() {};

    bool tactile_data_valid;

    int which_sensor;
    int sample_frequency;
    std::string manufacturer;
    std::string serial_number;

    int software_version;
    int pcb_version;
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
                           pst3.software_version, pst3.pcb_version),
        pressure(pst3.pressure), temperature(pst3.temperature),
        debug_1(pst3.debug_1), debug_2(pst3.debug_2),
        pressure_raw(pst3.pressure_raw), zero_tracking(pst3.zero_tracking), dac_value(pst3.dac_value)
    {};


    PST3Data(const GenericTactileData& gtd)
      : GenericTactileData(gtd.tactile_data_valid, gtd.sample_frequency,
                           gtd.manufacturer, gtd.serial_number,
                           gtd.software_version, gtd.pcb_version)
    {};

    ~PST3Data() {};
    int pressure;
    int temperature;

    int debug_1;
    int debug_2;

    int pressure_raw;
    int zero_tracking;

    int dac_value;
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
                           btac.software_version, btac.pcb_version),
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
                           gtd.software_version, gtd.pcb_version)
    {};

    ~BiotacData() {};

    int pac0; //always there, in word[0] and 1; int16u (2kHz)
    int pac1; //int16u

    int pdc; //int16u in word[2]

    int tac; //int16u in word[2]
    int tdc; //int16u in word[2]
    boost::array<short int, 19ul> electrodes; //int16u in word[2]
  };

  struct AllTactileData
  {
    BiotacData biotac;
    PST3Data pst;
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
