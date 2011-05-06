/**
 * @file   serial_glove.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu May  5 15:30:17 2011
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
 * @brief Communicate via the serial port with the Cyberglove.
 *
 */

#include "cyberglove/serial_glove.hpp"

#include <iostream>

namespace cyberglove_freq
{
  const std::string CybergloveFreq::fastest = "t 1152 0\r"; //fastest speed
  const std::string CybergloveFreq::hundred_hz = "t 1152 1\r"; //100Hz
  const std::string CybergloveFreq::fourtyfive_hz = "t 2560 1\r"; //45Hz
  const std::string CybergloveFreq::ten_hz = "t 11520 1\r"; //10Hz
  const std::string CybergloveFreq::one_hz = "t 57600 2\r"; //1Hz
}

namespace cyberglove
{
  const unsigned short CybergloveSerial::glove_size = 22;

  CybergloveSerial::CybergloveSerial(std::string serial_port, boost::function<void(std::vector<float>, bool)> callback) :
    nb_msgs_received(0), glove_pos_index(0), current_value(0), light_on(true), button_on(true)
  {
    cereal_port = boost::shared_ptr<cereal::CerealPort>(new cereal::CerealPort());

    std::cout << "Opening serial port "<< serial_port ;
    cereal_port->open(serial_port.c_str());

    std::cout << " result : " << cereal_port->portOpen() << std::endl;

    callback_function = callback;

    //initialize the vector of positions with 0s
    for (int i = 0; i < glove_size; ++i)
    {
      glove_positions.push_back(0);
    }
  }

  CybergloveSerial::~CybergloveSerial()
  {
    cereal_port->stopStream();

    cereal_port->write("^c", 2);
  }

  int CybergloveSerial::set_filtering(bool value)
  {
    if( value ) //Filtering will be on
    {
      cereal_port->write("f 1\r", 4);
      std::cout << " - Data filtered" << std::endl;
    }
    else // Filtering off
    {
      cereal_port->write("f 0\r", 4);
      std::cout << " - Data not filtered" << std::endl;
    }
    cereal_port->flush();

    //wait for the command to be applied
    sleep(1);

    return 0;
  }

  int CybergloveSerial::set_transmit_info(bool value)
  {
    if( value ) //transmit info will be on
    {
      cereal_port->write("u 1\r", 4);
      std::cout << " - Additional info transmitted" << std::endl;
    }
    else // transmit info off
    {
      cereal_port->write("u 0\r", 4);
      std::cout << " - Additional info not transmitted" << std::endl;
    }
    cereal_port->flush();

    //wait for the command to be applied
    sleep(1);

    return 0;
  }

  int CybergloveSerial::set_frequency(std::string frequency)
  {
    cereal_port->write(frequency.c_str(), frequency.size());
    cereal_port->flush();

    //wait for the command to be applied
    sleep(1);
    return 0;
  }

  int CybergloveSerial::start_stream()
  {
    std::cout << "starting stream"<<std::endl;

    cereal_port->startReadStream(boost::bind(&CybergloveSerial::stream_callback, this, _1, _2));

    cereal_port->write("S", 1);
    cereal_port->flush();

    return 0;
  }

  void CybergloveSerial::stream_callback(char* world, int length)
  {
    for (int i = 0; i < length; ++i)
    {
      current_value = (int)(unsigned char)world[i];
      switch( current_value )
      {
        //the line starts with S, followed by the sensors
      case 'S':
        ++nb_msgs_received;
        glove_pos_index = 0;
        break;

        //full message received: call the callback function now
      case 0:
        callback_function(glove_positions, light_on);
        break;

        //glove sensor value
      default:
        //last char if the status indication is transmitted
        if( glove_pos_index == glove_size )
        {
          //the status bit 1 corresponds to the button
          if(current_value & 2)
            button_on = true;
          else
            button_on = false;

          //the status bit 2 corresponds to the light
          if(current_value & 4)
            light_on = true;
          else
            light_on = false;
        }
        else
        {
          // the values sent by the glove are in the range [1;254]
          //   -> we convert them to float in the range [0;1]
          glove_positions[glove_pos_index] = (((float)current_value) - 1.0f) / 254.0f;
        }
        ++glove_pos_index;
        break;
      }
    }
  }

  int CybergloveSerial::get_nb_msgs_received()
  {
    return nb_msgs_received;
  }
}

void callback(std::vector<float> glove_pos, bool light_on)
{
  if(light_on)
  {
    std::cout << "Data received: ";
    for (unsigned int i = 0; i < glove_pos.size(); ++i)
    {
      std::cout << glove_pos[i] << " ";
    }
    std::cout << std::endl;
  }
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
