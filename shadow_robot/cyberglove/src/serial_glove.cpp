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
  const std::string CybergloveFreq::hundred_hz = "t 1152 1\r"; //100Hz not possible with this glove
  const std::string CybergloveFreq::fourtyfive_hz = "t 2560 1\r"; //fastest frequency with this glove
  const std::string CybergloveFreq::ten_hz = "t 11520 1\r"; //10Hz
  const std::string CybergloveFreq::one_hz = "t 57600 2\r"; //1Hz
}

namespace cyberglove
{
  const short CybergloveSerial::glove_size = 22;

  CybergloveSerial::CybergloveSerial(std::string serial_port, boost::function<void(std::vector<int>)> callback) :
    nb_msgs_received(0), glove_pos_index(0), current_value(0)
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

  int CybergloveSerial::set_filtering(int value)
  {
    //TODO: implement this
    return 0;
  }

  int CybergloveSerial::set_frequency(std::string frequency)
  {
    cereal_port->write(frequency.c_str(), frequency.size());
    cereal_port->flush();
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
      //the line starts with S (83 in ASCII), followed by the sensors
      if( current_value == 83 )
      {
        ++nb_msgs_received;
        glove_pos_index = 0;
        //std::cout << std::endl;
      }
      else
      {
        //if not the end of the line
        if( current_value != 0 )
        {
          //glove_positions[glove_pos_index] = (((float)current_value) - 1.0f) / 254.0f;
          glove_positions[glove_pos_index] = current_value;
          ++glove_pos_index;
        }
        else //full message received, call the callback function now
        {
          callback_function(glove_positions);
        }
      }
      //  std::cout << (int)(unsigned char)world[i] << " ";
    }
  }

  int CybergloveSerial::get_nb_msgs_received()
  {
    return nb_msgs_received;
  }
}

void callback(std::vector<int> glove_pos)
{
  std::cout << "Data received: ";
  for (unsigned int i = 0; i < glove_pos.size(); ++i)
  {
    std::cout << glove_pos[i] << " ";
  }
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  boost::shared_ptr<cyberglove::CybergloveSerial> serial_glove(new cyberglove::CybergloveSerial("/dev/ttyUSB0", boost::bind(&callback, _1)));

  int res = -1;
  res = serial_glove->set_filtering(0);
  std::cout << "FILTERING SET TO : " << res << std::endl;

  cyberglove_freq::CybergloveFreq frequency;
  res = serial_glove->set_frequency(frequency.one_hz);
  std::cout << "PERIOD SET TO : " << res << std::endl;

  res = serial_glove->start_stream();

  sleep(5);
  std::cout << "Stopping" << std::endl;

  int nb_msgs = serial_glove->get_nb_msgs_received();
  std::cout << "Nb messages received = "<< nb_msgs
            << " frequency = " << nb_msgs/5 << "Hz" << std::endl;

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
