/**
 * @file   serial_glove.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Apr 22 10:25:55 2010
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
 * @brief  The C interface to interact with the cyberglove.
 *
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef SERIAL_GLOVE_H
#define SERIAL_GLOVE_H


  int setup_glove(const char* path_to_glove);
  float* glove_get_values(void);
  int read_button_value(void);


#define PORT "3490" // use one port, client request or sends data depending on message
#define GLOVE_SIZE 22 // number of sensors in the glove



//! call the enum names directly as if they are #defines
//! the message protocol headers, for passing or requesting data, or for issuing commands

  typedef enum {  
    MSG_TEST_CONNECTION = 100,
    MSG_UPDATE_SETPOINTS,
    MSG_GET_POSITIONS,
    MSG_RESTART_ROBOT,  

    MSG_DATA_READY = 200,
    MSG_DATA_UNAVAILABLE
  } msg_id_t;




#endif

#ifdef __cplusplus
}
#endif
