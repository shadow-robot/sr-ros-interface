/**
 * @file   movement_from_image.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep 27 10:05:01 2011
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
 * @brief  Reads a png file and creates a movement from it.
 *
 *
 */


#include "sr_movements/movement_from_image.hpp"

#include <iostream>

namespace shadowrobot
{
  MovementFromImage::MovementFromImage(std::string image_path)
    : PartialMovement()
  {
    png::image< png::rgb_pixel > image( image_path );

    generate_movement(image);
  }

  MovementFromImage::~MovementFromImage()
  {}


  void MovementFromImage::generate_movement( png::image<png::rgb_pixel> image )
  {
    png::byte empty_byte(0);
    double img_height = static_cast<double>( image.get_width() );
    for (size_t y = 0; y < image.get_height(); ++y)
    {
      bool no_pixel = true;
      for (size_t x = 0; x < image.get_width(); ++x)
      {
        if( (image[x][y].red == empty_byte) && (image[x][y].green == empty_byte)
            && (image[x][y].blue == empty_byte) )
        {
          no_pixel = false;
          steps.push_back( 1.0 - static_cast<double>(x) / img_height);
          break;
        }
      }
      if(no_pixel)
      {
        //not sending any targets for this point.
        steps.push_back( -1.0 );
      }
    }
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

