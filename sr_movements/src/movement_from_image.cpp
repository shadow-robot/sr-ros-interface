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
#include <ros/ros.h>

#include <iostream>

namespace shadowrobot
{
  MovementFromImage::MovementFromImage(std::string image_path)
    : PartialMovement()
  {
    image_ = boost::shared_ptr<Magick::Image>( new Magick::Image() );
    image_->read( image_path );

    nb_cols_ = image_->columns();
    nb_rows_ = image_->rows();

    generate_movement_();
  }

  MovementFromImage::~MovementFromImage()
  {}


  void MovementFromImage::generate_movement_()
  {
    const Magick::PixelPacket* pixel_cache = image_->getConstPixels(0,0,nb_cols_, nb_rows_);

    for( ssize_t col = 0; col < nb_cols_; ++col)
    {
      bool no_pixel = true;
      for( ssize_t row=0; row < nb_rows_; ++row)
      {
        const Magick::PixelPacket* tmp_pixel = pixel_cache + row * nb_cols_ + col;
        if( tmp_pixel->red != 0xFFFF && tmp_pixel->green != 0xFFFF
            && tmp_pixel->blue != 0xFFFF)
        {
          no_pixel = false;
          steps.push_back( 1.0 - static_cast<double>(row) / static_cast<double>(nb_rows_) );
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

