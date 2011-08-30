/**
 * @file   geometry.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Jul 15 10:25:10 2010
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
 * @brief  
 * 
 * 
 */


#include "threeD_mouse/geometry.h"
#include <math.h>

namespace geometry
{
  /*****************
   *   Quaternion  *
   *****************/
  const double Quaternion::pi_over_360 = 0.0087266462599716477;


  Quaternion:: Quaternion()
    : x(0.0), y(0.0), z(0.0), w(0.0)
  {}

  Quaternion::Quaternion(double nx, double ny, double nz, double nw)
    : x(nx), y(ny), z(nz), w(nw)
  {}

  Quaternion:: Quaternion(Quaternion& q)
    : x(q.x), y(q.y), z(q.z), w(q.w)
  {}

  Quaternion::Quaternion(const Quaternion& q)
    : x(q.x), y(q.y), z(q.z), w(q.w)
  {}

  Quaternion:: ~Quaternion()
  {}

  Quaternion Quaternion::euler_to_quaternion(float pitch, float yaw, float roll)
  {
    // Basically we create 3 Quaternions, one for pitch, one for yaw, one for roll
    // and multiply those together.
    // the calculation below does the same, just shorter

    Quaternion quater; 

    double p = pitch * pi_over_360;
    double y = yaw * pi_over_360;
    double r = roll * pi_over_360;
 
    double sinp = sin(p);
    double siny = sin(y);
    double sinr = sin(r);
    double cosp = cos(p);
    double cosy = cos(y);
    double cosr = cos(r);
 
    quater.x = sinr * cosp * cosy - cosr * sinp * siny;
    quater.y = cosr * sinp * cosy + sinr * cosp * siny;
    quater.z = cosr * cosp * siny - sinr * sinp * cosy;
    quater.w = cosr * cosp * cosy + sinr * sinp * siny;
 
    normalise(quater);

    return Quaternion(quater);
  }

  void Quaternion::normalise(Quaternion quater)
  {
    // Don't normalize if we don't have to
    double mag2 = quater.w * quater.w + 
      quater.x * quater.x + quater.y * quater.y + 
      quater.z * quater.z;

    if (  mag2!=0.0 && (fabs(mag2 - 1.0f) > 0.01)) {
      double mag = sqrt(mag2);
      quater.w /= mag;
      quater.x /= mag;
      quater.y /= mag;
      quater.z /= mag;
    }
  }

  /*****************
   *  Translation  *
   *****************/

  Translation::Translation()
    : x(0.0), y(0.0), z(0.0)
  {}

  Translation::Translation(double nx, double ny, double nz)
    : x(nx), y(ny), z(nz)
  {}

  Translation::Translation(Translation& t)
    : x(t.x), y(t.y), z(t.z)
  {}

  Translation::Translation(const Translation& t)
    : x(t.x), y(t.y), z(t.z)
  {}

  /*****************
   *      Pose     *
   *****************/

  Pose::Pose()
    : translation(Translation()), quaternion(Quaternion())
  {}
    
  Pose::~Pose()
  {}

}; //end namespace
