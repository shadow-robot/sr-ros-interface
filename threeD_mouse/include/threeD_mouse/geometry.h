/**
 * @file   geometry.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Jul 15 10:25:54 2010
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


#ifndef _QUATERNION_H_
#define _QUATERNION_H_

namespace geometry
{
  class Quaternion
  {
  public:
    Quaternion();
    Quaternion(double nx, double ny, double nz, double nw);
    Quaternion(Quaternion& q);
    Quaternion(const Quaternion& q);
    ~Quaternion();

    /**
     * Convert roll, pitch, yaw (Euler Angles) to Quaternion
     * 
     * @param pitch pitch angle
     * @param yaw yaw angle
     * @param roll roll angle
     * 
     * @return 
     */
    Quaternion euler_to_quaternion(float pitch, float yaw, float roll);
    void normalise(Quaternion quater);

    double x;
    double y;
    double z;
    double w;

  private:
    static const double pi_over_360;

  };

  class Translation
  {
  public:
    Translation();
    Translation(double nx, double ny, double nz);
    Translation(Translation& t);
    Translation(const Translation& t);

    double x;
    double y;
    double z;
  };

  class Pose
  {
  public:
    Pose();
    ~Pose();

    Translation translation;

    Quaternion quaternion;
  };
};

#endif
