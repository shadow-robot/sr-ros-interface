/**
 * @file   geometry.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Jul 15 10:25:54 2010
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
