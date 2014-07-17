/**
 * @file   sr_math_utils.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Mon Jun 20 10:20:27 2011
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
 * @brief This is a header library used to implement some useful math
 * functions. It is used in our different packages.
 *
 *
 */

#ifndef _SR_MATH_UTILS_HPP_
#define _SR_MATH_UTILS_HPP_

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/thread/detail/singleton.hpp>


#include <ros/ros.h>

namespace sr_math_utils
{
static const double pi = 3.14159265;
/**
 * Convert an angle in degrees to an angle in radians.
 *
 * @param degrees the value in degrees
 *
 * @return the value in radians.
 */
static inline double to_rad(double degrees)
{
  return degrees * 0.017453292519943295;
}
/**
 * Convert an angle in degrees to an angle in degrees.
 *
 * @param rad the value in radians.
 *
 * @return the value in degrees.
 */
static inline double to_degrees(double rad)
{
  return rad * 57.295779513082323;
}
static inline int ipow(int base, int exp)
{
  int result = 1;
  while (exp)
  {
    if (exp & 1)
      result *= base;
    exp >>= 1;
    base *= base;
  }

  return result;
}
static inline bool is_bit_mask_index_true(long int bit_mask, int index)
{
  if (bit_mask & (((long int) 1) << index))
    return true;
  else
    return false;
}
static inline bool is_bit_mask_index_false(long int bit_mask, int index)
{
  return !(is_bit_mask_index_true(bit_mask, index));
}
/**
 * Increment a counter given a value which can overflow.
 *
 * WARNING: only works if called often enough: if new_value > last_value but
 * overflowed between the 2 calls, then we're not able to detect the overflow.
 *
 * @param full_value the full value (with no overflow)
 * @param new_value the new value we received from the motor.
 *
 * @return The new full value
 */
static inline uint64_t counter_with_overflow(uint64_t full_value, uint16_t new_value)
{
  uint16_t last_value = full_value & 0xFFFF; // Split the full value into the lower part
  full_value &= (uint64_t) 0xFFFFFFFFFFFF0000LL; // and the overflow part

  if (new_value < last_value) // if we overflowed
    full_value += (uint64_t) 0x0000000000010000LL; // then count the overflow

  full_value |= (uint64_t) new_value; // replace the bottom 16 bits with their new value

  return full_value;
}
/**
 * Interpolate linearly between the 2 points, for the given value
 *
 * y = y0 + (x-x0)*((y1-y0)/(x1-x0))
 *
 * @param x the X value to compute the interpolation for.
 * @param x0 the X value of our first point
 * @param y0 the Y value of our first point
 * @param x1 the X value of our second point
 * @param y1 the Y value of our second point
 *
 * @return the computed Y value (calibrated value)
 */
static inline double linear_interpolate_(double x,
                                         double x0, double y0,
                                         double x1, double y1)
{
  //y1 - y0
  double y = y1 - y0;
  // (y1 - y0) / (x1 - x0)
  y /= (x1 - x0);
  //  (x-x0)*((y1-y0)/(x1-x0))
  y *= (x - x0);
  //  y0 + (x-x0)*((y1-y0)/(x1-x0))
  y += y0;

  return y;
}
/**
 * Checks the sign of a given number.
 *
 * @param x the number we want to study.
 *
 * @return 1 if x is positive, -1 if negative.
 */
static inline int sign(double x)
{
  return x < 0.0 ? -1 : 1;
}

////////////////
//  FILTERS  //
//////////////
namespace filters
{

class LowPassFilter
{
public:
  LowPassFilter(double tau = 0.05)
    : is_first(true), dt(0.0),
    timestamp_1(0.0), q_prev(0.0),
    tau(tau), value_derivative(0.0, 0.0)
  {
  };
  /**
   * Computes the filtered value and its derivative.
   *
   * @param q the newly received value.
   * @param timestamp the time at which the last
   *                  measurement was made (in sec).
   *
   * @return a pair containing the filtered value first, then
   *         the derivative.
   */
  std::pair<double, double> compute(double q, double timestamp)
  {
    if (is_first)
    {
      q_prev = q;
      //initializing dt to 1ms
      dt = 0.001;

      is_first = false;
    }
    else
      dt = timestamp - timestamp_1;

    double alpha = exp(-dt / tau);

    //filtering the input
    value_derivative.first = alpha * value_derivative.first + (1 - alpha) * q;
    //filtering the derivative
    value_derivative.second = alpha * value_derivative.second + (1 - alpha) / dt * (q - q_prev);

    q_prev = q;
    timestamp_1 = timestamp;

    return value_derivative;
  };

private:
  bool is_first;
  double tau, dt, timestamp_1, q_prev;

  std::pair<double, double> value_derivative;
};

/**
 * An alpha beta filter as described on:
 *  http://en.wikipedia.org/wiki/Alpha_beta_filter
 *
 */
class AlphaBetaFilter
{
public:
  AlphaBetaFilter(double alpha = 0.85, double beta = 0.05)
    : a(alpha), b(beta),
    xk_1(0.0), vk_1(0.0), xk(0.0), vk(0.0), rk(0.0),
    dt(0.0), timestamp_1(0.0)
  {
  };
  /**
   * Computes the filtered value and its derivative.
   *
   * @param xm the newly received value.
   * @param timestamp the time at which the last
   *                  measurement was made (in sec).
   *
   * @return a pair containing the filtered value first, then
   *         the derivative.
   */
  std::pair<double, double> compute(double xm, double timestamp)
  {
    dt = timestamp - timestamp_1;

    xk = xk_1 + (vk_1 * dt);
    vk = vk_1;

    rk = xm - xk;

    xk += a * rk;
    vk += (b * rk) / dt;

    value_derivative.first = xk;
    value_derivative.second = vk;

    xk_1 = xk;
    vk_1 = vk;
    timestamp_1 = timestamp;

    return value_derivative;
  };

protected:
  double a, b;
  double xk_1, vk_1, xk, vk, rk;
  double dt, timestamp_1;

  std::pair<double, double> value_derivative;
};
};


////////////////
//  RANDOM   //
//////////////

/**
 * This class is not supposed to be used as is: use the RandomDouble
 * singleton instead.
 */
class Random_
{
public:
  Random_() :
    dist(boost::uniform_real<>(0.0, 1.0))
  {
  };
  /**
   * Generate a random number between min and max
   * (or between 0 and 1 by default)
   *
   *
   * @return
   */
  template <typename T>
  T generate(T min = static_cast<T> (0), T max = static_cast<T> (1))
  {
    return static_cast<T> (min + dist(gen) * (max - min));
  }

protected:
  boost::mt19937 gen;
  boost::uniform_real<> dist;
};

/**
 * This is the usable singleton of the Random class.
 *
 * The typical usage for this class is:
 * -> generate double betwen 0 and 1
 *  sr_math_utils::Random::instance().generate<double>()
 *
 * -> generate integer between 0 and 10
 *  sr_math_utils::Random::instance().generate<int>(0, 10)
 *
 * -> generate double betwen 0 and 10
 *  sr_math_utils::Random::instance().generate<double>(0, 10)
 */
typedef boost::detail::thread::singleton < class Random_ > Random;
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */

#endif
