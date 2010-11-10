/**
 * @file math_utils.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#ifndef MATH_UTILS_HPP_
#define MATH_UTILS_HPP_

#include "MersenneTwister.h"

namespace math_utils
{

class MathUtils
{
public:
    MathUtils(){};
    ~MathUtils(){};

    /**
     * Computes a random number between min and max. It is the responsibility
     * of the user to ensure that min < max as it is not tested.
     * @param min
     * @param max
     * @return random number between min and max
     */
    float maut_random(float min, float max);

protected:
    ///uses the Mersenne Twister algorithm
    MTRand mtrand;
};



}//end namespace

#endif /* MATH_UTILS_HPP_ */
