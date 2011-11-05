/**
 * @file math_utils.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#include "math_utils.hpp"
#include <ros/ros.h>
namespace math_utils
{

float MathUtils::maut_random( float min, float max )
{
    //TODO: read the seed from entropy key
    //long int seed = 0;
    //mtrand.seed(seed);
    double rand = mtrand.rand();
    rand *= ((double)(max - min));
    rand += min;
    return (float)rand;
}
}
