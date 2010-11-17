/**
 * @file measure.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#ifndef MEASURE_HPP_
#define MEASURE_HPP_

namespace dataglove
{

class Measure
{
public:
    Measure();
    ~Measure();

    float compute_probability();
};
}

#endif /* MEASURE_HPP_ */
