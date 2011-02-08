#ifndef SRH_ETHERCAT_HARDWARE_MOTOR_TRACE_H
#define SRH_ETHERCAT_HARDWARE_MOTOR_TRACE_H

#include <sr_edc_ethercat_drivers/MotorTraceSample.h>
#include <sr_edc_ethercat_drivers/ActuatorInfo.h>
#include <sr_edc_ethercat_drivers/MotorTrace.h>

#include <realtime_tools/realtime_publisher.h>
#include <boost/thread/condition.hpp>  // Missing from realtime_publisher : wg-ros-pkg Ticket #4682
#include <boost/utility.hpp>
#include <string>

namespace sr_edc_ethercat_drivers
{

/** 
 * \brief Class to buffer and publish previous 1-second of motor data
 *
 * Data is sampled with each cycle of realtime loop (1kHz) so that there
 * is too much data to be published continuouly.  Instead of continuously 
 * publishing data, the trace buffers the previous X seconds of samples. 
 * Publishing can be triggered as result of event or request.
 */
class MotorTraceBuffer : private boost::noncopyable
{
public:
  MotorTraceBuffer(unsigned trace_size);
  bool initialize(const sr_edc_ethercat_drivers::ActuatorInfo &actuator_info);
  void flagPublish(const std::string &reason, int level, int delay);
  void checkPublish();
  void sample(const sr_edc_ethercat_drivers::MotorTraceSample &s);
  void reset();
protected:
  unsigned trace_size_;  //!< size of trace vector
  unsigned trace_index_; //!< index of most recent element in trace vector
  unsigned published_traces_; //!< number of times motor trace has been published
  std::vector<sr_edc_ethercat_drivers::MotorTraceSample> trace_buffer_;
  realtime_tools::RealtimePublisher<sr_edc_ethercat_drivers::MotorTrace> *publisher_;
  int publish_delay_;
  int publish_level_;
  std::string publish_reason_;
};


};

#endif //SRH_ETHERCAT_HARDWARE_MOTOR_TRACE_H
