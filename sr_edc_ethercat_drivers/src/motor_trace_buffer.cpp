/**
 * @file   motor_trace_buffer.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Aug 23 11:39:25 2011
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
 *
 * @brief  Publishes the last second of the motor controller.
 *
 *
 */


#include <sr_edc_ethercat_drivers/motor_trace_buffer.h>

namespace sr_edc_ethercat_drivers
{

/**
 *
 */
MotorTraceBuffer::MotorTraceBuffer(unsigned trace_size) :
  trace_size_(trace_size),
  trace_index_(0),
  published_traces_(0)
{
  assert(trace_size_ > 0);
  trace_buffer_.reserve(trace_size_);
  reset();
}

void MotorTraceBuffer::reset()
{
  publish_delay_ = -1;
  publish_level_ = -1;
  publish_reason_ = "OK";
}


/** \brief Initializes motor trace publisher
 */
bool MotorTraceBuffer::initialize(const sr_edc_ethercat_drivers::ActuatorInfo &actuator_info)
{
  std::string topic("motor_trace");
  if (!actuator_info.name.empty())
    topic = topic + "/" + actuator_info.name;
  publisher_ = new realtime_tools::RealtimePublisher<sr_edc_ethercat_drivers::MotorTrace>(ros::NodeHandle(), topic, 1, true);
  if (publisher_ == NULL)
    return false;

  sr_edc_ethercat_drivers::MotorTrace &msg(publisher_->msg_);
  msg.actuator_info = actuator_info;
  msg.samples.reserve(trace_size_);

  return true;
}


/**  \brief Publishes motor trace if delay time is up
 */
void MotorTraceBuffer::checkPublish()
{
  if (publish_delay_ < 0)
    return;
  --publish_delay_;
  if (publish_delay_ >= 0)
    return;

  ++published_traces_;

  assert(publisher_ != NULL);
  if ((publisher_==NULL) || (!publisher_->trylock()))
    return;

  sr_edc_ethercat_drivers::MotorTrace &msg(publisher_->msg_);

  msg.header.stamp = ros::Time::now();
  msg.reason = publish_reason_;
  unsigned size=trace_buffer_.size();
  msg.samples.clear();
  msg.samples.reserve(size);

  // TODO : is there a beter way to copy data between two std::vectors?
  for (unsigned i=0; i<size; ++i) {
    msg.samples.push_back(trace_buffer_.at((trace_index_+1+i)%size));
  }

  // Cancel any delayed publishing from occuring
  publish_delay_ = -1;
  publish_level_ = -1;

  publisher_->unlockAndPublish();
}



/** \brief flags delayed publish of motor trace.
 *
 * New publish will only take precedence of previous publish iff level is higher than previous level
 */
void MotorTraceBuffer::flagPublish(const std::string &reason, int level, int delay)
{
  if (delay < 0)
    delay = 0;
  else if (delay > 900) {
    delay = 900;
  }
  if (level > publish_level_)
  {
    publish_reason_ = reason;
    publish_delay_ = delay;
    publish_level_ = level;
  }
}


/**  \brief Adds sample to motor trace.
 */
void MotorTraceBuffer::sample(const sr_edc_ethercat_drivers::MotorTraceSample &s)
{

  { // Add motor trace sample to trace buffer
    assert(trace_buffer_.size() <= trace_size_);
    if (trace_buffer_.size() >= trace_size_) {
      trace_index_ = (trace_index_+1)%trace_buffer_.size();
      trace_buffer_.at(trace_index_) = s;
    } else {
      trace_index_ = trace_buffer_.size();
      trace_buffer_.push_back(s);
    }
  }
}



};
