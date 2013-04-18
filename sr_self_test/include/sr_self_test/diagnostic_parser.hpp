/**
 * @file   diagnostic_parser.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Mar 27 06:26:23 2013
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
 * @brief Parsing the diagnostics for jitter, dropped messages, etc...
 *
 *
 */

#ifndef _DIAGNOSTIC_PARSER_HPP_
#define _DIAGNOSTIC_PARSER_HPP_

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/variant.hpp>
#include <sstream>
#include <ros/ros.h>

namespace shadow_robot
{
  typedef boost::variant<int, double> DiagValues;
  typedef std::map<std::string, std::vector<DiagValues> > DiagMap;

  class VariantParser
    : public boost::static_visitor<void>
  {
  public:
    void operator()(int int_val) const
    {
      values_it->second[0] = ::atoi( new_value.c_str() );
    }

    void operator()(double double_val) const
    {
      values_it->second[0] = ::atof(new_value.c_str() );
    }

    DiagMap::iterator values_it;
    std::string new_value;
  };

  class VariantGreaterThan
    : public boost::static_visitor<bool>
  {
  public:
    bool operator()(const int value1, const int value2) const
    {
      return value1 >= value2;
    }

    bool operator()(const double value1, const double value2) const
    {
      return value1 >= value2;
    }

    bool operator()(const double value1, const int value2) const
    {
      return value1 >= value2;
    }

    bool operator()(const int value1, const double value2) const
    {
      return value1 >= value2;
    }
  };

  class BaseDiagnostics
  {
  public:
    BaseDiagnostics(std::string name)
      : name(name)
    {}

    ~BaseDiagnostics()
    {}

    virtual void parse_diagnostics(std::vector<diagnostic_msgs::KeyValue> values,
                                   short level) = 0;

    virtual std::pair<bool, std::string> to_string() = 0;

    std::string name;
  };

  class MinMaxDiagnostics
    : public BaseDiagnostics
  {
  public:
    MinMaxDiagnostics(std::string name)
      : BaseDiagnostics(name)
    {};

    ~MinMaxDiagnostics()
    {};

    virtual void parse_diagnostics(std::vector<diagnostic_msgs::KeyValue> values,
                                   short level)
    {
      for( size_t values_i = 0; values_i < values.size(); ++values_i )
      {
        DiagMap::iterator values_it;
        values_it = values_->begin();
        for(values_it = values_->begin(); values_it != values_->end(); ++values_it)
        {
          if( values[values_i].key.compare(values_it->first) == 0 )
          {
            VariantParser parser;
            parser.new_value = values[values_i].value;
            parser.values_it = values_it;
            boost::apply_visitor( parser, values_it->second[0]);
          }
        }
      }
    }

    virtual std::pair<bool, std::string> to_string()
    {
      std::stringstream ss;
      bool ok = true;

      ss << "Diagnostics[" << name << "]:";

      DiagMap::iterator values_it;
      VariantGreaterThan greater_than;
      for(values_it = values_->begin(); values_it != values_->end(); ++values_it)
      {
        //is value > min?
        bool min_comp = boost::apply_visitor(greater_than, values_it->second[0], values_it->second[1]);
        //is value > max?
        bool max_comp = boost::apply_visitor(greater_than, values_it->second[0], values_it->second[2]);

        // value is in the [min;max] interval
        if( min_comp && (!max_comp) )
        {
          ss <<" OK(";
        }
        else
        {
          ok = false;
          ss <<" ERROR(";
        }

        ss << values_it->first << "=" << values_it->second[0] <<")";
      }

      return std::pair<bool, std::string>(ok, ss.str());
    }

  protected:
    boost::shared_ptr< DiagMap > values_;
  };

  class RTLoopDiagnostics
    : public MinMaxDiagnostics
  {
  public:
    RTLoopDiagnostics(std::string name)
      : MinMaxDiagnostics(name)
    {
      values_.reset(new DiagMap() );
      std::vector<DiagValues> jitter(3);
      jitter[0] = 0.0; //current value
      jitter[1] = 0.0; //min
      jitter[2] = 100.0; //max
      values_->insert( std::pair<std::string, std::vector<DiagValues> >("Avg Loop Jitter (us)", jitter) );
    }

    ~RTLoopDiagnostics()
    {}

  private:
    double avg_jitter;
    int control_loop_overruns;
  };

  class EtherCATMasterDiagnostics
    : public MinMaxDiagnostics
  {
  public:
    EtherCATMasterDiagnostics(std::string name)
      : MinMaxDiagnostics(name)
    {
      values_.reset(new DiagMap() );
      std::vector<DiagValues> dropped_packet(3);
      dropped_packet[0] = 0; //current value
      dropped_packet[1] = 0; //min
      dropped_packet[2] = 200; //max (TODO: this should be a ratio dropped/sent packets??)
      values_->insert( std::pair<std::string, std::vector<DiagValues> >("Dropped Packets", dropped_packet) );
    }

    ~EtherCATMasterDiagnostics()
    {};
  };

  class IsOKDiagnostics
    : public BaseDiagnostics
  {
  public:
    IsOKDiagnostics(std::string name)
      : BaseDiagnostics(name)
    {
    };

    ~IsOKDiagnostics()
    {};

    virtual void parse_diagnostics(std::vector<diagnostic_msgs::KeyValue> values,
                                   short level)
    {
      level_ = level;
    };

    virtual std::pair<bool, std::string> to_string()
    {
      std::stringstream ss;
      bool ok = true;

      ss << "Diagnostics[" << name << "]:";

      if( level_ == diagnostic_msgs::DiagnosticStatus::ERROR )
      {
        ok = false;
        ss << " status = ERROR";
      }
      else if( level_ == diagnostic_msgs::DiagnosticStatus::WARN )
      {
        ss << " status = WARN";
      }
      else
      {
        ss << " status = OK";
      }

      return std::pair<bool, std::string>(ok, ss.str());
    };

  protected:
    short level_;
  };

  class DiagnosticParser
  {
  public:
    DiagnosticParser();
    ~DiagnosticParser()
    {};

    void parse_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

  private:
    ros::NodeHandle nh_;

    ///ROS subscriber to the diagnostics_agg topic
    ros::Subscriber diag_sub_;
    /**
     * Susbscribed to the diagnostics_agg topic.
     * @param msg new incoming msg
     */
    void diagnostics_agg_cb_(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);

    boost::ptr_vector<BaseDiagnostics> diagnostics_;
  };
}

  /* For the emacs weenies in the crowd.
     Local Variables:
     c-basic-offset: 2
     End:
  */

#endif /* _DIAGNOSTIC_PARSER_HPP_ */
