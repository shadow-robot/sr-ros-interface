/**
 * @file   diagnostics_common.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Apr 23 05:54:44 2013
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
 *
 * @brief Contains the common classes for parsing the diagnostics (generic class
 *        to check for min/max, whether a diag is OK or ERROR, etc...)
 *
 *
 */

#ifndef _DIAGNOSTIC_COMMON_HPP_
#define _DIAGNOSTIC_COMMON_HPP_

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/variant.hpp>
#include <sstream>
#include <ros/ros.h>
#include <self_test/self_test.h>


namespace shadow_robot
{
  typedef boost::variant<int, double> DiagValues;

  class DiagnosticTest
  {
  public:
    DiagnosticTest()
    {};
    ~DiagnosticTest()
    {};

    std::vector<DiagValues> received_values;
    std::pair<DiagValues, DiagValues> min_max;
  };

  typedef std::map<std::string, DiagnosticTest> DiagMap;

  /**
   * Adds a new value to the list of received value for a given
   * test.
   */
  class VariantParser
    : public boost::static_visitor<void>
  {
  public:
    void operator()(int int_val) const
    {
      values_it->second.received_values.push_back( ::atoi( new_value.c_str() ) );
    }

    void operator()(double double_val) const
    {
      values_it->second.received_values.push_back( ::atof(new_value.c_str() ) );
    }

    DiagMap::iterator values_it;
    std::string new_value;
  };

  /**
   * Compare to Diagnostic Values
   */
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
    BaseDiagnostics(std::string name, self_test::TestRunner* test_runner)
      : name(name), test_runner_(test_runner)
    {}

    ~BaseDiagnostics()
    {}

    virtual void parse_diagnostics(std::vector<diagnostic_msgs::KeyValue> values,
                                   short level, std::string full_name) = 0;

    virtual void add_test()
    {
      test_runner_->add(full_name, this, &BaseDiagnostics::run_test);
    }

    virtual void run_test(diagnostic_updater::DiagnosticStatusWrapper& status)
    {
      std::pair<bool, std::string> res = to_string_();
      if( res.first )
        status.summary( diagnostic_msgs::DiagnosticStatus::OK, res.second );
      else
        status.summary( diagnostic_msgs::DiagnosticStatus::ERROR, res.second );
    };

    virtual std::auto_ptr<BaseDiagnostics> shallow_clone(std::string name) = 0;

    std::string name;
    std::string full_name;

  protected:
    self_test::TestRunner* test_runner_;

    virtual std::pair<bool, std::string> to_string_() = 0;
  };

  class MinMaxDiagnostics
    : public BaseDiagnostics
  {
  public:
    MinMaxDiagnostics(std::string name, self_test::TestRunner* test_runner)
      : BaseDiagnostics(name, test_runner)
    {};

    ~MinMaxDiagnostics()
    {};

    virtual void parse_diagnostics(std::vector<diagnostic_msgs::KeyValue> values,
                                   short level, std::string full_name)
    {
      this->full_name = full_name;

      for( size_t values_i = 0; values_i < values.size(); ++values_i )
      {
        DiagMap::iterator values_it;
        for(values_it = values_->begin(); values_it != values_->end(); ++values_it)
        {
          if( values[values_i].key.compare(values_it->first) == 0 )
          {
            VariantParser parser;
            parser.new_value = values[values_i].value;
            parser.values_it = values_it;
            boost::apply_visitor( parser, values_it->second.min_max.first);
          }
        }
      }
    }

    virtual std::auto_ptr<BaseDiagnostics> shallow_clone(std::string name)
    {
      std::auto_ptr<BaseDiagnostics> tmp( new MinMaxDiagnostics(name, test_runner_) );
      return tmp;
    };

  protected:
    boost::shared_ptr< DiagMap > values_;

    virtual std::pair<bool, std::string> to_string_()
    {
      std::stringstream ss;
      bool ok = true;
      DiagValues out_of_range_value;

      ss << "\nDiagnostics[" << name << "]:";

      DiagMap::iterator values_it;
      VariantGreaterThan greater_than;
      for(values_it = values_->begin(); values_it != values_->end(); ++values_it)
      {
        //We're checking all values. If one is out of the specified range -> test fails
        for(size_t i = 0; i<values_it->second.received_values.size(); ++i)
        {
          //is value > min?
          bool min_comp = boost::apply_visitor(greater_than, values_it->second.received_values[i], values_it->second.min_max.first);
          //is value > max?
          bool max_comp = boost::apply_visitor(greater_than, values_it->second.received_values[i], values_it->second.min_max.second);

          // value is in the [min;max] interval
          if( min_comp && (!max_comp) )
          { }
          else
          {
            ok = false;
            out_of_range_value = values_it->second.received_values[i];
            break; //test fails no need to go further
          }
        }
        if (ok)
        {
          ss <<" OK(";
        }
        else
        {
          ss <<" ERROR(";
        }

        //we're returning the first value that was out of the range
        ss << values_it->first << "=" << out_of_range_value <<")";
      }

      return std::pair<bool, std::string>(ok, ss.str());
    }
  };

  class IsOKDiagnostics
    : public BaseDiagnostics
  {
  public:
    IsOKDiagnostics(std::string name, self_test::TestRunner* test_runner)
      : BaseDiagnostics(name, test_runner)
    {};

    ~IsOKDiagnostics()
    {};

    virtual void parse_diagnostics(std::vector<diagnostic_msgs::KeyValue> values,
                                   short level, std::string full_name)
    {
      this->full_name = full_name;
      level_ = level;
    };

    virtual std::auto_ptr<BaseDiagnostics> shallow_clone(std::string name)
    {
      std::auto_ptr<BaseDiagnostics> tmp( new IsOKDiagnostics(name, test_runner_) );
      return tmp;
    };

  protected:
    short level_;

    virtual std::pair<bool, std::string> to_string_()
    {
      std::stringstream ss;
      bool ok = true;

      ss << "Diagnostics[" << full_name << "]:";

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
  };
}

  /* For the emacs weenies in the crowd.
     Local Variables:
     c-basic-offset: 2
     End:
  */

#endif /* _DIAGNOSTIC_COMMON_HPP_ */
