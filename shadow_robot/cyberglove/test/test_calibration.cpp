/**
 * @file   test_calibration.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Mar 29 15:03:12 2011
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
 * @brief  Testing the calibration mapping for the glove.
 *
 *
 */


#include <ros/ros.h>

#include <math.h>
#include <cyberglove/xml_calibration_parser.h>
#include <gtest/gtest.h>

#define TEST_EXPRESSION(a) EXPECT_EQ((a), meval::EvaluateMathExpression(#a))

using namespace ros;
using namespace xml_calibration_parser;

std::string path_to_calibration = "test/cyberglove_test.cal";

float epsilon = 0.01f;

XmlCalibrationParser calib_parser;

TEST(LookupTable, testSimple)
{
  float valtmp;
  valtmp = calib_parser.get_calibration_value(0.05f,"test1" );
  EXPECT_TRUE(fabs( valtmp - 50.0f ) < epsilon)
    << "Expected value : 50 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.1f,"test1" );
  EXPECT_TRUE(fabs( valtmp - 100.0f ) < epsilon)
    << "Expected value : 100 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.0f,"test1" );
  EXPECT_TRUE(fabs( valtmp - 0.0f ) < epsilon)
    << "Expected value : 0 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.2f,"test1" );
  EXPECT_TRUE(fabs( valtmp - 200.0f ) < epsilon)
    << "Expected value : 200 "
    << "Received value : "<< valtmp;
}

TEST(LookupTable, integrity)
{
  std::vector<XmlCalibrationParser::JointCalibration> myCalib = calib_parser.getJointsCalibrations();

 EXPECT_EQ(4, myCalib.size());
 for(unsigned int i = 0; i<myCalib.size() ; ++i)
   {
     EXPECT_EQ(2, myCalib[i].calibrations.size());
   }
}

TEST(LookupTable, testCalibNotStartingAtZero)
{
  float valtmp;
  valtmp = calib_parser.get_calibration_value(0.05f,"test2" );
  EXPECT_TRUE(fabs( valtmp - 35.0f ) < epsilon)
    << "Expected value : 35 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.1f,"test2" );
  EXPECT_TRUE(fabs( valtmp - 60.0f ) < epsilon)
    << "Expected value : 60 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.0f,"test2" );
  EXPECT_TRUE(fabs( valtmp - 10.0f ) < epsilon)
    << "Expected value : 10 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.2f,"test2" );
  EXPECT_TRUE(fabs(valtmp - 110.0f ) < epsilon)
    << "Expected value : 110 "
    << "Received value : "<< valtmp;
}

TEST(LookupTable, testRawNotStartingAtZero)
{
  float valtmp;
  valtmp = calib_parser.get_calibration_value(0.15f,"test3" );
  EXPECT_TRUE(fabs( valtmp - 35.0f ) < epsilon)
    << "Expected value : 35 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.2f,"test3" );
  EXPECT_TRUE(fabs( valtmp - 60.0f ) < epsilon)
    << "Expected value : 60 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.1f,"test3" );
  EXPECT_TRUE(fabs( valtmp - 10.0f ) < epsilon)
    << "Expected value : 10 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.3f,"test3" );
  EXPECT_TRUE(fabs(valtmp - 110.0f ) < epsilon)
    << "Expected value : 110 "
    << "Received value : "<< valtmp;
}

TEST(LookupTable, tableNotOrdered)
{
  float valtmp;
  valtmp = calib_parser.get_calibration_value(0.05f,"test4" );
  EXPECT_TRUE(fabs( valtmp - 35.0f ) < epsilon)
    << "Expected value : 35 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.1f,"test4" );
  EXPECT_TRUE(fabs( valtmp - 10.0f ) < epsilon)
    << "Expected value : 10 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.0f,"test4" );
  EXPECT_TRUE(fabs( valtmp - 60.0f ) < epsilon)
    << "Expected value : 60 "
    << "Received value : "<< valtmp;

  valtmp = calib_parser.get_calibration_value(0.2f,"test4" );
  EXPECT_TRUE(fabs( valtmp + 40.0f) < epsilon)
    << "Expected value : -40 "
    << "Received value : "<< valtmp;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){

  calib_parser = XmlCalibrationParser(path_to_calibration);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

}
