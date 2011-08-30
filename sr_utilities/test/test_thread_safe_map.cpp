/**
 * @file   test_thread_safe_map.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Jun 16 11:44:00 2011
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
 * @brief  Testing the implementation of our thread safe map.
 *
 *
 */

#include "sr_utilities/thread_safe_map.hpp"

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <gtest/gtest.h>
#include <string>

class TestMultiThread
{
public:
  TestMultiThread(int num_threads)
  {
    last_value = 1;
    map_.insert("a", last_value);
    map_.insert("b", 2);

    boost::thread_group tg;
    tg.create_thread(boost::bind(&TestMultiThread::write, this));

    for(int i=0; i<num_threads; ++i)
      tg.create_thread(boost::bind(&TestMultiThread::read, this ));

    boost::this_thread::sleep(boost::posix_time::seconds(2));
    tg.join_all();
  };

  ~TestMultiThread()
  {};

  void write()
  {
    mut.lock();
    ++last_value;
    map_.update("a", last_value);
    mut.unlock();
    boost::this_thread::sleep(boost::posix_time::microseconds(1));
  };

  void read()
  {
    mut.lock_shared();
    int value = map_.find("a");
    EXPECT_EQ(value, last_value);
    mut.unlock_shared();

    value = map_.find("b");
    //b should be constant
    EXPECT_EQ(value, 2);

    boost::this_thread::sleep(boost::posix_time::microseconds(1));
  };

private:
  threadsafe::Map<int> map_;
  int last_value;
  boost::shared_mutex mut;
};

TEST(ThreadSafeMapOneThread, initialization)
{
  threadsafe::Map<int> map;

  map.insert("a", 1);
  map.insert("b", 2);

  int value = map.find("a");
  EXPECT_EQ(value, 1);

  value = map.find("b");
  EXPECT_EQ(value, 2);

  EXPECT_EQ(map.keys().size(), 2);

  EXPECT_EQ(map.keys()[0].compare("a"), 0 );
  EXPECT_EQ(map.keys()[1].compare("b"), 0 );
}

TEST(ThreadSafeMapOneThread, update)
{
  threadsafe::Map<int> map;

  map.insert("a", 1);
  map.insert("b", 2);

  int value = map.find("a");
  EXPECT_EQ(value, 1);

  value = map.find("b");
  EXPECT_EQ(value, 2);


  map.update("a", 3);
  value = map.find("a");
  EXPECT_EQ(value, 3);

  value = map.find("b");
  EXPECT_EQ(value, 2);
}

TEST(ThreadSafeMapMultiThreads, update)
{
  TestMultiThread tmt(12);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
