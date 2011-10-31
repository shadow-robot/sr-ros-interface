/**
 * @file   denso_arm_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Dan Greenwald <dg@shadowrobot.com>
 * @date   Mon Oct 31 09:26:15 2011
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
 * @brief A standard interface to the DENSO arm.
 *
 *
 */

#include "denso_arm/denso_arm_node.hpp"

namespace denso
{
  DensoArmNode::DensoArmNode()
  {}

  DensoArmNode::~DensoArmNode()
  {}

}

int main(int argc, char *argv[])
{
  boost::shared_ptr<denso::DensoArmNode> denso_node;
  denso_node = boost::shared_ptr<denso::DensoArmNode> ( new denso::DensoArmNode() );
  return 0;
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
