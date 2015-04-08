#!/usr/bin/python

# Copyright 2015 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.


from moveit_commander import MoveGroupCommander


class SrRobotCommander(object):
    """
    Base class for hand and arm commanders
    """

    def __init__(self, name):
        """
        Initialize MoveGroupCommander object
        @param name - name of the MoveIt group
        """
        self._move_group_commander = MoveGroupCommander(name)

    def _set_joint_value_target(self, joint_states):
        """
        Set target of the robot's links
        @param joint_states - dictionary with joint name and value
        """
        self._move_group_commander.set_joint_value_target(joint_states)

    def _set_position_target(self, xyz, end_effector_link=""):
        """
        Specify a target position for the end-effector.
        @param xyz - new position of end-effector
        @param end_effector_link - name of the end effector link
        """
        self._move_group_commander.set_position_target(xyz, end_effector_link)

    def _get_joints_position(self):
        """
        Returns joints position
        @return - dictionary with joints positions
        """
        joints_names = self._move_group_commander.get_joints()
        joints_values = self._move_group_commander.get_current_joint_values()
        return dict(zip(joints_names, joints_values))

    def _get_joints_velocity(self):
        """
        Returns joints velocities
        @return - dictionary with joints velocities
        """
        # TODO Implement
        pass
