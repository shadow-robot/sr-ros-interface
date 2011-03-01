#!/usr/bin/env python

import roslib; roslib.load_manifest("sr_move_arm")
import rospy
from object_manipulation_msgs.msg import GripperTranslation, ManipulationPhase

##abort exception
class Aborted(Exception): pass

class ReactiveGrasper(object):
    """
    reactive/guarded movement and grasping
    """
    def __init__(self, which_arm = 'r', slip_detection = False):
        self.using_slip_detection = slip_detection
        self.whicharm = which_arm
        
        #force to use when closing hard
        self.close_force = 100
        if self.using_slip_detection:
            self.close_force = 10

        #has the hand been closed hard recently?
        self._closed_hard = 0

        #collision name of support surface
        self._table_name = "table"

        #for taking photos of each step
        self._photo = 0

        #broadcast the current phase of the manipulation
        self._phase_pub = rospy.Publisher('/reactive_manipulation_phase', ManipulationPhase)

        #dictionary for ManipulationPhase
        self.manipulation_phase_dict = {}
        for element in dir(ManipulationPhase):
            if element[0].isupper():
                self.manipulation_phase_dict[eval('ManipulationPhase.'+element)] = element

        #dictionary of return values for reactive approach
        self.reactive_approach_result_dict = {"success":0, "within goal threshold":1, "both fingers touching":2, "ran out of tries":3, "touching table":4, "saw palm contact":5, "grasp infeasible":6}

        #dictionary of return values for reactive_grasp
        self.reactive_grasp_result_dict = {"success":0, "ran out of grasp tries":1, "ran out of approach tries":2, \
                                               "aborted":3, "grasp infeasible":4}

        rospy.loginfo("done with ReactiveGrasper init for the %s arm"%self.whicharm)

    def broadcast_phase(self, phase):
        """
        broadcast the current manipulation phase (of type ManipulationPhase)
        """
        rospy.loginfo("broadcasting reactive phase %s"%self.manipulation_phase_dict[phase])
        self._phase_pub.publish(phase)
    

    def reactive_grasp(self, approach_pose, grasp_pose, joint_path = None, side_step = .015, back_step = .03,
                       approach_num_tries = 10, goal_pos_thres = 0.01, grasp_num_tries = 2, 
                       forward_step = 0.03, object_name = "points", table_name = "table", 
                       grasp_adjust_x_step = .02, grasp_adjust_z_step = .015, grasp_adjust_num_tries = 3):
        rospy.logerr("Reactive Grasp: Not implemented yet")

        self._table_name = table_name

        self.check_preempt()

        #if approach_pose is not specified, use the current pose of the hand
        if approach_pose == None:
            rospy.logerr("TODO: get the current palm pose")
            return
        
        self.check_preempt()

        #compute (unit) approach direction in base_link frame
        approach_dir = self.compute_approach_dir(approach_pose, grasp_pose)

        num_tries = 0
        current_goal_pose = grasp_pose
        while 1:
            if approach_num_tries:
                #try the approach, unless the reactive approach is disabled (approach_num_tries is 0)
                approach_result = self.reactive_approach(approach_dir, current_goal_pose, joint_path, 
                                                         side_step, back_step, 
                                                         approach_num_tries, goal_pos_thres)
                #quit if the approach ran out of tries (couldn't get around the object)
                if approach_result == self.reactive_approach_result_dict["ran out of tries"]:
                    return self.reactive_grasp_result_dict["ran out of approach tries"]
                elif approach_result == self.reactive_approach_result_dict["grasp infeasible"]:
                    return self.reactive_grasp_result_dict["grasp infeasible"]
            else:
                #reactive approach is disabled; just move to the goal
                self.broadcast_phase(ManipulationPhase.MOVING_TO_GRASP)
                rospy.loginfo("reactive approach is disabled, moving to the goal")
                self.move_cartesian_step(grasp_pose, timeout = 10.0, settling_time = 5.0, blocking = 1)


    def compute_approach_dir(self, approach_pose, grasp_pose):
        """
        compute (unit) approach direction in base_link frame from an approach_pose and grasp_pose (returns scipy array)
        """
        approach_vect = self.compute_approach_vect(approach_pose, grasp_pose)
        approach_dir = approach_vect/scipy.dot(approach_vect, approach_vect)**.5
        return approach_dir


  ##compute approach vector in base_link frame from an approach_pose and grasp_pose (returns scipy array)
  def compute_approach_vect(self, approach_pose, grasp_pose):
    (approach_pos, approach_rot) = pose_stamped_to_lists(self.cm.tf_listener, approach_pose, 'base_link')
    (grasp_pos, grasp_rot) = pose_stamped_to_lists(self.cm.tf_listener, grasp_pose, 'base_link')
    approach_vect = scipy.array(grasp_pos) - scipy.array(approach_pos)
    return approach_vect

    def check_preempt(self):
        """
        Needs to be overloaded
        """
        pass
