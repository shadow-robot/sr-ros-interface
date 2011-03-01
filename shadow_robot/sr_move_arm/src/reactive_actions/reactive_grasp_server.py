#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_move_arm')
import rospy

import sys

from object_manipulation_msgs.msg import ReactiveGraspAction, ReactiveGraspResult, \
    ManipulationResult, GripperTranslation, ReactiveLiftAction, ReactiveLiftResult, \
    ReactivePlaceAction, ReactivePlaceResult, ReactiveGraspFeedback, \
    ReactiveLiftFeedback, ReactivePlaceFeedback, ManipulationPhase
import actionlib
from object_manipulator.convert_functions import *
import reactive_grasp

from std_srvs.srv import Empty, EmptyResponse

##preempt exception
class Preempted(Exception): pass

class ReactiveGrasperWithPreempt(reactive_grasp.ReactiveGrasper):
    """
    reactive grasp class with overloaded preempt
    """
    def __init__(self, rg_as, lift_as, approach_as, place_as, which_arm='r', slip_detection=False ):
        
        reactive_grasp.ReactiveGrasper.__init__(self, which_arm, slip_detection)

        #add the action servers so we can check for preempts and send feedback
        self._rg_as = rg_as
        self._lift_as = lift_as
        self._approach_as = approach_as
        self._place_as = place_as

        self.rg_state = "off"
        
        self.pause_for_recording = 0

    ##overloaded to add action preempts
    def check_preempt(self):
        
        if self.rg_state == "rg" and self._rg_as.is_preempt_requested() or \
                self.rg_state == "lift" and self._lift_as.is_preempt_requested() or \
                self.rg_state == "approach" and self._approach_as.is_preempt_requested() or \
                self.rg_state == "place" and self._place_as.is_preempt_requested():
            
            rospy.loginfo("preempt seen, state=%s"%self.rg_state)
            
            raise Preempted

    ##overloaded to add feedback broadcasts
    def broadcast_phase(self, phase, send_feedback = 1):
        self._phase_pub.publish(phase)

        if self.rg_state == "rg":
            feedback = ReactiveGraspFeedback()
            feedback.manipulation_phase.phase = phase
            self._rg_as.publish_feedback(feedback)
        elif self.rg_state == "lift":
            feedback = ReactiveLiftFeedback()
            feedback.manipulation_phase.phase = phase
            self._lift_as.publish_feedback(feedback)
        elif self.rg_state == "place":
            feedback = ReactivePlaceFeedback()
            feedback.manipulation_phase.phase = phase
            self._place_as.publish_feedback(feedback)
        
        #if recording in the grasp playpen, pause at MOVING_TO_GRASP or PLACING
        #for the recorder to finish broadcasting
        if self.pause_for_recording and (phase == ManipulationPhase.MOVING_TO_GRASP \
                or phase == ManipulationPhase.PLACING):
            time.sleep(3)

class ReactiveGraspActionServer(object):
    _grasp_result = ReactiveGraspResult()
    _lift_result = ReactiveLiftResult()
    _place_result = ReactivePlaceResult()

    def __init__(self, which_arm):
        """
        Class for the reactive grasp actions and services.

        @which_arm is 'r' or 'l'
        """
        self.which_arm = which_arm

        self._node_name = which_arm+'_reactive_grasp'
        if which_arm == 'r':
            self._action_name = 'reactive_grasp/right'
            self._lift_action_name = 'reactive_lift/right'
            self._approach_action_name = 'reactive_approach/right'
            self._place_action_name = 'reactive_place/right'
        else:
            self._action_name = 'reactive_grasp/left'
            self._lift_action_name = 'reactive_lift/left'
            self._approach_action_name = 'reactive_approach/left'
            self._place_action_name = 'reactive_place/left'

        #params for reactive approach/grasp
        self.side_step = rospy.get_param("~side_step", .015)
        self.back_step = rospy.get_param("~back_step", .03)
        self.forward_step = rospy.get_param("~forward_step", 0)  #select based on top/side
        self.approach_num_tries = rospy.get_param("~approach_num_tries", 5)
        self.goal_pos_thres = rospy.get_param("~goal_pos_thres", .01)

        #params for reactive grasp/grasp adjustment
        self.grasp_adjust_x_step = rospy.get_param("~grasp_adjust_x_step", .02)
        self.grasp_adjust_z_step = rospy.get_param("~grasp_adjust_z_step", .015)
        self.grasp_adjust_num_tries = rospy.get_param("~grasp_adjust_num_tries", 3)

        #params for reactive grasp 
        self.grasp_num_tries = rospy.get_param("~grasp_num_tries", 2)
        self.close_force = rospy.get_param("~close_force", 100)
        self.use_slip_detection = rospy.get_param("~use_slip_detection", 0)
        rospy.loginfo("reactive_grasp_server: using_slip_detection:"+str(self.use_slip_detection))

        #params for reactive place
        self.place_overshoot = rospy.get_param("~place_overshoot", .01)
        rospy.loginfo("place_overshoot: %f"%self.place_overshoot)

        #start action servers for reactive grasp, lift, and approach
        self._as = actionlib.SimpleActionServer(self._action_name, ReactiveGraspAction, \
                                                    execute_cb=self.reactive_grasp_cb)
        self._lift_as = actionlib.SimpleActionServer(self._lift_action_name, ReactiveLiftAction, \
                                                         execute_cb=self.reactive_lift_cb)
        self._approach_as = actionlib.SimpleActionServer(self._approach_action_name, ReactiveGraspAction, \
                                                             execute_cb = self.reactive_approach_cb)
        self._place_as = actionlib.SimpleActionServer(self._place_action_name, ReactivePlaceAction, \
                                                          execute_cb = self.reactive_place_cb)

        #advertise services for compliant grasp and grasp adjustment
        rospy.Service(self._node_name+'/compliant_close', \
                          Empty, self.compliant_close_callback)
        rospy.Service(self._node_name+'/grasp_adjustment', \
                          Empty, self.grasp_adjustment_callback)
        
        self.rg = ReactiveGrasperWithPreempt(self._as, self._lift_as, self._approach_as, self._place_as, which_arm, self.use_slip_detection )


    def reactive_grasp_cb(self, goal):
        """
        do a reactive grasp using the fingertip sensors
        (backs up and moves to the side if a fingertip contacts on the way to the grasp,
        closes compliantly, tries approach and grasp again if gripper opening is not within bounds)
        """

        rospy.loginfo("got reactive grasp request")
        try:
            self.rg_state = "rg"
            (trajectory, object_name, table_name, forward_step) = self.init_reactive_grasp(goal)

            #perform the reactive grasp
            #self.cm.switch_to_cartesian_mode()
            result = self.rg.reactive_grasp(None, goal.final_grasp_pose, trajectory, 
                                            self.side_step, self.back_step, 
                                            self.approach_num_tries, self.goal_pos_thres,
                                            self.grasp_num_tries, forward_step, 
                                            object_name, table_name,
                                            self.grasp_adjust_x_step, self.grasp_adjust_z_step, self.grasp_adjust_num_tries)
            self.rg.check_preempt()

            #rospy.loginfo("switching back to joint controllers")
            #self.cm.switch_to_joint_mode()
            self.rg_state = "off"

            if result == 0:
                self.rg.broadcast_phase(ManipulationPhase.SUCCEEDED, send_feedback = 0)
                self._grasp_result.manipulation_result.value = ManipulationResult.SUCCESS
                self._as.set_succeeded(self._grasp_result)
            else:
                self.rg.broadcast_phase(ManipulationPhase.FAILED, send_feedback = 0)
                self._grasp_result.manipulation_result.value = ManipulationResult.FAILED
                self._as.set_aborted(self._grasp_result)

        except Preempted:
            rospy.loginfo("reactive grasp preempted, returning")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._grasp_result.manipulation_result.value = ManipulationResult.FAILED       
            self._as.set_preempted(self._grasp_result)

        except reactive_grasp.Aborted:
            rospy.loginfo("reactive grasp saw a serious error!  Aborting")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._grasp_result.manipulation_result.value = ManipulationResult.ERROR
            self._as.set_aborted(self._grasp_result)



    ##pull relevant parts out of the goal for both reactive grasp and approach
    def init_reactive_grasp(self, goal):

        self.rg.check_preempt()

        #pull trajectory angles out of JointTrajectory
        if len(goal.trajectory.points) > 0:
            trajectory = [goal.trajectory.points[i].positions for i in range(len(goal.trajectory.points))]
            print "trajectory:"
            for angles in trajectory:
                print pplist(angles)
        else:
            trajectory = None

        object_name = "points"
        table_name = goal.collision_support_surface_name

        #if forward_step is 0, adjust the forward_step depending on whether the grasp is a top or side grasp
        #if the palm's x-axis's z-component is high enough, it's a top grasp
        if self.forward_step == 0:
            #transform pose to base_link
            forward_step = .03
        else:
            forward_step = self.forward_step

        self.rg.check_preempt()

        return (trajectory, object_name, table_name, forward_step)



    ##do a reactive approach using the fingertip sensors
    #(backs up and moves to the side if a fingertip contacts on the way to the grasp)
    def reactive_approach_cb(self, goal):

        rospy.loginfo("got reactive approach request")
        try:
            self.rg_state = "approach"

            rospy.logerr("NOT IMPLEMENTED YET")

            self.rg_state = "off"
            self.rg.check_preempt()

            if result == 0:
                self.rg.broadcast_phase(ManipulationPhase.SUCCEEDED, send_feedback = 0)
                self._grasp_result.manipulation_result.value = ManipulationResult.SUCCESS
                self._approach_as.set_succeeded(self._grasp_result)
            else:
                self.rg.broadcast_phase(ManipulationPhase.FAILED, send_feedback = 0)
                self._grasp_result.manipulation_result.value = ManipulationResult.FAILED
                self._approach_as.set_aborted(self._grasp_result)

        except Preempted:
            rospy.loginfo("reactive grasp preempted, returning")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._grasp_result.manipulation_result.value = ManipulationResult.FAILED       
            self._approach_as.set_preempted(self._grasp_result)

        except reactive_grasp.Aborted:
            rospy.loginfo("reactive grasp saw a serious error!  Aborting")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._grasp_result.manipulation_result.value = ManipulationResult.ERROR
            self._approach_as.set_aborted(self._grasp_result)

    ##do a reactive lift using the fingertip sensors (uses slip-servoing to exert just enough force to grasp and lift)
    def reactive_lift_cb(self, goal):

        try:
            rospy.loginfo("got reactive lift request")
            
            rospy.logerr("NOT IMPLEMENTED YET")

            self.rg_state = "off"

            if success:
                self.rg.broadcast_phase(ManipulationPhase.SUCCEEDED, send_feedback = 0)
                self._lift_result.manipulation_result.value = ManipulationResult.SUCCESS
                self._lift_as.set_succeeded(self._lift_result)
            else:
                self.rg.broadcast_phase(ManipulationPhase.FAILED, send_feedback = 0)
                self._lift_result.manipulation_result.value = ManipulationResult.FAILED
                self._lift_as.set_aborted(self._lift_result)
            
        except Preempted:
            rospy.loginfo("reactive lift preempted, returning")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._lift_result.manipulation_result.value = ManipulationResult.FAILED       
            self._lift_as.set_preempted(self._lift_result)

        except reactive_grasp.Aborted:
            rospy.loginfo("reactive lift saw a serious error!  Aborting")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._lift_result.manipulation_result.value = ManipulationResult.ERROR
            self._lift_as.set_aborted(self._lift_result)


    ##do a reactive place using the fingertip sensors and accelerometer 
    #(uses slip controller to stop and open when the object hits the table)
    #if slip controller is not running, just goes past the goal with the Cartesian controller
    def reactive_place_cb(self, goal):
        
        try:
            rospy.loginfo("got reactive place request")

            rospy.logerr("NOT IMPLEMENTED YET")

            #place and open when the object hits the table
            self.rg_state = "place"

            self._place_result.manipulation_result.value = ManipulationResult.SUCCESS
            if result == 1:
                self.rg.broadcast_phase(ManipulationPhase.SUCCEEDED, send_feedback = 0)
            self._place_as.set_succeeded(self._place_result)

        except Preempted:
            rospy.loginfo("reactive place preempted, returning")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._place_result.manipulation_result.value = ManipulationResult.FAILED
            self._place_as.set_preempted(self._place_result)
        
        except reactive_grasp.Aborted:
            rospy.loginfo("reactive place saw a serious error!  Aborting")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._place_result.manipulation_result.value = ManipulationResult.ERROR
            self._place_as.set_aborted(self._place_result)

    ##do a compliant close using the fingertip sensors (stops at the first detected 
    #contact and moves the arm so that the contact stays in place)
    def compliant_close_callback(self, req):
        rospy.loginfo("executing compliant grasp--switching to Cartesian controllers")

        rospy.logerr("NOT IMPLEMENTED YET")
        
        return EmptyResponse()


    ##do a grasp adjustment using the fingertip sensors (tries to move the hand to
    #obtain non-tip and non-edge contacts)
    def grasp_adjustment_callback(self, req):
        rospy.loginfo("executing grasp adjustment--switching to Cartesian controllers")

        rospy.logerr("NOT IMPLEMENTED YET")
        
        return EmptyResponse()


if __name__ == "__main__":
    
    if len(sys.argv) < 2 or sys.argv[1] != 'r' and sys.argv[1] != 'l':
        rospy.logerr("usage: reactive_grasp_server.py which_arm (which_arm is r or l)")
        sys.exit(1)

    which_arm = sys.argv[1]

    node_name = which_arm+'_reactive_grasp'
    rospy.init_node(node_name, anonymous=True)

    reactive_grasp_services = ReactiveGraspActionServer(which_arm)
    rospy.loginfo("Reactive grasp action ready")

    rospy.spin()
