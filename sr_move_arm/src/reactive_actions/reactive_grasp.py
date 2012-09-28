#!/usr/bin/env python

import roslib; roslib.load_manifest("sr_move_arm")
import rospy

import tf

import scipy, time
from object_manipulator.convert_functions import *
from object_manipulation_msgs.msg import GripperTranslation, ManipulationPhase
from sr_robot_msgs.msg import sendupdate, joint
from sr_robot_msgs.srv import which_fingers_are_touching
from trajectory_msgs.msg import JointTrajectory
from actionlib import SimpleActionClient
#from motion_planning_msgs.msg import *
from arm_navigation_msgs.msg import *
#from geometric_shapes_msgs.msg import Shape
from actionlib_msgs.msg import GoalStatus

##abort exception
class Aborted(Exception): pass

class ReactiveGrasper(object):
    """
    reactive/guarded movement and grasping
    """
    def __init__(self, which_arm = 'r', slip_detection = False):
        self.using_slip_detection = slip_detection
        self.whicharm = which_arm

	self.mypub = rospy.Publisher("/command",JointTrajectory)

        #force to use when closing hard
        self.close_force = 100
        if self.using_slip_detection:
            self.close_force = 10

        #has the hand been closed hard recently?
        self._closed_hard = 0

        #collision name of support surface
        self._table_name = "table"

        #root and tip name for listening
        #to the current palm pose
        self.tf_listener = tf.TransformListener()
        self.root_name = "world"
        self.tip_name = "palm"

        #for taking photos of each step
        self._photo = 0

        #broadcast the current phase of the manipulation
        self._phase_pub = rospy.Publisher('/reactive_manipulation_phase', ManipulationPhase)

        #send targets to the arm / hand
        self.sr_hand_target_pub = rospy.Publisher('/srh/sendupdate', sendupdate)
        self.sr_arm_target_pub = rospy.Publisher('/sr_arm/sendupdate', sendupdate)
        self.move_arm_client = None

        if which_arm == 'r':
            self.move_arm_client = SimpleActionClient( "/move_right_arm", MoveArmAction )
        else:
            self.move_arm_client = SimpleActionClient( "/move_left_arm", MoveArmAction )
        self.move_arm_client.wait_for_server()

        #client to the tactile sensor manager.
        rospy.loginfo("Waiting for service which_fingers_are_touching")
        rospy.wait_for_service('which_fingers_are_touching')
        rospy.loginfo("OK service which_fingers_are_touching found.")
        self.which_fingers_are_touching_client = rospy.ServiceProxy('which_fingers_are_touching', which_fingers_are_touching)

        #load tactile thresholds
        self.light_touch_thresholds = rospy.get_param('light_touch_thresholds', [100.,100.,100.,100.,0.0])
        rospy.loginfo("light and grasp threashold are :")
	rospy.loginfo(self.light_touch_thresholds)
        self.grasp_touch_thresholds = rospy.get_param('grasp_touch_thresholds', [117.,117.,113.,111.,0.0])
        rospy.loginfo(self.grasp_touch_thresholds)

        #dictionary for ManipulationPhase
        self.manipulation_phase_dict = {}
        for element in dir(ManipulationPhase):
            if element[0].isupper():
                self.manipulation_phase_dict[eval('ManipulationPhase.'+element)] = element

        #dictionary of return values for reactive approach
        self.reactive_approach_result_dict = {"success":0, "within goal threshold":1, "both fingers touching":2, "ran out of tries":3,
                                              "touching table":4, "saw palm contact":5, "grasp infeasible":6}

        #dictionary of return values for reactive_grasp
        self.reactive_grasp_result_dict = {"success":0, "ran out of grasp tries":1, "ran out of approach tries":2,
                                           "aborted":3, "grasp infeasible":4}

        rospy.logdebug("done with ReactiveGrasper init for the %s arm"%self.whicharm)

    def broadcast_phase(self, phase):
        """
        broadcast the current manipulation phase (of type ManipulationPhase)
        """
        rospy.loginfo("broadcasting reactive phase %s"%self.manipulation_phase_dict[phase])
        self._phase_pub.publish(phase)


    def reactive_grasp(self, approach_pose, grasp_pose, joint_path = None, pregrasp_posture = None, grasp_posture = None, side_step = .015, back_step = .03,
                       approach_num_tries = 10, goal_pos_thres = 0.01, grasp_num_tries = 4,
                       forward_step = 0.03, object_name = "points", table_name = "table",
                       grasp_adjust_x_step = .02, grasp_adjust_z_step = .015, grasp_adjust_num_tries = 3):
        self._table_name = table_name

        self.check_preempt()

        #if approach_pose is not specified, use the current pose of the hand
        if approach_pose == None:
            (current_trans,current_rot) = self.tf_listener.lookupTransform(self.root_name, self.tip_name, rospy.Time(0))
            approach_pose = create_pose_stamped(current_trans+current_rot)

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

            #go to the pregrasp posture
            rospy.loginfo("Going to pregrasp posture.")
            self.open_and_reset_fingertips(pregrasp_posture, reset = 1)
            #then start closing the hand compliantly
            rospy.loginfo("starting compliant_close")
            self.compliant_close(grasp_posture, pregrasp_posture)

            self.check_preempt()

            #check if the grasp is okay
            if self.check_good_grasp():
                rospy.loginfo("Grasp successfully achieved")
                return self.reactive_grasp_result_dict["success"]

            #if the grasp is not okay
            num_tries += 1
            if num_tries < grasp_num_tries:
                rospy.logwarn("Failed to grasp: trying once more")
                #move the goal forward along the approach direction by forward_step
                move_forward_vect = approach_dir*forward_step
                current_goal_pose = self.return_rel_pose(move_forward_vect, self.root_name, current_goal_pose)

                joint_path = None

                #print the new goal
                (goal_pos, goal_rot) = pose_stamped_to_lists(self.tf_listener, current_goal_pose, self.root_name)
                rospy.loginfo("trying approach again with new goal, pos: "+pplist(goal_pos)+" rot: "+pplist(goal_rot))

                #open the gripper back up
                self.open_and_reset_fingertips(pregrasp_posture, reset = 1)

            else:
                rospy.loginfo("ran out of grasp tries!")
                return self.reactive_grasp_result_dict["ran out of grasp tries"]


    def reactive_approach(self, approach_dir, current_goal_pose, joint_path = None,
                          side_step = .015, back_step = .03,
                          num_tries = 10, goal_pos_thres = 0.01):
        """
        Stops and goes back a little in case of a contact.
        """

        rospy.loginfo("Executing a reactive approach")
        self.broadcast_phase(ManipulationPhase.MOVING_TO_GRASP)

        #check the position of the palm compared to the position of the object
        #adjust if necessary

        current_sleep_time = rospy.Duration(0.0)
        last_time = rospy.Duration(0.0)
        #follow the joint_path if one is given
        if joint_path != None:
            self.check_preempt()
	    	    
	    self.mypub.publish(joint_path)
	    rospy.logerr("Sent a trajectory !")
	    rospy.sleep(10)
	    rospy.logerr("Trajectory should be finished !") 
            '''joint_names = joint_path.joint_names
	    countTime = 5

            for trajectory_step in joint_path.points:
                sendupdate_msg = []

                for (joint_name, joint_target) in zip(joint_names, trajectory_step.positions):
                    # convert targets to degrees
                    sendupdate_msg.append(joint(joint_name = joint_name, joint_target = joint_target * 57.325))

                self.sr_arm_target_pub.publish(sendupdate(len(sendupdate_msg), sendupdate_msg) )
                self.sr_hand_target_pub.publish(sendupdate(len(sendupdate_msg), sendupdate_msg) )

                current_sleep_time = trajectory_step.time_from_start - last_time
                rospy.sleep( countTime ) # current_sleep_time )
		countTime=max(countTime-1,0.5)
                last_time = trajectory_step.time_from_start
        else:
            #if no joint_path is given, go to the current_goal_pose
            self.command_cartesian( current_goal_pose )'''

        return self.reactive_approach_result_dict["success"]


    def reactive_lift(self, goal):
        """
        Lifts the object while monitoring for contacts.
        """
        #follow the joint_path if one is given
        joint_path = goal.trajectory
        last_time = rospy.Duration(0.0)
        current_sleep_time = rospy.Duration(0.0)

        if joint_path != None:
            self.check_preempt()
            joint_names = joint_path.joint_names

            for trajectory_step in joint_path.points:
                sendupdate_msg = []

                for (joint_name, joint_target) in zip(joint_names, trajectory_step.positions):
                    # convert targets to degrees
                    sendupdate_msg.append(joint(joint_name = joint_name, joint_target = joint_target * 57.325))

                self.sr_arm_target_pub.publish(sendupdate(len(sendupdate_msg), sendupdate_msg) )
                self.sr_hand_target_pub.publish(sendupdate(len(sendupdate_msg), sendupdate_msg) )

                current_sleep_time = trajectory_step.time_from_start - last_time
                rospy.sleep( current_sleep_time )
                last_time = trajectory_step.time_from_start

        else:
            #if no joint_path is given, go to the current_goal_pose
            rospy.logerr("TODO: Implement this. Get current pose, add the lift and set this as the command_cartesian target.")
            self.command_cartesian( goal.lift )

        return self.reactive_approach_result_dict["success"]

    def reactive_place(self, goal):
        """
        Lifts the object while monitoring for contacts.
        """
        #follow the joint_path if one is given
        joint_path = goal.trajectory
        last_time = rospy.Duration(0.0)
        current_sleep_time = rospy.Duration(0.0)

        if joint_path != None:
            self.check_preempt()
            joint_names = joint_path.joint_names

            for trajectory_step in joint_path.points:
                sendupdate_msg = []

                for (joint_name, joint_target) in zip(joint_names, trajectory_step.positions):
                    # convert targets to degrees
                    sendupdate_msg.append(joint(joint_name = joint_name, joint_target = joint_target * 57.325))

                self.sr_arm_target_pub.publish(sendupdate(len(sendupdate_msg), sendupdate_msg) )
                self.sr_hand_target_pub.publish(sendupdate(len(sendupdate_msg), sendupdate_msg) )

                current_sleep_time = trajectory_step.time_from_start - last_time
                rospy.sleep( current_sleep_time )
                last_time = trajectory_step.time_from_start

        else:
            #if no joint_path is given, try to go to the final_place_pose directly
            self.command_cartesian( goal.final_place_pose )

        return self.reactive_approach_result_dict["success"]


    def compliant_close(self, grasp_posture = None, pregrasp_posture = None):
        """
        Close compliantly. We're going to interpolate from pregrasp to grasp, sending
        data each iteration_time. We stop the fingers when they come in contact with something
        using the feedback from the tactile sensors.

        @grasp_posture: the posture of the hand for the grasp - joint names and positions
        @pregrasp_posture: the posture of the hand when doing the approach. We assume
                           that the grasp and pregrasp have the same joint order.
        @nb_steps: the number of steps used in the interpolation.
        @iteration_time: how long do we wait between 2 steps.
        """
        rospy.loginfo("Closing the hand compliantly")
        self.check_preempt()
        self.broadcast_phase(ManipulationPhase.CLOSING)
        use_slip_controller_to_close = self.using_slip_detection

        if grasp_posture == None:
            rospy.logerr("No grasp posture given, aborting")
            return

        if pregrasp_posture == None:
            rospy.logerr("No pregrasp posture given, aborting - FIXME")
            #doesn't need to abort here, could read current position from the hand and take
            #those as the pregrasp.
            return

        #TODO: good compliant close - someone from HANDLE?
        #   Counter-move the arm to keep the touching fingers in-place while closing?
        joint_names = grasp_posture.name
        grasp_targets = grasp_posture.position
        pregrasp_targets = pregrasp_posture.position
        #QUESTION: Should we make sure grasp and pregrasp have the same order?

        current_targets = self.close_until_force(joint_names, pregrasp_targets, grasp_targets, self.light_touch_thresholds)

        #now that all the fingers are touching the object, we're going to close
        #with a stronger grasp.
        rospy.sleep(1)
        rospy.loginfo("Now closing with stronger grasp.")

        current_targets = self.close_until_force(joint_names, current_targets, grasp_targets, self.grasp_touch_thresholds, "All the fingers are now grasping the object strongly.")

        rospy.logwarn("Waiting a little after closing the hand")
        rospy.sleep(1)

    def close_until_force(self, joint_names, pregrasp, grasp, forces_threshold, success_msg = "All the fingers are now touching the object.", nb_steps = 20, iteration_time=0.2):
        """
        Closes the hand from pregrasp to grasp
        until the given forces are reached.

        @return returns the positions reached when the forces were reached.
        """
        current_targets = []
        for pos in pregrasp:
            current_targets.append(pos)

        #loop on all the iteration steps. The target for a given finger
        # is :
        #      target = pregrasp + (grasp - pregrasp) * (i / TOTAL_NB_OF_STEPS)
        fingers_touching = [0,0,0,0,0]

        for i_step in range(0, nb_steps + 1):
            sendupdate_msg = []
            fingers_touch_index = {"FF":0, "MF":1, "RF":2, "LF":3, "TH":4}
            for (index, joint_name, grasp_target, pregrasp_target) in zip(range(0,len(joint_names)),joint_names, grasp, pregrasp):
                # only update the finger that are not touching
                # takes the first 2 letters of the names (= finger name)
                # and check the index from the dict defined before the for loop.
                # return -1 if the finger is not in the dict
                touch_index = fingers_touch_index.get(joint_name[:2], -1)
                if touch_index == -1:
                    touching = 0
                else:
                    touching = fingers_touching[touch_index]
                if touching == 0:
                    joint_target = pregrasp_target + float(grasp_target - pregrasp_target)*(float(i_step) / float(nb_steps) )
                    myjoint_target = joint_target * 180.0 / math.pi
                    current_targets[index] = joint_target
                    sendupdate_msg.append(joint(joint_name = joint_name, joint_target = myjoint_target))
                    rospy.logdebug("["+joint_name+"]: (p/g/t) = "+str(pregrasp_target)+"/"+str(grasp_target)+"/"+str(myjoint_target) + " ("+
                                   str(float(i_step) / float(nb_steps))+"%)")

            self.sr_hand_target_pub.publish(sendupdate(len(sendupdate_msg), sendupdate_msg) )
            rospy.sleep(iteration_time)
            #check which fingers are touching
            which_fingers_are_touching_response = []
            try:
                which_fingers_are_touching_response = self.which_fingers_are_touching_client(forces_threshold)
            except rospy.ServiceException, e:
                rospy.logerr("Couldn't call the service which_fingers_are_touching")

            #check if all the fingers are touching
            fingers_touching = which_fingers_are_touching_response.touch_forces
            all_fingers_touching = True

            #we don't check the thumb as the dummy tactile sensors are not
            # working properly for the thumb
            for value in fingers_touching[:4]:
                if value == 0.0:
                    all_fingers_touching = False
                    break
            #stop when all the fingers are touching the object
            if all_fingers_touching:
                rospy.loginfo(success_msg)
                break

        return current_targets

    def check_good_grasp(self):
        """
        Check if we have a good grasp on the object.
        """
        rospy.logerr("TODO: really check if we have a good grasp - shake the object maybe?")

        return True

    def compute_approach_dir(self, approach_pose, grasp_pose):
        """
        compute (unit) approach direction in base_link frame from an approach_pose and grasp_pose (returns scipy array)
        """
        approach_vect = self.compute_approach_vect(approach_pose, grasp_pose)
        approach_dir = approach_vect/scipy.dot(approach_vect, approach_vect)**.5
        return approach_dir

    def compute_approach_vect(self, approach_pose, grasp_pose):
        """
        compute approach vector in base_link frame from an approach_pose and grasp_pose (returns scipy array)
        """
        (approach_pos, approach_rot) = pose_stamped_to_lists(self.tf_listener, approach_pose, self.root_name)
        (grasp_pos, grasp_rot) = pose_stamped_to_lists(self.tf_listener, grasp_pose, self.root_name)
        approach_vect = scipy.array(grasp_pos) - scipy.array(approach_pos)
        return approach_vect

    def return_rel_pose(self, vector, frame, start_pose = None, orthogonal_to_vect = None, orthogonal_to_vect_frame = 'world'):
        """
        convert a relative vector in frame to a pose in the base_link frame
        if start_pose is not specified, uses current pose of the wrist
        if orthogonal_to_vect and orthogonal_to_vect_frame are specified, first convert vector to be orthogonal to orthogonal_to_vect
        """

        #if start_pose is not specified, use the current Cartesian pose of the wrist
        if start_pose == None:
            (start_trans, start_rot) =  pose_stamped_to_lists(self.tf_listener, self.tip_name, self.root_name)
        else:
            start_pose.header.stamp = rospy.Time(0)
            (start_trans, start_rot) = pose_stamped_to_lists(self.tf_listener, start_pose, self.root_name)

        #convert the vector in frame to base_link frame
        if frame != self.root_name:
            vector3_stamped = create_vector3_stamped(vector, frame)
            base_link_vector = vector3_stamped_to_list(self.tf_listener, vector3_stamped, self.root_name)
        else:
            base_link_vector = vector

        #if orthogonal_to_vect and orthogonal_to_vect_frame are specified, make the vector orthogonal to that vector
        if orthogonal_to_vect != None:
            #first convert it to the base_link frame if it's not already
            if orthogonal_to_vect_frame != self.root_name:
                ortho_vector3_stamped = create_vector3_stamped(orthogonal_to_vect, orthogonal_to_vect_frame)
                ortho_base_link_vector = vector3_stamped_to_list(self.tf_listener, ortho_vector3_stamped, self.root_name)
            else:
                ortho_base_link_vector = orthogonal_to_vect

            #now project the desired vector onto the orthogonal vector and subtract out that component
            base_link_vector = self.make_orthogonal(base_link_vector, ortho_base_link_vector)

        #add the desired vector to the position
        new_trans = [x+y for (x,y) in zip(start_trans, base_link_vector)]
        #create a new poseStamped
        pose_stamped = create_pose_stamped(new_trans+start_rot)

        return pose_stamped

    def open_and_reset_fingertips(self, pregrasp_posture, reset = 0):
        """
        Open the hand
        """
        joint_names = pregrasp_posture.name
        joint_targets = pregrasp_posture.position
        sendupdate_msg = []

        for (joint_name, joint_target) in zip(joint_names, joint_targets):
            myjoint_target=joint_target * 180 / math.pi
            sendupdate_msg.append(joint(joint_name = joint_name, joint_target = myjoint_target))

        self.sr_hand_target_pub.publish(sendupdate(len(sendupdate_msg), sendupdate_msg) )

    def command_cartesian(self, pose, frame_id='world'):
        """
        Tries to go to the given pose with the arm.
        """
        goalA = MoveArmGoal()
        if self.whicharm == "r":
            goalA.motion_plan_request.group_name = "right_arm"
        else:
            goalA.motion_plan_request.group_name = "left_arm"

        goalA.motion_plan_request.num_planning_attempts = 1
        goalA.motion_plan_request.planner_id = ""
        goalA.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)


        pc = PositionConstraint()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = self.root_name
        pc.link_name = self.tip_name
        pc.position = pose.pose.position

        pc.constraint_region_shape.type = Shape.BOX
        pc.constraint_region_shape.dimensions = [0.02, 0.02, 0.02]
        pc.constraint_region_orientation.w = 1.0

        goalA.motion_plan_request.goal_constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.stamp = rospy.Time.now()
        oc.header.frame_id = self.root_name
        oc.link_name = self.tip_name
        oc.orientation =  pose.pose.orientation

        oc.absolute_roll_tolerance = 0.04
        oc.absolute_pitch_tolerance = 0.04
        oc.absolute_yaw_tolerance = 0.04
        oc.weight = 1.

        goalA.motion_plan_request.goal_constraints.orientation_constraints.append(oc)

        self.move_arm_client.send_goal(goalA)
        finished_within_time = False
        finished_within_time = self.move_arm_client.wait_for_result()

        if not finished_within_time:
            self.move_arm_client.cancel_goal()
            rospy.logout('Timed out achieving goal A')
        else:
            state = self.move_arm_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.logout('Action finished with SUCCESS')
            else:
                rospy.logout('Action failed')

    def check_preempt(self):
        """
        Needs to be overloaded
        """
        pass
