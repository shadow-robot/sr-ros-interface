from Grasp import Grasp

class GraspInterpoler:
	def __init__(self, grasp_from, grasp_to="current"):
		"""
		interpolate from one grasp to another

		Keyword arguments
		grasp_from -- goes from this grasp
		grasp_to -- to this grasp (if not defined, then consider
		the current position of the hand as a grasp)
		"""
		self.percentage = 0
		self.grasp_from = grasp_from
		self.grasp_to = grasp_to

		# store the joints in common between the 2 grasps, with a table
		# [ starting_value, end_value ] 
		self.from_grasp1_to_grasp2 = {}

		for joint in self.grasp_from.joints_and_positions.keys():
            #if joint in self.grasp_to.joints_and_positions.keys():
			self.from_grasp1_to_grasp2.update({joint: [self.grasp_from.joints_and_positions.get(joint),self.grasp_to.joints_and_positions.get(joint)]})
        
	def set_percentage_grasp1_to_grasp2(self, percentage):
		"""
		move to a given percentage between grasp1 and grasp2

		Keyword arguments
		percentage -- must be between 0 and 100
		"""
		self.percentage = percentage
		positions_to_send = {}

		for joint in self.from_grasp1_to_grasp2.iteritems():
			new_target = percentage/100.0 * (joint[1][1] - joint[1][0]) 
			positions_to_send.update({joint[0]:new_target})

		return positions_to_send
               
	def interpolate(self, percentage):
		position_to_send = {}
		for name, position in self.grasp_to.joints_and_positions.items():
			if name in self.grasp_from.joints_and_positions.keys():
				pos_from = self.grasp_from.joints_and_positions[name]
				pos_to = position
				position_to_send[name] = pos_from + (pos_to-pos_from)*(percentage/100.0)
		return position_to_send
