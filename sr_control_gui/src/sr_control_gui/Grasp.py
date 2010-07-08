VERBOSE = 1

class Grasp:
	def __init__(self):
		self.grasp_name = ""
		self.joints_and_positions = {}

	def display_grasp(self):
		print self.grasp_name
		print self.joints_and_positions
		
	def convert_to_xml(self):
		grasp = '	<grasp name="'+self.grasp_name+'">'+'\n'
		for key, value in self.joints_and_positions.items():
			grasp = grasp + '		<joint name="'+str(key)+'">'+str(value)+'</joint>\n'
		grasp = grasp + '	</grasp>'+'\n'
		return grasp	
