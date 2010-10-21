import os, sys

#Not very pretty....
import subprocess
process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
rootPath = process.communicate()[0]
rootPath = rootPath.split('\n')
rootPath = rootPath[0]
sys.path.append(rootPath+ "/src/sr_control_gui/plugins")

from joint_slider import Joint, JointSlider
  

class ArmJointSlider(JointSlider):  
    name = "Arm Joint Sliders"
        
    def __init__(self):        
        joints_list = [Joint("trunk_rotation", -45, 90),
                       Joint("shoulder_rotation", 0, 90),
                       Joint("elbow_abduction", 0, 120),
                       Joint("forearm_rotation", -90, 90)
                       ]
        
        JointSlider.__init__(self, joints_list)
        
        
    def sendupdate(self, dict):
        self.parent.parent.libraries["sr_library"].sendupdate_arm_from_dict(dict)
        self.set_icon(self.parent.parent.rootPath + '/src/sr_control_gui/images/icons/iconArm.png')
