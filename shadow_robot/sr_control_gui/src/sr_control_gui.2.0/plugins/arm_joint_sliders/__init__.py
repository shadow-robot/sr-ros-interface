import os, sys
sys.path.append(os.getcwd() + "/plugins")
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
        self.set_icon('images/icons/iconArm.png')
        
    def sendupdate(self, dict):
        self.parent.parent.libraries["sr_library"].sendupdate_arm_from_dict(dict)
