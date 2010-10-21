import os, sys
sys.path.append(os.getcwd() + "/plugins")
from joint_slider import Joint, JointSlider
  
class HandJointSlider(JointSlider):  
    name = "Hand Joint Sliders"
        
    def __init__(self):
        joints_list = []
        
        joints_list.append(Joint("FFJ0", 0, 180))
        joints_list.append(Joint("FFJ3"))
        joints_list.append(Joint("FFJ4", -25, 25))
        
        joints_list.append(Joint("MFJ0", 0, 180))
        joints_list.append(Joint("MFJ3"))
        joints_list.append(Joint("MFJ4", -25, 25))
        
        joints_list.append(Joint("RFJ0", 0, 180))
        joints_list.append(Joint("RFJ3"))
        joints_list.append(Joint("RFJ4", -25, 25))
        
        joints_list.append(Joint("LFJ0", 0, 180))
        joints_list.append(Joint("LFJ3"))
        joints_list.append(Joint("LFJ4", -25, 25))
        joints_list.append(Joint("LFJ5", 0, 45))
        
        joints_list.append(Joint("THJ1"))
        joints_list.append(Joint("THJ2", -30, 30))
        joints_list.append(Joint("THJ3", -15, 15))
        joints_list.append(Joint("THJ4", 0, 70))
        joints_list.append(Joint("THJ5", -60, 60))
        
        JointSlider.__init__(self, joints_list)
        self.set_icon('images/icons/iconHand.png')

    def sendupdate(self, dict):
        self.parent.parent.libraries["sr_library"].sendupdate_from_dict(dict)
