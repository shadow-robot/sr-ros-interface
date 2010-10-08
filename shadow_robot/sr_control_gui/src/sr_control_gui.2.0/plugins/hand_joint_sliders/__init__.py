import os, sys
sys.path.append(os.getcwd() + "/plugins")
from generic_plugin import GenericPlugin
from joint_slider import Joint, JointSlider
  
class HandJointSlider(JointSlider):  
    name = "Hand Joint Sliders"
        
    def __init__(self):
        joints_map = {}
        
        joints_map["FFJ0"] = Joint("FFJ0")
        joints_map["FFJ3"] = Joint("FFJ3")
        joints_map["FFJ4"] = Joint("FFJ4", -25, 25)
        
        joints_map["MFJ0"] = Joint("MFJ0")
        joints_map["MFJ3"] = Joint("MFJ3")
        joints_map["MFJ4"] = Joint("MFJ4", -25, 25)
        
        joints_map["RFJ0"] = Joint("RFJ0")
        joints_map["RFJ3"] = Joint("RFJ3")
        joints_map["RFJ4"] = Joint("RFJ4", -25, 25)
        
        joints_map["LFJ0"] = Joint("LFJ0")
        joints_map["LFJ3"] = Joint("LFJ3")
        joints_map["LFJ4"] = Joint("LFJ4", -25, 25)
        joints_map["LFJ5"] = Joint("LFJ5", 0, 45)
        
        joints_map["THJ1"] = Joint("THJ1")
        joints_map["THJ2"] = Joint("THJ2", -30, 30)
        joints_map["THJ3"] = Joint("THJ3", -15, 15)
        joints_map["THJ4"] = Joint("THJ4", 0, 70)
        joints_map["THJ5"] = Joint("THJ5", -60, 60)
        
        JointSlider.__init__(self, joints_map)
