#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
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
#
"""
Calibration utility for the cyberglove.

@author: Ugo Cupcic
@contact: ugo@shadowrobot.com, contact@shadowrobot.com
"""
import roslib; roslib.load_manifest('sr_control_gui')

import os
from cyberglove_library import Cyberglove
from cyberglove.srv import Calibration as CalibrationSrv

import rospy

class Joint:
    """
    A class containing simple information regarding a joint: name, and calibrated value
    """
    def __init__(self, name, min=0, max = 90):
        """
        @param name: the name of the joint
        @param min: the min value to calibrate
        @param max: the max value to calibrate   
        """
        self.name  = name
        self.min = min
        self.max = max
        

class CalibrationStep:
    """
    A class containing all the data for a given calibration step
    """
    def __init__(self, step_name="", step_description = [""], joints = []):
        """
        @param step_name: A short name for this calibration step 
        @param step_description: A table containing 2 descriptions: the first one describes the min pose, the second 
        the max
        @param joints: A table containing the joints to be calibrated 
        
        """    
        self.step_name    = step_name
        self.step_description = step_description
        self.joints = joints
        
class Calibration:
    """
    A class Containing all the calibration data for a given joint.
    """
    def __init__(self, raw_min = 0.0, raw_max= 0.0, 
                 calibrated_min = 0.0, calibrated_max= 0.0,
                 is_calibrated = 0):
        self.raw_min        = raw_min
        self.raw_max        = raw_max
        self.calibrated_min = calibrated_min
        self.calibrated_max = calibrated_max
        self.is_calibrated  = is_calibrated
    
def default_description(step_name, max = 0):
    """
    The default description function for a step. Just prints a text for each step.
    
    @param step_name: the name of the step
    @param max: if 0=>we're reading the min values, if 1=> max values
    """
    if max == 0:
        print "calibrating min values for: "+ str(step_name)
    else:        
        print "calibrating max values for: "+ str(step_name)
        
def do_nothing(step_name, max = 0):
    """
    A function that does nothing. Used to have an empty description function.
    """
    nothing = True
    
class CybergloveCalibrer:
    """
    A utility to calibrate the cyberglove.
    """
    def __init__(self, description_function = default_description):
        """
        Initialize some class variables: a table containing the calibration steps, a connection to the cyberglove
        library and a description function for the calibration steps
        
        @param description_function: specify a function you want to use to describe the calibration steps ( text / 
        pictures / animation / ... ). Must take a joint name as parameter.
        """        
        self.calibration_steps = []
                
        self.cyberglove = Cyberglove()
        
        # fill the table containing all the joints
        self.joints = {}
        for name in self.cyberglove.get_joints_names():
            self.joints[name] = Calibration()
        
        self.get_calibration_steps()
        
        if description_function == None:
            description_function = do_nothing
        self.description_function = description_function
        
    def get_calibration_steps(self):
        """
        Read the calibration steps from the xml file.
        
        @return: 0 when the values were read.
        """
        
        #first step: calibrate 0s, 3s and TH4 
        joints1 = [Joint("G_IndexMPJ", 0, 90), Joint("G_IndexPIJ", 0, 90), Joint("G_IndexDIJ", 0, 90), 
                  Joint("G_MiddleMPJ", 0, 90), Joint("G_MiddlePIJ", 0, 90), Joint("G_MiddleDIJ", 0, 90),
                  Joint("G_RingMPJ", 0, 90), Joint("G_RingPIJ", 0, 90), Joint("G_RingDIJ", 0, 90),
                  Joint("G_PinkieMPJ", 0, 90), Joint("G_PinkiePIJ", 0, 90), Joint("G_PinkieDIJ", 0, 90),
                  Joint("G_ThumbAb", 50, 0) ]
        self.calibration_steps.append(CalibrationStep(step_name="Joints 0s, 3s and thumb abduction (THJ4)",
                                                      step_description = ["Hand flat on a table, fingers joined, thumb opened",
                                                                          "Hand forming a fist, thumb closed"],
                                                      joints = joints1 ))
        
        #second step: calibrate 4s, TH1, TH2, TH5 and WR2
        joints2 = [Joint("G_ThumbIJ", 90, 0), Joint("G_ThumbMPJ", 30, -30),  
                   Joint("G_MiddleIndexAb", 0, 50), Joint("G_RingMiddleAb", 0, 50), Joint("G_PinkieRingAb", 0, 50),
                   Joint("G_WristYaw", 10, -30)]
        self.calibration_steps.append(CalibrationStep(step_name="Joints 4s + TH1, 2, and WR2",
                                                      step_description = ["Hand flat fingers joined, thumb completely curled under the palm, wrist 2 bent to the left",
                                                                          "Hand flat fingers apart, thumb completely opened, wrist bent 2 to the right"],
                                                      joints = joints2 ))
        
        #third step: calibrate TH5 
        joints3 = [Joint("G_ThumbRotate", 60, -60)]
        self.calibration_steps.append(CalibrationStep(step_name="Joint TH5",
                                                      step_description = ["thumb completely under the palm",
                                                                          "thumb completely opened, curled over the palm"],
                                                      joints = joints3 ))
        
        #fourth step: calibrate LF5 and WR1
        joints4 = [Joint("G_PalmArch", 0, 40), Joint("G_WristPitch", -30, 40)]
        self.calibration_steps.append(CalibrationStep(step_name="LF5 and WR1",
                                                      step_description = ["Hand flat, wrist 1 bent backward",
                                                                          "Palm curled (LFJ5), wrist 1 bent forward"],
                                                      joints = joints4 ))
        
        return 0
        
    def do_step_min(self, index):
        """
        Run  the given step of the calibration, gets the min values.
        
        @param index: the index of the step in the calibration file
        @return: 0 when the values were read. 
        """
        joints = self.calibration_steps[index].joints
        # display the description
        self.description_function(self.calibration_steps[index].step_description[0], 0)
        
        for joint in joints:
            name = joint.name         
            #read the min values
            self.joints[name].raw_min = self.cyberglove.read_raw_average_value(name)
            self.joints[name].calibrated_min = joint.min
            #still needs to read the max before fully calibrated
            self.joints[name].is_calibrated += 0.5
        return 0
        
    def do_step_max(self, index):
        """
        Run  the given step of the calibration, gets the max values.
        As this method is called after the do_step_min() method, we don't display the description
        
        @param index: the index of the step in the calibration file
        @return: 0 when the values were read. 
        """
        joints = self.calibration_steps[index].joints
        # display the description
        self.description_function(self.calibration_steps[index].step_description[1], 0)
        
        for joint in joints:
            name = joint.name         
            #read the max values
            self.joints[name].raw_max = self.cyberglove.read_raw_average_value(name)
            self.joints[name].calibrated_max = joint.max
            #still needs to read the max before fully calibrated
            self.joints[name].is_calibrated += 0.5
        return 0
    
    def is_step_done(self, joint_name):
        """
        Check if the joint is calibrated.
        
        @param joint_name: the name of the joint
        @return: 1 if the joint has already been calibrated. 
        """
        
        return self.joints[joint_name].is_calibrated
        
    def all_steps_done(self):
        """
        Check if all the steps were processed.
        
        @return: True if all the steps were processed. 
        """
        for calib in self.joints.values():
            if calib.is_calibrated != 1:
                return False

        return True
    
    def reorder_calibration(self):
        """
        Reorder the calibration: set the raw_min to the min raw_value
        """
        for name in self.joints.keys():
            if self.joints[name].raw_min > self.joints[name].raw_max:
                tmp_raw = self.joints[name].raw_min
                tmp_cal = self.joints[name].calibrated_min
                self.joints[name].raw_min = self.joints[name].raw_max
                self.joints[name].calibrated_min = self.joints[name].calibrated_max
                self.joints[name].raw_max = tmp_raw
                self.joints[name].calibrated_max = tmp_cal
    
    def write_calibration_file(self, filepath):
        """
        Checks if all the steps were processed by calling self.all_steps_done()
        Reorder the calibration
        Then writes the whole calibration to a given file.
        
        @param filepath: Where to write the calibration
        @return: 0 if the file has been written, 
                -1 if the calibration is not finished yet,
                -2 if other error     
        """
        
        #Where all the steps processed?
        if not self.all_steps_done():
            return -1
        
        #reorder the calibration
        self.reorder_calibration()        
        
        ###############
        # Write to an xml file
        ###############
        #store the text in a table
        text = []
                
        text.append("<?xml version=\"1.0\" ?>")
        text.append("<Cyberglove_calibration>")
        for name in self.joints:
            #joint name
            text.append("<Joint name=\""+name+"\">")
            
            cal = self.joints[name]
            #min value
            text.append("<calib raw_value=\""+str(cal.raw_min)
                            +"\" calibrated_value=\""
                            +str(cal.calibrated_min)+"\"/>")
            
            #max value
            text.append("<calib raw_value=\""+str(cal.raw_max)
                            +"\" calibrated_value=\""
                            +str(cal.calibrated_max)+"\"/>")
            
            text.append("</Joint>")
        
        text.append("</Cyberglove_calibration>")
        
        #write the text to a file
        try:
            output = open(filepath, "w")
            for line in text:
                output.write(line+"\n")
            
            output.close()
        except:
            return -2

        return 0
    
    def load_calib(self, filename):
        if filename == "":
            return -1
        
        rospy.wait_for_service('/cyberglove/calibration')
        try:
            calib = rospy.ServiceProxy('cyberglove/calibration', CalibrationSrv)
            path = filename.encode("iso-8859-1")
            resp = calib(path)
            return resp.state
        except rospy.ServiceException, e:
            print 'Failed to call start service'
            return -2
    
    

##############
#    MAIN    #
##############
def main():
    cyber_calib = CybergloveCalibrer()
    for i in range(0, len(cyber_calib.calibration_steps)):
        raw_input(cyber_calib.calibration_steps[i].step_description[0])
        cyber_calib.do_step_min(i)
        raw_input(cyber_calib.calibration_steps[i].step_description[1])
        
        cyber_calib.do_step_max(i)
    error = cyber_calib.write_calibration_file("../../param/cyberglove.cal")
    print error
    
    return 0


# start the script
if __name__ == "__main__":
    main()
