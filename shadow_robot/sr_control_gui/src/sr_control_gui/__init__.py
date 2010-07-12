#!/usr/bin/env python

"""
This is a graphical application which integrates most of the tools needed by the end-user to control Shadow Robot's Hardware. 

1. How to use the GUI.

    To start the GUI, run C{rosrun sr_control_gui sr_control_gui}

    The GUI is organized in tabs.
        - B{Hand control} provides sliders to control the hand joint by joint, displaying actual positions and current targets set by the sliders. It also provides a grasp manager which allows to go from one saved grasp to another smoothly, moving all the fingers at the same time. It is also possible to come back to the former grasp. Finally, grasps can be saved into the grasps.xml file, using the saver. Beware, only the selected joints (ticked in the joint chooser) will be writen to the file.
        - B{Visualisation} provides various services to launch external useful programs, such as ROS utils. The main part, which is not implemented yet, will allow to choose which information should be displayed in Rviz.
        - B{Player} is used to choose a file containing a list of grasps to replay step by step. Two options are available : playing them by clicking on play for each grasps, or playing the whole file with a given time between each grasp. This is design to be used with the record button on the menu. The field must be filled with a filename to save the grasps.
        - B{Accessories} provides services to control some spare parts of the hand, such as the Shadow Arm, which can be controled like the hand, the Cyberglove (including a calibration service) or the Cybergrasp (not implemented yet). It is also possible to change the topics read by the program in order to change the hand controled.

2. Code API

    It is based on shadowhand_ros.py (see in sr_hand package) library to connect to the hand. Refer to it to see what you can control with that GUI.
"""

from hand_controler import MyApp

def main():
    """
    starts the main sr_control_gui application
    """    
    app = MyApp(0)
    app.MainLoop()

#start the script
if __name__ == "__main__":
    main()        
