#!/usr/bin/env python
import sys
sys.path.append("python_hand_library/")
import roslib; roslib.load_manifest('sr_control_gui')
import rospy
from shadowhand_ros import ShadowHand_ROS
from hand_joint_sliders import JointSliders
from hand_grasps_controller import GraspsSaver
from hand_joint_chooser import JointChooser

#import the wxpython GUI package
import wx
import time

MENU_EXIT = 0
MENU_RECORD = 1

class MainWindowLite(wx.Frame):
    """
    The main window of the lite GUI, displaying sliders and grasp manager to control the hand.
    """
    def __init__(self, parent, id, title):
        """
        Initializes the window and create all the widgets and the menu
    
        @param parent : the window containing the program. Should be None
        @param id : the identifying number the the frame
        @param title : the title displayed on the window
        """
        wx.Frame.__init__(self, parent, id, title, size=(1280,600))    
        #Create the library
        self.myShadowHand = ShadowHand_ROS()

        #Draw menu
        menubar=wx.MenuBar()
        fil=wx.Menu()
        fil.Append(MENU_EXIT,"Exit","Quit")
        rec=wx.Menu()
        menubar.Append(fil,"File")
        wx.EVT_MENU(self, MENU_EXIT, self.menuListener)
        self.SetMenuBar(menubar)
        
        #Add the tabs

        #Create pages and layouts
        page1 = wx.Panel(self,-1)
        layout1 = wx.FlexGridSizer(cols=1, rows=2, vgap=20)
        page1.SetSizer(layout1, True)
        #if self.myShadowHand.has_arm():
            #subPage0 = wx.Panel(page1,-1)
            #sublayout0 = wx.FlexGridSizer(cols=2, hgap=10)
            #subPage0.SetSizer(sublayout0, True)
        subPage1 = wx.Panel(page1, -1)
        sublayout1 = wx.FlexGridSizer(cols=2, rows=1, vgap=20)
        subPage1.SetSizer(sublayout1, True)
        #Create the widgets
        jointSliders = JointSliders(page1, -1,"Joints",self.myShadowHand)
        jointChooser = JointChooser(subPage1,-1,"Joint Chooser", self.myShadowHand)
        self.grasps = GraspsSaver(subPage1, -1, "Grasps", self.myShadowHand, jointChooser)
        
        #Add the widgets to display
        self.addControler(layout1, jointSliders)
        layout1.Add(subPage1)
        self.addControler(sublayout1, self.grasps)
        self.addControler(sublayout1, jointChooser)
        
    def addControler(self,container,controler, flag = wx.ALIGN_CENTER):
        """
        Adds a widget to the window        

        @param container : the sizer which should contain the widget
        @param controler : the widget to add
        """
        sizertmp=wx.StaticBoxSizer(controler,wx.HORIZONTAL)
        sizertmp.Add(controler.panel,0,wx.ALL,2)
        container.Add(sizertmp, flag)    
    
    def menuListener(self, event):
        """
        Bind the menu to its action

        @param event : the event thrown by the clicked component
        """
        if event.GetId() == MENU_EXIT : 
            self.Close()
        

class MyApp(wx.App):

    def OnInit(self):

        frame = MainWindowLite(None,-1, "ShadowHand controller lite")
        frame.Show(True)

        self.SetTopWindow(frame)

        return True

def main():
    app = MyApp(0)
    app.MainLoop()

#start the script
if __name__ == "__main__":
    main()        
