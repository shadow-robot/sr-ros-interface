#!/usr/bin/env python
import sys
sys.path.append("python_hand_library/")
import roslib; roslib.load_manifest('sr_control_gui')
import rospy
from shadowhand_ros import ShadowHand_ROS
from hand_joint_sliders import JointSliders
from hand_grasps_controller import GraspsSaver
from hand_joint_chooser import JointChooser
from hand_visualisation_manager import VisualisationManager
from hand_step_recorder import StepRecorder
from hand_step_player import StepPlayer
from hand_accessories import Accessories
from cyberglove_library import Cyberglove
from arm_joint_sliders import ArmJointSliders
from glove import Glove

#import the wxpython GUI package
import wx
import time

MENU_EXIT = 0
MENU_RECORD = 1

class MainWindow(wx.Frame):
    """
    The main window of the GUI, displaying all the information about the
    hand software.
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
        #time.sleep(1.0)
        self.myCyberglove = Cyberglove()        
        #time.sleep(1.0)

        #Draw menu
        menubar=wx.MenuBar()
        fil=wx.Menu()
        fil.Append(MENU_EXIT,"Exit","Quit")
        rec=wx.Menu()
        rec.Append(MENU_RECORD,"Record","Record steps")
        menubar.Append(fil,"File")
        menubar.Append(rec,"Record")
        wx.EVT_MENU(self, MENU_EXIT, self.menuListener)
        wx.EVT_MENU(self, MENU_RECORD, self.menuListener)
        self.SetMenuBar(menubar)
        
        #Add the tabs
        self.tabs = wx.Notebook(self, -1, style=(wx.NB_TOP))

        #Create pages and layouts
        page1 = wx.Panel(self.tabs,-1)
        layout1 = wx.FlexGridSizer(cols=1, rows=2, vgap=20)
        page1.SetSizer(layout1, True)
        #if self.myShadowHand.has_arm():
            #subPage0 = wx.Panel(page1,-1)
            #sublayout0 = wx.FlexGridSizer(cols=2, hgap=10)
            #subPage0.SetSizer(sublayout0, True)
        subPage1 = wx.Panel(page1, -1)
        sublayout1 = wx.FlexGridSizer(cols=2, rows=1, vgap=20)
        subPage1.SetSizer(sublayout1, True)
        page2 = wx.Panel(self.tabs,-1)
        layout2 = wx.FlexGridSizer(cols=1, rows=2, vgap=20)
        page2.SetSizer(layout2, True)
        page3 = wx.Panel(self.tabs,-1)
        layout3 = wx.FlexGridSizer(cols=1, rows=2, vgap=20)
        page3.SetSizer(layout3, True)
        page4 = wx.Panel(self.tabs,-1)
        layout4 = wx.FlexGridSizer(cols=1, rows=2, hgap = 20,vgap=20)
        page4.SetSizer(layout4, True)

        #Create the widgets
        #if self.myShadowHand.has_arm():
            #jointSliders = JointSliders(subPage0, -1,"Joints",self.myShadowHand)
        #else:
            #jointSliders = JointSliders(page1, -1,"Joints",self.myShadowHand)
        jointSliders = JointSliders(page1, -1,"Joints",self.myShadowHand)
        jointChooser = JointChooser(subPage1,-1,"Joint Chooser", self.myShadowHand)
        self.grasps = GraspsSaver(subPage1, -1, "Grasps", self.myShadowHand, jointChooser)
        visualisation = VisualisationManager(page2, -1, "Visualisation", self.myShadowHand)
        self.step = StepRecorder(self,-1,'Step recorder', self.myShadowHand, self.grasps)
        self.player = StepPlayer(page3,-1,'Step Player', self.myShadowHand)

        self.accessories = Accessories(page4,-1,'Accessories', self.myShadowHand)
        if self.myCyberglove.has_glove():
            self.glove = Glove(page4,-1,'Cyberglove', self.myCyberglove)
        
        if self.myShadowHand.has_arm():
            arm_slider = ArmJointSliders(page4, -1, "Arm", self.myShadowHand)

        #Add the widgets to display
        self.addControler(layout1, jointSliders)
        layout1.Add(subPage1)
        self.addControler(sublayout1, self.grasps)
        self.addControler(sublayout1, jointChooser)
        
        self.addControler(layout2, visualisation)
        self.addControler(layout3, self.player)
        self.addControler(layout4, self.accessories)
        if self.myShadowHand.has_arm():
            self.addControler(layout4, arm_slider)
        else:
            layout4.Add(wx.Panel(page4,-1))
        if self.myCyberglove.has_glove():
            self.addControler(layout4, self.glove)
        else:
            layout4.Add(wx.Panel(page4,-1))
        #Add the pages
        self.tabs.AddPage(page1,'Hand control')            
        self.tabs.AddPage(page2, 'Visualisation')
        self.tabs.AddPage(page3, 'Player')
        self.tabs.AddPage(page4, 'Accessories')
        
        if self.myShadowHand.has_arm():
            print "Arm detected."

        if self.myCyberglove.has_glove():
            print "Glove detected."


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
        if event.GetId() == MENU_RECORD :
            #self.step = StepRecorder(self,-1,'Step recorder', self.myShadowHand, self.grasps)
            self.step.Show(True)    
        

class MyApp(wx.App):

    def OnInit(self):

        frame = MainWindow(None,-1, "ShadowHand controller")
        frame.Show(True)

        self.SetTopWindow(frame)

        return True

def main():
    app = MyApp(0)
    app.MainLoop()

#start the script
if __name__ == "__main__":
    main()        
