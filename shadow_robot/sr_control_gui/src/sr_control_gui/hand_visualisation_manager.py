#!/usr/bin/python
import sys
sys.path.append("python_hand_library/")

#import wx GUI
import wx
import subprocess
import os

#import other stuff

BUTTON_STACK = 1
BUTTON_REPLACE = 2
BUTTON_RVIZ = 3
BUTTON_RXGRAPH = 4
BUTTON_RXCONSOLE = 5
BUTTON_MONITOR = 6
class VisualisationManager(wx.StaticBox):
    """
    A GUI widget displaying the joints and the information which can be 
    displayed in rviz, and allow the user to choose what to display.
    """
    def __init__(self, parent, id, title, hand):
        """
        Initializes all the variables needed by the widget

        @param parent : the wx.window component containing the widget
        @param id : the identifying number of the widget
        @param title : the title of the widget shown on the border
        @param hand : the library connecting to ROS 
        """
        wx.StaticBox.__init__(self, parent, id, title)
        self.myShadowHand = hand
        self.panel = wx.Panel(parent, id)
        self.sizer = 0
        #Hand checkbox
        self.hand_cb = {}
        #List of the fingers
        self.fingers = []
        self.fingers_cb = {}
        self.fingers_bt = {}
        #Dict of joints list by finger
        self.joints = {}
        self.joints_cb = {}
        self.joints_bt = {}
        self.joints_sizers = {}
        #List of information 
        self.information = []
        self.information_cb = []

        self.map_to_box = {}
        self.map_to_text = {}
        self.index = 0
        self.initData()
        self.drawVisualisationManager()


    def drawVisualisationManager(self):
        """
        Actually draws the component
        """
        self.sizer = wx.FlexGridSizer(cols=1, rows=4, vgap=10)
        self.hand_cb = wx.CheckBox(self.panel,self.index, label="Hand")
        self.sizer.Add(self.hand_cb, flag=wx.ALIGN_CENTER)
        self.addToMap(self.hand_cb)
        sizer_finger = wx.FlexGridSizer(cols=len(self.fingers), rows=2,vgap=10,hgap=30)
        for finger in self.fingers :
            self.fingers_cb[finger] = wx.CheckBox(self.panel,self.index,label=finger)
            sizer_finger.Add(self.fingers_cb[finger])
            self.addToMap(self.fingers_cb[finger])
        self.sizer.Add(sizer_finger)
        
        for finger in self.fingers :
            sizer_finger_temp = wx.FlexGridSizer(cols=1, rows=len(self.joints[finger])) 
            self.joints_cb[finger] = []
            for joint in self.joints[finger]:
                self.joints_cb[finger].append(wx.CheckBox(self.panel,self.index,label=joint))
                sizer_finger_temp.Add(self.joints_cb[finger][len(self.joints_cb[finger])-1], flag=wx.ALIGN_RIGHT)
                self.addToMap(self.joints_cb[finger][len(self.joints_cb[finger])-1])
            sizer_finger.Add(sizer_finger_temp)

        buttonpanel = wx.Panel(self.panel,-1)
        stack = wx.Button(buttonpanel,BUTTON_STACK,label="Stack")
        stack.Bind(wx.EVT_BUTTON, self.buttonEvent)
        replace = wx.Button(buttonpanel,BUTTON_REPLACE,label="Replace")
        replace.Bind(wx.EVT_BUTTON, self.buttonEvent)
        rviz = wx.Button(buttonpanel, BUTTON_RVIZ, label="Rviz")
        rviz.Bind(wx.EVT_BUTTON, self.buttonEvent)
        rxgraph = wx.Button(buttonpanel, BUTTON_RXGRAPH, label="Rxgraph")
        rxgraph.Bind(wx.EVT_BUTTON, self.buttonEvent)
        rxconsole = wx.Button(buttonpanel, BUTTON_RXCONSOLE, label="Rxconsole")
        rxconsole.Bind(wx.EVT_BUTTON, self.buttonEvent)
        monitor = wx.Button(buttonpanel, BUTTON_MONITOR, label="Robot Monitor")
        monitor.Bind(wx.EVT_BUTTON, self.buttonEvent)
        sizerbutton = wx.FlexGridSizer(cols=3)
        sizerbutton.Add(stack)
        sizerbutton.Add(replace)
        sizerbutton.Add(rviz)
        sizerbutton.Add(rxgraph)
        sizerbutton.Add(rxconsole)
        sizerbutton.Add(monitor)
        buttonpanel.SetSizer(sizerbutton)
        self.sizer.Add(buttonpanel, flag=wx.ALIGN_CENTER)

        sizer_info = wx.FlexGridSizer(cols=1,rows=len(self.information))
        for info in self.information :
            self.information_cb.append(wx.CheckBox(self.panel,self.index,label=info))
            self.addToMap(self.information_cb[len(self.information_cb)-1])    
            sizer_info.Add(self.information_cb[len(self.information_cb)-1])
        self.sizer.Add(sizer_info)        

        border = wx.BoxSizer()
        border.Add(self.sizer, 0, wx.ALL, 15)
        self.panel.SetSizer(border)

    def buttonEvent(self, event):
        """
        Binds the 3 buttons to its action and calls the service to manage
        rviz display
        @todo : actually call the ROS service
        @param event : the event thrown by the buttons when clicked
        """
        toCall = {}
        for finger in self.joints_cb.keys():
            for joint in self.joints_cb[finger]:
                if joint.GetValue():
                    #print joint.GetLabelText()
                    toCall[joint.GetLabelText()] = []
                    for info in self.information_cb :
                        if info.GetValue() :
                            toCall[joint.GetLabelText()].append(info.GetLabelText())
        if event.GetId() == BUTTON_STACK:
            self.myShadowHand.callVisualisationService(toCall,0)
            process = subprocess.Popen("rostopic list".split(), stdout=subprocess.PIPE)
            print process.communicate()[0]
        if event.GetId() == BUTTON_REPLACE:
            self.myShadowHand.callVisualisationService(toCall,1)
        if event.GetId() == BUTTON_RVIZ:
            subprocess.Popen("roslaunch sr_hand rviz.launch".split())
        if event.GetId() == BUTTON_RXGRAPH:
            subprocess.Popen("rxgraph".split())
        if event.GetId() == BUTTON_RXCONSOLE:
            subprocess.Popen("rxconsole".split())
        if event.GetId() == BUTTON_MONITOR:
            subprocess.Popen("rosrun robot_monitor robot_monitor".split())
    
    def checkBoxEvent(self, event):
        """
        Handles the consistency of the joints checked

        @param event : the event thrown by the clicked checkbox
        """
        name = self.map_to_text[event.GetId()]
        box = self.map_to_box[event.GetId()]
        if name == 'Hand':
            for finger in self.fingers:
                self.fingers_cb[finger].SetValue(box.GetValue())
                for joint_cb in self.joints_cb[finger]:
                    joint_cb.SetValue(box.GetValue())
    
        #Box is a finger box : check all joints
        if name in self.fingers:
            for joint_cb in self.joints_cb[name]:
                joint_cb.SetValue(box.GetValue())
        #Box is a joint box : uncheck the finger box
        else :
            for finger in self.fingers:
                for joints in self.joints[finger]:
                    if name in joints:
                        self.fingers_cb[finger].SetValue(False)    
                        self.hand_cb.SetValue(False)

    def addToMap(self, box):
        """
        Add a box to the maps, mapping the ID of the box to its name and their graphic component

        @param box : the box to add to the maps
        """
        self.map_to_text[self.index]=box.GetLabelText()
        self.map_to_box[self.index]=box
        box.Bind(wx.EVT_CHECKBOX, self.checkBoxEvent, id=self.index)
        self.index = self.index + 1

    def initData(self):
        """
        Initializes the lists with the existing joints of the hand
        """
        current_finger_name_list = []
        current_finger_name = ''
        for joint, pos in self.myShadowHand.dict_pos.items() : 
            current_finger_name = joint[0]+joint[1]
            if not ((current_finger_name) in current_finger_name_list) :
                current_finger_name_list.append(current_finger_name)
                self.fingers.append(current_finger_name)
                self.joints[current_finger_name] = []
            self.joints[current_finger_name].append(joint)
        self.initInfo()


    def initInfo(self):
        """
        Initializes the lists with the displayable information of the joints
        """
        self.information.append("Info 1")
        self.information.append("Info 2")

