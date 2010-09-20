#!/usr/bin/python
import sys
sys.path.append("python_hand_library/")

#import the wxpython GUI package
import wx
import time
class ArmJointSliders(wx.StaticBox):
    """
    A GUI widget diplaying the sliders which can control the joints of the hand.
    """
    def __init__(self, parent, id,title, hand):
        """
        Initializes all the variables needed by the widget

        @param parent : the wx.window component containing the widget
        @param id : the identifying number of the widget
        @param title : the title of the widget shown on the border
        @param hand : the library connecting to ROS 
        """
        wx.StaticBox.__init__(self,parent, id, title)
        self.myShadowHand = hand    
        self.sliders = {}
        self.sliders_values = {}
        self.panel = wx.Panel(parent, id)
        self.dataToSend = {}
        self.actualPositions = {}
        t = 0.0

        self.drawJointSliders()
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.update_joints, self.timer)
        self.timer.Start(100)

    def drawJointSliders(self):
        """
        Actually draws the component
        """
        index = 0
        #Create the widgets
        for joint in self.myShadowHand.armJoints:
            #print self.myShadowHand.valueof(joint.name)
            self.dataToSend[joint.name]=self.myShadowHand.valueof(joint.name)
            self.sliders[joint.name]=(wx.Slider(self.panel, id = index, value = self.myShadowHand.valueof(joint.name), minValue = joint.min, maxValue = joint.max, size=(200,25), style = wx.HORIZONTAL   ))
            index += 1

        #Draw and place the widgets
        layout = wx.FlexGridSizer(cols=4, rows=4,  hgap=20)
        for joint in self.myShadowHand.armJoints:
            slider_key=joint.name
            #print slider_key
        #for slider_key, slider_value in self.sliders.items():
            self.actualPositions[slider_key] = wx.TextCtrl(self.panel, -1, value = str(round(self.myShadowHand.valueof(slider_key),2)), size=(35,20))
            self.actualPositions[slider_key].SetEditable(False)
            self.actualPositions[slider_key].SetBackgroundColour((86,255,63))
            layout.Add(self.actualPositions[slider_key], wx.ALIGN_CENTER)
        #for slider_key, slider_value in self.sliders.items():
        #for joint in self.myShadowHand.armJoints:
            #slider_key=joint.name
            layout.Add(self.sliders[slider_key], wx.ALIGN_CENTER)    
            self.sliders_values[slider_key] = wx.StaticText(self.panel,-1, label=str(self.sliders[slider_key].GetValue()))
            layout.Add(self.sliders_values[slider_key], wx.ALIGN_CENTER)
        #for slider_key, slider_value in self.sliders.items():
        #for joint in self.myShadowHand.armJoints:
            #slider_key=joint.name
            layout.Add(wx.StaticText(self.panel,-1,slider_key), wx.ALIGN_CENTER)
        border = wx.BoxSizer()
        border.Add(layout, 0, wx.ALL | wx.ALIGN_CENTER , 15)
        self.panel.SetSizerAndFit(border)

        #Bind the event
        for key, value in self.sliders.items():
            value.Bind(wx.EVT_SLIDER,self.update)

    def update(self, event):
        """
        Binds the sliders action : send new targets (for the whole hand)

        @param event : the event thrown by the slider which moved 
        """
        for slider_key, slider in self.sliders.items():
            self.dataToSend[slider_key]=slider.GetValue()
            self.sliders_values[slider_key].SetLabel(str(slider.GetValue()))
        self.myShadowHand.sendupdate_arm_from_dict(self.dataToSend)    


    def update_joints(self,event):
        """
        Resets the sliders position to be always synchronized with the sent targets    

        @param event : the event thrown by the update timer
        """
        new_values = self.myShadowHand.read_all_current_arm_targets()
        for key, value in new_values.items():
            if key in self.sliders.keys():
                self.sliders[key].SetValue(value)    

        positions=self.myShadowHand.read_all_current_arm_positions()

        for joint, position in positions.items():
            if self.actualPositions.has_key(joint):
                self.actualPositions[joint].SetValue(str(round(float(position),2)))
