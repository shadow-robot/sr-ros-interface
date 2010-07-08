#!/usr/bin/python
import sys
sys.path.append("python_hand_library/")
import time
import subprocess

#import wx GUI
import wx

#import other stuff

from Grasp import Grasp
from grasps_parser import GraspParser


class StepPlayer(wx.StaticBox):
    """
    A GUI widget diplaying tools to replay recorded grasps from an xml file.
    """
    def __init__(self, parent, id, title,hand):
        """
        Initializes all the variables needed by the widget

        @param parent : the wx.window component containing the widget
        @param id : the identifying number of the widget
        @param title : the title of the widget shown on the border
        @param hand : the library connecting to ROS 
        """
        wx.StaticBox.__init__(self,parent,id, title)
        self.myShadowHand = hand
        self.panel = wx.Panel(parent, id)
        self.file_to_play_tf = 0
        self.frequency_cb = 0
        self.frequency_tf = 0
        self.button_play = 0
        self.button_browse = 0
        self.toSend = []
        self.ready = False
        self.lastSent = -1
        self.drawStepPlayer()

    def drawStepPlayer(self):
        """
        Actually draws the component
        """
        sizer = wx.FlexGridSizer(rows=2, cols=1, hgap=5, vgap = 5)
        #sizer_top = wx.FlexGridSizer(cols=2)
        sizer_down = wx.GridSizer(cols=2)
        sizer_down_left = wx.FlexGridSizer(rows=2)

        #self.file_to_play_tf = wx.TextCtrl(self.panel, -1)
        #self.file_to_play_tf.SetEditable(False)
        
        self.button_play = wx.Button(self.panel, -1, 'Play')
        self.button_play.Enable(False)
        self.button_browse = wx.FilePickerCtrl(self.panel, -1)
        self.frequency_cb = wx.CheckBox(self.panel, -1, label='Play all the file')
        self.frequency_tf = wx.TextCtrl(self.panel, -1)
        self.frequency_tf.SetValue('Speed')
        self.frequency_tf.Enable(False)

        sizer.Add(self.button_browse, flag=wx.EXPAND)
        sizer_down_left.Add(self.frequency_cb)
        sizer_down_left.Add(self.frequency_tf)
        sizer_down.Add(sizer_down_left)        
        sizer_down.Add(self.button_play, 0, wx.ALIGN_CENTER)

        #sizer.Add(sizer_top, flag= wx.EXPAND)
        sizer.Add(sizer_down)
        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 15)
        self.panel.SetSizer(border)

        self.frequency_cb.Bind(wx.EVT_CHECKBOX, self.checkBoxEvent)
        self.button_play.Bind(wx.EVT_BUTTON, self.playEvent)
        self.button_browse.Bind(wx.EVT_FILEPICKER_CHANGED, self.chooseEvent)

    def checkBoxEvent(self, event):
        """
        Binds the checkbox control, activates the speed field

        @param event : the event thrown by the checkbox when clicked 
        """
        if self.frequency_cb.IsChecked():
            self.frequency_tf.Enable(True)    
        else : 
            self.frequency_tf.Enable(False)

    def chooseEvent(self, event):
        """
        Binds the browser to its action : parsing the choosen file    

        @param event : the event thrown when the button is clicked
        """
        parser = GraspParser()
        parser.parse_tree(self.button_browse.GetPath())
        grasps = parser.grasps
        for key, value in grasps.items():
            self.toSend.append(value)
        self.ready = True
        self.button_play.Enable(True)

    def playEvent(self, event):
        """
        Binds the play button to its action : reading the file with the specified options

        @param event : the event thrown when the button is clicked
        """
        print self.button_browse.GetPath()
        #print parser.grasps
        if self.frequency_cb.IsChecked() and self.ready:
            speed = float(self.frequency_tf.GetValue())
            for index in range(0, len(self.toSend)):
                time.sleep(speed)
                self.myShadowHand.sendupdate_from_dict(self.toSend[index].joints_and_positions)
        if (not self.frequency_cb.IsChecked()) and self.ready:
            if self.lastSent != len(self.toSend)-1:
                self.lastSent = self.lastSent+1
                
 
