#!/usr/bin/python
import sys
sys.path.append("python_hand_library/")
from Grasp import Grasp
#import wx GUI
import wx

class StepRecorder(wx.Frame):
    """
    A small GUI window displaying a tool to record a succession of steps in a specified xml file.
    """
    def __init__(self, parent, id, title, hand, grasps_controler):
        """
        Initializes all the variables needed by the widget

        @param parent : the wx.window component containing the widget
        @param id : the identifying number of the widget
        @param title : the title of the widget shown on the border
        @param hand : the library connecting to ROS 
        @param grasps_controler : the controler used to save grasps.
        """
        wx.Frame.__init__(self, parent, id, title, size=(250,70))    
        self.panel=wx.Panel(self,-1)
        self.grasps_controler = grasps_controler
        self.myShadowHand = hand
        self.files = 0
        self.textCtrl = 0 
        self.number = 0
        self.recordState = False
        self.drawStepRecorder()

    def drawStepRecorder(self):
        """
        Actually draws the component
        """
        sizer = wx.FlexGridSizer(rows=2, cols=2,vgap=10, hgap=5)
        record = wx.Button(self.panel,-1, "Record current step")
        record.Bind(wx.EVT_BUTTON, self.record)
        self.files = wx.TextCtrl(self.panel,-1)
        
        self.textCtrl = wx.StaticText(self.panel, -1, label='0')
        sizer.Add(record,0 ,wx.EXPAND)
        sizer.Add(self.files,0 ,wx.EXPAND)

        sizer.Add(wx.StaticText(self.panel,-1, label="Steps recorded : "),0 ,wx.ALIGN_CENTER)
        sizer.Add(self.textCtrl,0 ,wx.ALIGN_CENTER)
        
        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 15)
        self.panel.SetSizer(border)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

    def record(self, event):
        """
        Binds the record button to its action : writing the grasp to xml

        @param event : the event thrown when the record button is clicked
        """
        self.grasps_controler.saveAsCurrent()
        grasp_xml = self.grasps_controler.current_grasp.convert_to_xml()
        self.myShadowHand.record_step_to_file(self.files.GetValue(),grasp_xml)
        self.number = self.number+1
        self.textCtrl.SetLabel(str(self.number))
        #print grasp_xml
        

    def OnClose(self, event):
        """
        Rebinds the effect of closing the window : just hides it to be recalled

        @param event : the event thrown when the close button is clicked
        """
        self.Hide()
        self.number = 0
        
