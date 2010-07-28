#!/usr/bin/python
import sys
import subprocess
from grasps_parser import GraspParser
from Grasp import Grasp
from grasps_interpoler import GraspInterpoler

sys.path.append("python_hand_library/")
import time
import threading
#import the wxpython GUI package
import wx

class Line(wx.Panel):
    def __init__(self,parent,id, number, grasps, lines):
        wx.Panel.__init__(self,parent,id)
        self.number = number
        self.grasp = wx.ComboBox(self,-1,choices=grasps)
        self.grasp.SetSelection(0)
        self.pause_time = wx.TextCtrl(self,-1, value ="0.0", size=(35,20))
        self.interpolate_time = wx.TextCtrl(self,-1, value ="0.0", size=(35,20))
        array_lines = ['No']
        for l in range (0, lines):
            array_lines.append("Step "+str(l+1))
        self.looping = wx.ComboBox(self, -1, choices = array_lines)
        self.looping.SetSelection(0)
        self.looping_iter = wx.TextCtrl(self,-1,value="1", size=(35,20))
        self.looping_iter.Enable(False)
        self.looping.Bind(wx.EVT_COMBOBOX, self.comboListener)
        self.drawLine()
        
    def drawLine(self):
        layout = wx.FlexGridSizer(cols=12, rows=1, vgap=5)
        layout.Add(wx.StaticText(self, -1,"Step "+ str(self.number)+" :  "), flag=wx.ALIGN_BOTTOM)
        layout.Add(self.grasp, flag=wx.ALIGN_BOTTOM)
        layout.Add(wx.StaticText(self, -1, " Pause time : "), flag=wx.ALIGN_BOTTOM)
        layout.Add(self.pause_time, flag=wx.ALIGN_BOTTOM)
        layout.Add(wx.StaticText(self, -1, " s "), flag=wx.ALIGN_BOTTOM)
        layout.Add(wx.StaticText(self, -1, "Interpolation time : "), flag=wx.ALIGN_BOTTOM)
        layout.Add(self.interpolate_time, flag=wx.ALIGN_BOTTOM)
        layout.Add(wx.StaticText(self, -1, " s "), flag=wx.ALIGN_BOTTOM)
        layout.Add(wx.StaticText(self, -1, "Looping : "), flag=wx.ALIGN_BOTTOM)
        layout.Add(self.looping, flag=wx.ALIGN_BOTTOM)
        layout.Add(self.looping_iter, flag=wx.ALIGN_BOTTOM)
        layout.Add(wx.StaticText(self, -1, " times"), flag=wx.ALIGN_BOTTOM)

        border = wx.BoxSizer()
        border.Add(layout, 0, wx.ALL, 15)
        self.SetSizer(border)

    def comboListener(self, event):
        if self.looping.GetSelection() == 0:
            self.looping_iter.Enable(False)
        else:
            self.looping_iter.Enable(True)

class HandTimeLine(wx.Panel):
    def __init__(self, parent, id, title, hand, window):
        wx.Panel.__init__(self, parent, id)
        #self.bar = wx.ScrollBar(self, -1)
        self.myShadowHand = hand
        self.window = window
        self.resizing = True
        self.panel=wx.Panel(self, id)
        self.grasps = {}
        self.lines = []
        process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
        self.rootPath = process.communicate()[0]
        self.rootPath = self.rootPath.split('\n')
        self.rootPath = self.rootPath[0]
        self.myParser = GraspParser()
        self.myParser.parse_tree(self.rootPath+"/src/sr_control_gui/grasps.xml")
        self.grasps = self.myParser.grasps
        self.current_step = Grasp()
        self.current_step.joints_and_positions = self.myShadowHand.read_all_current_positions()
        self.button_grasps = 0
        self.add_buttons = []
        self.indexToShow = 0
        self.index_button = 0
        self.sizer = wx.GridSizer(rows=20, cols=2)
        self.drawTimeLine()


    def drawTimeLine(self):
        main_sizer = wx.FlexGridSizer(rows = 2, cols=1 )

        panel_top = wx.Panel(self,-1)
        #panel_top.SetBackgroundColour(wx.RED)
        sizer_top = wx.GridSizer(rows=1, cols=3)
        panel_top.SetSizer(sizer_top)        

        button_play = wx.Button(panel_top,-1,"Play")
        self.button_grasps = wx.FilePickerCtrl(panel_top, -1, size=(100,30))
        sizer_top.Add(button_play)
        sizer_top.Add(wx.StaticText(panel_top,-1,"Load more grasps : "), wx.ALIGN_BOTTOM)
        sizer_top.Add(self.button_grasps)
        main_sizer.Add(panel_top)
        main_sizer.Add(self.panel)

        #self.panel.SetBackgroundColour(wx.BLUE)

        self.addLine()
        self.panel.SetSizer(self.sizer)

        
        border = wx.BoxSizer()
        border.Add(main_sizer, 0, wx.ALL, 15)
        self.SetSizer(border)

        button_play.Bind(wx.EVT_BUTTON, self.play)
        self.button_grasps.Bind(wx.EVT_FILEPICKER_CHANGED, self.addGrasps)

    def addLine(self, position=-1):
        newline =  Line(self.panel,-1,len(self.lines)+1, self.grasps.keys(), len(self.lines))      
        self.lines.append(newline) 
        self.sizer.Add(newline, flag=wx.ALIGN_BOTTOM)
        self.index_button = self.index_button+1
        button = wx.Button(self.panel, self.index_button, "+", size=(30,30))
        self.add_buttons.append(button)
        self.sizer.Add(button, flag=wx.ALIGN_CENTER_VERTICAL) 
        button.Bind(wx.EVT_BUTTON, self.addListener)
        if position != -1:
            index = len(self.lines)
            #print "Button clicked : "+str(position)+" should change from "+str(position+1)+" to "+str(len(self.lines))
            while index > position:
                #print "  Changing : "+str(index)
                self.lines[index-1].grasp.SetSelection(self.lines[index-2].grasp.GetSelection())
                self.lines[index-1].pause_time.SetValue(self.lines[index-2].pause_time.GetValue())
                self.lines[index-1].interpolate_time.SetValue(self.lines[index-2].interpolate_time.GetValue())
                self.lines[index-1].looping.SetSelection(self.lines[index-2].looping.GetSelection())
                self.lines[index-1].looping_iter.SetValue(self.lines[index-2].looping_iter.GetValue())
                index=index-1
            self.lines[index].grasp.SetSelection(0)
            self.lines[index].pause_time.SetValue(str(0.0))
            self.lines[index].interpolate_time.SetValue(str(0.0))
            self.lines[index].looping.SetSelection(0)
            self.lines[index].looping_iter.SetValue(str(1))
            

    
    def addListener(self, event):
        #self.indexToShow = self.indexToShow+1
        #self.lines[self.indexToShow].Show()
        #self.add_buttons[self.indexToShow].Show()
        if event.GetId() == self.index_button:
            self.addLine()
        else:
            self.addLine(event.GetId())
        self.window.Validate()
        self.window.Refresh()
        self.window.Update()
        self.window.SetSize((1280,600))
        if self.resizing :
            self.window.SetSize((self.window.GetSize().GetWidth(),self.window.GetSize().GetHeight()+1))
            self.resizing = False
        else :
            self.window.SetSize((self.window.GetSize().GetWidth(),self.window.GetSize().GetHeight()-1))
            self.resizing = True
            

    def addGrasps(self, event):
        self.myParser.parse_tree(self.button_grasps.GetPath())
        new_dict = self.myParser.grasps
        print new_dict
        for key, value in new_dict.items():
            self.grasps[key]=value
        for line in self.lines:
            line.grasp.SetItems(self.grasps.keys())

    def play(self, event=0):
        self.play_steps(0, len(self.lines),1)


    def play_steps(self, start, stop, itera):
        if itera == 0:
            return
        for index in range(start, stop):
            #print 'Playing line '+str(index)
            line = self.lines[index]
            line.SetBackgroundColour(wx.GREEN)
            self.window.Update()
            next_step=self.grasps[line.grasp.GetValue()]
            interpoler = GraspInterpoler(self.current_step,next_step)
            if float(line.interpolate_time.GetValue()) == 0.0:
                    self.myShadowHand.sendupdate_from_dict(next_step.joints_and_positions)   
            else:
                for interpolation in range (0,10*int(float(line.interpolate_time.GetValue()))):
                    targets_to_send = interpoler.interpolate(100.0*interpolation/(10*float(line.interpolate_time.GetValue())))
                    self.myShadowHand.sendupdate_from_dict(targets_to_send)   
                    time.sleep(0.1)

            self.current_step = next_step
            time.sleep(float(line.pause_time.GetValue()))
            line.SetBackgroundColour(wx.NullColor)
            loop_value = line.looping_iter.GetValue()
            if line.looping.GetSelection() != 0:
                line.looping_iter.SetValue(str(int(float(line.looping_iter.GetValue()))-1))
                self.play_steps(int(line.looping.GetSelection())-1,line.number, int(line.looping_iter.GetValue())+1)
            line.looping_iter.SetValue(loop_value)
                
