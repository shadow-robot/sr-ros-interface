#!/usr/bin/python
import sys
sys.path.append("python_hand_library/")

#import wx GUI
import wx

#import other stuff
import subprocess
from Grasp import Grasp
from grasps_interpoler import GraspInterpoler
from grasps_parser import GraspParser
from joint_sliders import MyJointFrame


class GraspsSaver(wx.StaticBox):
    """
    A GUI widget displaying a grasp manager, allowing the user to save grasps
    and to make interpolation between two grasps.
    """
    def __init__(self, parent, id, title,hand,joint_chooser=0):
        """
        Initializes all the variables needed by the widget

        @param parent : the wx.window component containing the widget
        @param id : the identifying number of the widget
        @param title : the title of the widget shown on the border
        @param hand : the library connecting to ROS 
        @param joint_chooser : the component used to select joints to save
        """
        wx.StaticBox.__init__(self,parent,id, title)
        self.myShadowHand = hand
        process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
        self.rootPath = process.communicate()[0]
        self.rootPath = self.rootPath.split('\n')
        self.rootPath = self.rootPath[0]
        #print "path : "+self.rootPath
        self.myParser = GraspParser()
        self.myParser.parse_tree(self.rootPath+"/src/sr_control_gui/grasps.xml")
        self.grasp_interpoler1 = 0
        self.grasp_interpoler2 = 0
        self.panel = wx.Panel(parent,id)
        self.grasps_from = 0
        self.grasps_to = 0
        self.select_grasp_to = 0
        self.select_grasp_from = 0
        self.current_grasp = 0
        self.myslider = 0
        self.myslider2 = 0
        self.jointsToSave = 0
        self.joint_chooser = joint_chooser
        self.buttonsave=0
        self.graspname = 0
        self.drawGraspsSaver()
        self.Fit()

    def drawGraspsSaver(self):
        """
        Actually draws the component
        """
        keys = self.myParser.grasps.keys()
        keys.sort()
        self.grasps_from = wx.ListBox(self.panel, 26, wx.DefaultPosition, (170, 130), keys , wx.LB_SINGLE | wx.EXPAND)
        self.grasps_from.SetSelection(0)
        self.select_grasp_from = self.myParser.grasps.get(self.myParser.grasps.keys()[0])
        self.grasps_to = wx.ListBox(self.panel, 27, wx.DefaultPosition, (170, 130),  keys, wx.LB_SINGLE)
        self.grasps_to.SetSelection(0)
        self.select_grasp_to = self.myParser.grasps.get(self.myParser.grasps.keys()[0])
        
        sizer = wx.FlexGridSizer(vgap=5, hgap=20)
        subsubsizer1 = wx.FlexGridSizer(rows=2, cols=1, vgap = 10)
        subsubsizer1.AddGrowableCol(1,1)
        subsubsizer1.Add(wx.StaticText(self.panel, -1, "From: "))
        subsubsizer1.Add(self.grasps_from)

        subsubsizer2 = wx.FlexGridSizer(rows=3, cols=1, vgap = 20)
        subsubsizer2.Add(wx.StaticText(self.panel, -1, "Past to current"),0,wx.ALIGN_CENTER)
        self.myslider = wx.Slider(self.panel, 28, value = 100, minValue = 0,maxValue = 100, size=(200,40),style = wx.HORIZONTAL | wx.SL_LABELS | wx.SL_AUTOTICKS  )
        subsubsizer2.Add(self.myslider)
        self.buttonsave = wx.Button(self.panel,-1,'Save current grasp')
        subsubsizer2.Add(self.buttonsave,0,wx.EXPAND)
        
        subsubsizer3 = wx.FlexGridSizer(rows=2, cols=1, vgap = 10)
        subsubsizer3.Add(wx.StaticText(self.panel, -1, "To: "))
        subsubsizer3.Add(self.grasps_to)
        
        subsubsizer4 = wx.FlexGridSizer(rows=3, cols=1, vgap = 20, )
        subsubsizer4.Add(wx.StaticText(self.panel, -1, "Current to next"),0,wx.ALIGN_CENTER)
        self.myslider2= wx.Slider(self.panel, 28, value = 0, minValue = 0,maxValue = 100, size=(200,40),style = wx.HORIZONTAL | wx.SL_LABELS | wx.SL_AUTOTICKS  )
        subsubsizer4.Add(self.myslider2)
        self.graspname=wx.TextCtrl(self.panel,-1)
        self.graspname.SetValue('Grasp name')
        subsubsizer4.Add(self.graspname, 0,wx.EXPAND)
        
        subsizer = wx.FlexGridSizer(rows=1, cols=4, vgap = 10)
        subsizer.Add(subsubsizer1)
        subsizer.Add(subsubsizer2)
        subsizer.Add(subsubsizer4)
        subsizer.Add(subsubsizer3)
        sizer.Add(subsizer)

        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 15)
        self.panel.SetSizer(border)

        #Bind the events
        self.grasps_from.Bind(wx.EVT_LISTBOX, self.onSelectPrevious)
        self.grasps_to.Bind(wx.EVT_LISTBOX, self.onSelectNext)
        self.myslider.Bind(wx.EVT_SLIDER, self.onChangeSlider1)
        self.myslider2.Bind(wx.EVT_SLIDER, self.onChangeSlider2)
        self.buttonsave.Bind(wx.EVT_BUTTON, self.saveGrasp)
        self.saveAsCurrent()

    #def generateGraspToXML(self, name, dict):
        #"""
        #Converts data from the hand to xml to allow saving        

        #@param name : the name the grasp should have
        #@param dict : the dictionnary mapping joints to their position for the grasp
        #@return : the grasp to its xml format
        #"""
        #grasp = '    <grasp name="'+name+'">'+'\n'
        #if type == ID_POS:
            #for nam, pos in dict.items():
                #if nam in self.jointsToSave:
                    #grasp = grasp + '        <joint name="'+str(nam)+'">'+str(pos)+'</joint>\n'
        #grasp = grasp+'    </grasp>'+'\n'
        #return grasp
    
    def saveGrasp(self, event):
        """
        Saves the current grasp to grasps.xml
    
        @param event : the event thrown by the saving button (not used)
        """
        print 'saving'    
        grasp = Grasp()
        grasp.grasp_name = self.graspname.GetValue()
        all_joints = self.myShadowHand.read_all_current_positions()
        if self.joint_chooser != 0:
            joints_to_save = self.joint_chooser.getSelected()
            for joint_to_save in joints_to_save:
                grasp.joints_and_positions[joint_to_save] = all_joints[joint_to_save]
        else:
            grasp.joints_and_positions = all_joints
        toWrite = grasp.convert_to_xml() 
        objFileRead = open(self.rootPath+'/src/sr_control_gui/grasps.xml','r')
        previous = objFileRead.readlines()
        objFileRead.close()
        objFileWrite = open(self.rootPath+'/src/sr_control_gui/grasps.xml','w')
        for index in range (0,len(previous)-1):
            objFileWrite.write(previous[index])
        objFileWrite.write(toWrite)
        objFileWrite.write('</root>')
        objFileWrite.close()
        self.myParser.parse_tree()
        self.myParser.grasps.keys().sort()
        keys = self.myParser.grasps.keys()
        keys.sort()
        self.grasps_from.SetItems(keys)
        self.grasps_to.SetItems(keys)

    def onSelectPrevious(self, event):
        """
        Binds the action of changing the selection of the previous grasp and
        changes the current grasp
    
        @param event : the event thrown by the list
        """
        #Try / except used to face strange behaviour at window close
        try:
            index = event.GetSelection()
            name = self.grasps_from.GetString(index)
            self.select_grasp_from = self.myParser.grasps[name]
            self.saveAsCurrent()    
        except wx.PyDeadObjectError,e:
            lapin = 42

    def onSelectNext(self, event):
        """
        Binds the action of changing the selection of the next grasp and
        changes the current grasp
    
        @param event : the event thrown by the list
        """
        #Try / except used to face strange behaviour at window close
        try:
            index = event.GetSelection()
            name = self.grasps_from.GetString(index)
            self.select_grasp_to = self.myParser.grasps[name]
            self.saveAsCurrent()
        except wx.PyDeadObjectError,e:
            lapin = 42
        
    def onChangeSlider1(self, event):
        """
        Binds the slider action to do the interpolation and send it to the hand

        @param event : the event thrown by the slider
        """
        if self.myslider2.GetValue() != 0 :
            self.saveAsCurrent()

        #targets_to_send = self.grasp_interpoler_1.set_percentage_grasp1_to_grasp2(self.myslider.GetValue())
        targets_to_send = self.grasp_interpoler_1.interpolate(self.myslider.GetValue())
        self.myShadowHand.sendupdate_from_dict(targets_to_send)   

    def onChangeSlider2(self, event):
        """
        Binds the slider action to do the interpolation and send it to the hand

        @param event : the event thrown by the slider
        """
        if self.myslider.GetValue() != 100 :
            self.saveAsCurrent()

        #targets_to_send = self.grasp_interpoler_2.set_percentage_grasp1_to_grasp2(self.myslider2.GetValue())
        targets_to_send = self.grasp_interpoler_2.interpolate(self.myslider2.GetValue())
        self.myShadowHand.sendupdate_from_dict(targets_to_send)   

    def saveAsCurrent(self):
        """
        Records the current grasp as the base for interpolations
        """
        self.current_grasp = Grasp()
        self.current_grasp.name = 'CURRENT_UNSAVED'
        self.current_grasp.joints_and_positions = self.myShadowHand.read_all_current_positions()
        self.grasp_interpoler_1 = GraspInterpoler(self.select_grasp_from,self.current_grasp)
        self.grasp_interpoler_2 = GraspInterpoler(self.current_grasp, self.select_grasp_to)
        self.myslider.SetValue(100)
        self.myslider2.SetValue(0)
