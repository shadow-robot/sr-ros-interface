#!/usr/bin/env python
import sys
import os
import roslib; roslib.load_manifest('grasp')
sys.path.append("python_hand_library/")
#from shadowhand import ShadowHand
from shadowhand_ros import ShadowHand_ROS
#import what's needed for the grasps
from Grasp import Grasp
from grasps_interpoler import GraspInterpoler
from grasps_parser import GraspParser
from joint_sliders import MyJointFrame
# import the wxPython GUI package
import wx

DEBUG = 1
ID_POS = 0
ID_TAR = 1

# Create a new frame class, derived from the wxPython Frame.
class MyFrame(wx.Frame):

    def __init__(self, parent, id, title):
        # First, call the base class' __init__ method to create the frame
        wx.Frame.__init__(self, parent, id, title)
        
	jFrame = MyJointFrame(None, -2, "Joint Sliders")
	jFrame.Show(True)
        
	self.myShadowHand = jFrame.myShadowHand
	self.myParser = GraspParser()
        self.myParser.parse_tree()
        
        self.saved_current_grasp = Grasp()
        self.saved_current_grasp.grasp_name = "current"
        
	#Lists containing elements 
        self.select_grasp_from = ""
        self.select_grasp_to   = ""
        self.grasp_interpoler  = ""
	#The central slider
	self.myslider = ""        
	self.save = ""
	self.jointsToSave = []

	menu = wx.Menu()
	menu.Append(42, "Save positions", "Save current position to an xml file")
	menu.Append(43, "Save targets", "Save current target to an xml file")
	menubar=wx.MenuBar()
	menubar.Append(menu,'Save')
	self.SetMenuBar(menubar)

        # Add a panel and some controls to display the size and position
        self.panel = wx.Panel(self, -1)
        self.grasps_from = wx.ListBox(self.panel, 26, wx.DefaultPosition, (170, 130), ["current"] + self.myParser.grasps.keys() , wx.LB_SINGLE)
        self.grasps_to = wx.ListBox(self.panel, 27, wx.DefaultPosition, (170, 130), ["current"] + self.myParser.grasps.keys(), wx.LB_SINGLE)
        
        sizer = wx.FlexGridSizer(vgap=5, hgap=20)
        subsubsizer1 = wx.FlexGridSizer(rows=2, cols=1, vgap = 10)
        subsubsizer1.Add(wx.StaticText(self.panel, -1, "From: "))
        subsubsizer1.Add(self.grasps_from)

        subsubsizer2 = wx.FlexGridSizer(rows=3, cols=1, vgap = 20)
        subsubsizer2.Add(wx.StaticText(self.panel, -1, ""))
        self.myslider = wx.Slider(self.panel, 28, value = 0, minValue = 0, 
                                   maxValue = 100, size=(200,40), 
                                   style = wx.HORIZONTAL | wx.SL_LABELS | wx.SL_AUTOTICKS  )
        subsubsizer2.Add(self.myslider)
        
        subsubsizer3 = wx.FlexGridSizer(rows=2, cols=1, vgap = 10)
        subsubsizer3.Add(wx.StaticText(self.panel, -1, "To: "))
        subsubsizer3.Add(self.grasps_to)
        
        subsizer = wx.FlexGridSizer(rows=1, cols=3, vgap = 10)
        subsizer.Add(subsubsizer1)
        subsizer.Add(subsubsizer2)
        subsizer.Add(subsubsizer3)
        sizer.Add(subsizer)
        
        # bind the events
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate, id=28)
        self.Bind(wx.EVT_LISTBOX, self.onSelectFrom, id=26)
        self.Bind(wx.EVT_LISTBOX, self.onSelectTo, id=27)
	self.Bind(wx.EVT_MENU, self.savePosition,id=42)
	self.Bind(wx.EVT_MENU, self.saveTarget,id=43)
		

        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 15)
        self.panel.SetSizerAndFit(border)
        self.Fit()

    def savePosition(self, event):
	self.clickSave(ID_POS)
	
    def saveTarget(self, event):
	self.clickSave(ID_TAR)

    def generateGraspToXML(self, name, type):
	grasp = '	<grasp name="'+name+'">'+'\n'
	if type == ID_POS:
		for nam, pos in self.myShadowHand.dict_pos.items():
			if nam in self.jointsToSave:
				grasp = grasp + '		<joint name="'+str(nam)+'">'+str(pos)+'</joint>\n'
	if type == ID_TAR:
		for nam, pos in self.myShadowHand.dict_tar.items():
			if nam in self.jointsToSave:
				grasp = grasp + '		<joint name="'+str(nam)+'">'+str(pos)+'</joint>\n'
	grasp = grasp+'	</grasp>'+'\n'
	return grasp
	
    def clickSave(self,type):
	jointList=self.myShadowHand.dict_pos.keys()
	windowChoice = wx . MultiChoiceDialog(self,"Available joints : ","Select joints to save",jointList)	
	if windowChoice.ShowModal() == wx.ID_OK:
		self.jointsToSave = windowChoice.GetSelections()
	#	print(self.jointsToSave)
	for index in range(0,len(self.jointsToSave)):
		self.jointsToSave[index]=jointList[self.jointsToSave[index]]
	#print self.jointsToSave
	#fileChooser = wx.FileDialog(self,'choose','','','*.*',wx.OPEN)
	savePath='grasps.xml';
	#if fileChooser.ShowModal() == wx.ID_OK:
	#	savePath=fileChooser.GetPath()
	objFileRead = open('grasps.xml','r')	
	previous = objFileRead.readlines()	
	objFileRead.close()
	objFileWrite = open(savePath,'w')	
	for index in range (0,len(previous)-1):
		objFileWrite.write(previous[index])
	windowName = wx.TextEntryDialog(self,'Name of the grasp : ')
	name='grasp_test'
	if windowName.ShowModal() == wx.ID_OK:
		name=windowName.GetValue()
	objFileWrite.write(self.generateGraspToXML(name, type))
	objFileWrite.write('</root>')
	objFileWrite.close()
        self.myParser.parse_tree()
	self.grasps_from.SetItems(["current"] + self.myParser.grasps.keys())
	self.grasps_to.SetItems(["current"] + self.myParser.grasps.keys())

    def onSelectFrom(self,event):
        index = event.GetSelection()
        if index == 0: # current position
            self.save_current_grasp()
        self.select_grasp_from = self.myParser.grasps.get(self.myParser.grasps.keys()[index-1])
        if self.select_grasp_to != "":
            self.grasp_interpoler = GraspInterpoler(self.select_grasp_from, self.select_grasp_to)
	print(self.select_grasp_from.joints_and_positions)
	self.myShadowHand.sendupdate_from_dict(self.select_grasp_from.joints_and_positions)
        
    def onSelectTo(self,event):
        index = event.GetSelection()
        if index == 0: # current position
            self.save_current_grasp()
        self.select_grasp_to = self.myParser.grasps.get(self.myParser.grasps.keys()[index-1])
        if self.select_grasp_from != "":
            self.grasp_interpoler = GraspInterpoler(self.select_grasp_from, self.select_grasp_to)

    def sliderUpdate(self, event):
        #currentslider = event.GetEventObject()
        if self.select_grasp_from != "" and self.select_grasp_to != "":
            #then move :)
            targets_to_send = self.grasp_interpoler.set_percentage_grasp1_to_grasp2(self.myslider.GetValue())
            print targets_to_send
            self.myShadowHand.sendupdate_from_dict(targets_to_send)   
 
    def save_current_grasp(self):
        current_positions = self.myShadowHand.read_all_current_positions()
                        
        self.saved_current_grasp.joints_and_positions = self.myShadowHand.dict_pos

# Every wxWidgets application must have a class derived from wx.App
class MyApp(wx.App):

    # wxWindows calls this method to initialize the application
    def OnInit(self):

        # Create an instance of our customized Frame class
        frame = MyFrame(None, -1, "Grasps Manager")
        frame.Show(True)
        # Tell wxWindows that this is our main window
        self.SetTopWindow(frame)

        # Return a success flag
        return True
##############
#    MAIN    #
##############
def main():
    app = MyApp(0)     # Create an instance of the application class
    app.MainLoop()     # Tell it to start processing events
    
    return 0


# start the script
if __name__ == "__main__":
    main()
