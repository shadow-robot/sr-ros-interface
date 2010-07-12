import sys
sys.path.append("cyberglove/")
import roslib; roslib.load_manifest('sr_control_gui')
import rospy
import subprocess
#import wx GUI
import wx

from cyberglove.srv import Calibration
from cyberglove_calibrer import CybergloveCalibrer

class Glove(wx.StaticBox):

    def __init__(self, parent, id, title,glove):
        wx.StaticBox.__init__(self,parent,id, title)
        self.myCyberglove = glove
        self.calib = CybergloveCalibrer()
        self.panel = wx.Panel(parent,id)
        self.instruction = 0
        self.steps = 0
        self.joints_names = {}
        self.current_instruction = 0
        self.button_browse = 0
        process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
        self.new_Path = process.communicate()[0]
        self.new_Path = self.new_Path.split('\n')
        self.new_Path = self.new_Path[0]
        self.drawGlove()

    def drawGlove(self):
        sizer = wx.FlexGridSizer(rows=2, cols=4, hgap = 20, vgap=10)
        sizer.Add(wx.StaticText(self.panel,-1,"Path to save calibration : "))
        self.new_Path = wx.TextCtrl(self.panel, -1, self.new_Path+"/param/new_calibration.cal")
        sizer.Add(self.new_Path, flag=wx.EXPAND)
        sizer.Add(wx.Panel(self.panel,-1))
        sizer.Add(wx.Panel(self.panel,-1))
        self.instruction = wx.StaticText(self.panel,-1,"Instruction : ",size=(180,200))
              

        grasps = wx.FlexGridSizer(rows=2, cols=1)
        grasps.Add(wx.StaticText(self.panel,-1,"Steps : "))
        choice = []
        for step in self.calib.calibration_steps:
            choice.append(step.step_name)
        self.steps = wx.ListBox(self.panel,-1,wx.DefaultPosition, (300,170), choices=choice)
        grasps.Add(self.steps, wx.EXPAND)

        buttons = wx.FlexGridSizer(rows=3, cols=1)
        calib = wx.Button(self.panel,-1,"Calibrate")
        buttons.Add(calib)
        self.button_browse = wx.FilePickerCtrl(self.panel, -1)
        buttons.Add(self.button_browse)

        joints = wx.FlexGridSizer(rows=11, cols=2)
        for joint in self.myCyberglove.joints.keys():
            self.joints_names[joint] = wx.StaticText(self.panel,-1,joint)
            joints.Add(self.joints_names[joint])
            self.joints_names[joint].SetForegroundColour(wx.RED)

        sizer.Add(grasps, wx.EXPAND)
        sizer.Add(self.instruction)
        sizer.Add(buttons)
        sizer.Add(joints)

        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 15)
        self.panel.SetSizer(border)

        self.steps.Bind(wx.EVT_LISTBOX, self.stepListener)
        calib.Bind(wx.EVT_BUTTON, self.buttonListener)
        self.button_browse.Bind(wx.EVT_FILEPICKER_CHANGED,self.browseListener)
        #self.button_browse.Enable(False)

    def stepListener(self, event):
        index = event.GetSelection()
        self.instruction.SetLabel("Instruction :\n\n"+self.calib.calibration_steps[index].step_description[0])
        self.instruction.Wrap(180)

    def buttonListener(self,event):
        if self.current_instruction == 0:
            self.calib.do_step_min(self.steps.GetSelection())
            self.current_instruction = 1
            self.instruction.SetLabel("Instruction :\n\n"+self.calib.calibration_steps[self.steps.GetSelection()].step_description[self.current_instruction])
        else :
            self.calib.do_step_max(self.steps.GetSelection())
            self.current_instruction = 0
            if self.steps.GetSelection() < self.steps.GetCount():
                self.steps.SetSelection(self.steps.GetSelection()+1)
                self.instruction.SetLabel("Instruction :\n\n"+self.calib.calibration_steps[self.steps.GetSelection()].step_description[self.current_instruction])
            for joint in self.joints_names.keys():
                if self.calib.is_step_done(joint):
                    self.joints_names[joint].SetForegroundColour(wx.GREEN)
        self.instruction.Wrap(180)
        if self.calib.all_steps_done():
            print "All steps done, saving."
            self.calib.write_calibration_file(self.new_Path.GetValue())
            dialog = wx.MessageDialog(self.panel,"Calibration successful, do you want to load it ?", style = wx.YES_NO | wx.ICON_QUESTION)
            if dialog.ShowModal() == wx.YES:
                rospy.wait_for_service('/cyberglove/calibration')
                try:
                    calib = rospy.ServiceProxy('cyberglove/calibration', Calibration)
                    path = self.new_Path.GetValue().encode("iso-8859-1")
                    resp = calib(path)
                    return resp.state
                except rospy.ServiceException, e:
                    print 'Failed to call start service'
                
        

    def browseListener(self, event):
        print "Loading calibration file : "+self.button_browse.GetPath()
        rospy.wait_for_service('/cyberglove/calibration')
        try:
            calib = rospy.ServiceProxy('cyberglove/calibration', Calibration)
            path = self.button_browse.GetPath().encode("iso-8859-1")
            resp = calib(path)
            return resp.state
        except rospy.ServiceException, e:
            print 'Failed to call start service'
            

