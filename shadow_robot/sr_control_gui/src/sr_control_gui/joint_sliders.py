#!/usr/bin/python
import sys
from shadowhand_ros import ShadowHand_ROS


# import the wxPython GUI package
import wx


# Create a new frame class, derived from the wxPython Frame.
class MyJointFrame(wx.Frame):

    def __init__(self, parent, id, title):
        # First, call the base class' __init__ method to create the frame
        wx.Frame.__init__(self, parent, id, title)

        #shadow arm -> TODO : use dialog window to set ssh / not ssh  
      
	self.myShadowHand = ShadowHand_ROS()

        # Add a panel and some controls to display the size and position
        panel = wx.Panel(self, -1)
        
        #display a 	def resendslider for each joints
        self.joints = []
        self.labels = []
        self.sliders = {} 
        self.slidervalues = {}
        self.display_position_values = {}
        jointindex = 0
        for joint in self.myShadowHand.handJoints:
            self.slidervalues[joint.name]=(self.myShadowHand.valueof(joint.name))
            self.joints.append(joint)
            self.labels.append(wx.StaticText(panel, -1, joint.name))
	    print joint.name
	    print(self.myShadowHand.valueof(joint.name))
            self.sliders[joint.name]=(wx.Slider(panel, jointindex, value = self.myShadowHand.valueof(joint.name), minValue = joint.min, maxValue = joint.max, size=(25,200), style = wx.VERTICAL | wx.SL_LABELS | wx.SL_AUTOTICKS  ))
            jointindex += 1
            texttmp = wx.TextCtrl(panel, -1, value = str(round(self.myShadowHand.valueof(joint.name),2)), size=(50,20))
            texttmp.SetEditable(False)
            texttmp.SetBackgroundColour((86,255,63))
            self.display_position_values[joint.name]=(texttmp)
        self.panel = panel
        
        
        # initialize the positions + sendupdate only when relevant
        self.oldPositions = {} 
        for joint in self.joints:
            self.oldPositions[joint.name]=(self.myShadowHand.valueof(joint.name))

        # Use some sizers for layout of the widgets
        sizer = wx.FlexGridSizer(vgap=5, hgap=20)
        subsizers = []
        
        index = 0
        for label in self.labels:
            # draw the stuff
            subsizerTmp = wx.FlexGridSizer(rows=3, cols=1, vgap = 10)
            subsizerTmp.Add(self.display_position_values[label.GetLabelText()])
            subsizerTmp.Add(self.sliders[label.GetLabelText()])
            subsizerTmp.Add(label)
            subsizers.append(subsizerTmp)
            index += 1
            
        for subsizer in subsizers:
            sizer.Add(subsizer)
            
        #self.timer = wx.Timer(self)      


        # bind the slider event
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)
        #self.Bind(wx.EVT_TIMER, self.update, self.timer)
        #self.Bind(wx.EVT_TIMER, self.update)
                                          
        #self.timer.Start(200)
        
        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 15)
        panel.SetSizerAndFit(border)
        self.Fit()

    def sliderUpdate(self, event):
        #currentslider = event.GetEventObject()
        #self.myShadowHand.sendupdate(currentslider.GetName(), currentslider.GetValue())
        index = 0 
        for name, slider in self.sliders.items():
            value_tmp = slider.GetValue()
            if self.oldPositions[name] != value_tmp:
                self.slidervalues[name] = slider.GetValue()
            index += 1
	#self.update(0)        
    
    def update(self, event):
#       self.myShadowHand.resend_targets(self.slidervalues)
	self.myShadowHand.sendupdate_from_dict(self.slidervalues)
        positions = self.myShadowHand.read_all_current_positions()
        index = 0
        for name, display_pos in self.display_position_values.items():
            display_pos.SetValue(str(round(float(positions[name]),2)))
            index += 1

# Every wxWidgets application must have a class derived from wx.App
class MyApp(wx.App):

    # wxWindows calls this method to initialize the application
    def OnInit(self):

        # Create an instance of our customized Frame class
        frame = MyJointFrame(None, -1, "Joint Sliders")
        frame.Show(True)

        # Tell wxWindows that this is our main window
        self.SetTopWindow(frame)

        # Return a success flag
        return True
def main():
	app = MyApp(0)     # Create an instance of the application class
	app.MainLoop()     # Tell it to start processing events

#start the script
if __name__ == "__main__":
	main()
