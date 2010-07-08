#!/usr/bin/python
import sys
sys.path.append("python_hand_library/")

#import wx GUI
import wx

class JointChooser(wx.StaticBox):
    """
    A GUI widget displaying which joints are possible to save as a grasp.
    """
    def __init__(self, parent, id, title, hand, show1And2=False):
        """
        Initializes all the variables needed by the widget

        @param parent : the wx.window component containing the widget
        @param id : the identifying number of the widget
        @param title : the title of the widget shown on the border
        @param hand : the library connecting to ROS 
        @param show1And2 : sets the widget to display or not joints 1 and 2 instead of joint 0
        """
        wx.StaticBox.__init__(self, parent, id, title)
        self.myShadowHand = hand
        self.show1And2 = show1And2
        self.jointList = {}
        self.jointListSorted = []
        self.panel = wx.Panel(parent, id)
        self.select=wx.Button(self.panel, label='Select all')
        self.unselect=wx.Button(self.panel, label='Unselect all')
        self.drawJointChooser()

    def drawJointChooser(self):
        """
        Actually draws the component
        """
        sizer=wx.FlexGridSizer(cols=5, rows=20,vgap=5, hgap=20)
        for key, value in self.myShadowHand.dict_pos.items():
        #    self.jointList[key]=wx.CheckBox(self.panel, -1, label=key)
        #    self.jointList[key].Show(False)
            self.jointListSorted.append(key)
        self.jointListSorted.sort()
        #Draw the joints only if needed
        if not self.show1And2 :
            listTmp = []
            for index in range(0, len(self.jointListSorted)):
                joint = self.jointListSorted[index]
                if  ((not('2' in joint ) and not('1' in joint)) or (('THJ' in joint) or ('WRJ' in joint))):
                    listTmp.append(joint)
            self.jointListSorted=listTmp
        for joint in self.jointListSorted:
            self.jointList[joint]=wx.CheckBox(self.panel, -1, label=joint)
            sizer.Add(self.jointList[joint])
            self.jointList[joint].Show(True)
        sizer.Add(self.select)
        sizer.Add(self.unselect)
        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 15)
        self.panel.SetSizer(border)
        self.select.Bind(wx.EVT_BUTTON, self.selectAll)
        self.unselect.Bind(wx.EVT_BUTTON, self.unSelectAll)
    

    def selectAll(self, event):
        """
        Ticks all the joints
        """
        for key, value in self.jointList.items():
            value.SetValue(True)

    def unSelectAll(self, event):
        """
        Unticks all the joints
        """
        for key, value in self.jointList.items():
            value.SetValue(False)

    def getSelected(self):
        """
        Gets the list of the joints that are selected for saving
    
        @return : array containing the name of the checked joints
        """
        ret = []
        for key, value in self.jointList.items():
            if value.IsChecked():
                ret.append(key)
        return ret


