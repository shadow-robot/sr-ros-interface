import sys
sys.path.append("python_hand_library/")
import roslib; roslib.load_manifest("sr_control_gui")
import rospy
#from Start.srv import *
#import wx GUI
import wx
import subprocess
from glove import Glove
#import other stuff

BOX_SEND = 0
BOX_DATA = 1
BOX_GRASP = 2 
BOX_GLOVE = 3

class Accessories(wx.StaticBox):
    """
    A GUI widget displaying and controling the topics published and subscribed
    by the hand, plus cyberglove and cybergrasp.
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
        self.panel = wx.Panel(parent,id)
        self.sendupdaters = []        
        self.data = []    
        self.cyberglove = []
        self.cybergrasp = []
            
        self.liste = []
        self.getTopics()
        self.findHands()
        self.findAccessories()

        self.drawAccessories()

    def drawAccessories(self):
        """
        Actually draws the component
        """
        sizer = wx.FlexGridSizer(rows=3, cols=4)
        
        sizer.Add(wx.StaticText(self.panel,-1,"Publish to : "))
        self.sendup_cb = wx.ComboBox(self.panel,BOX_SEND, choices=self.sendupdaters)
        if len(self.sendupdaters) > 0:
            self.sendup_cb.SetSelection(0)        

        sizer.Add(self.sendup_cb)

        sizer.Add(wx.StaticText(self.panel,-1,"Read from : "))
        self.data_cb = wx.ComboBox(self.panel,BOX_DATA, choices=self.data)        
        if len(self.data) > 0:
            self.data_cb.SetSelection(0)        
        
        sizer.Add(self.data_cb)

        sizer.Add(wx.StaticText(self.panel,-1,"Cyberglove : "))
        self.glove_cb = wx.ComboBox(self.panel,BOX_GLOVE, choices=self.cyberglove)        
        sizer.Add(self.glove_cb)

        sizer.Add(wx.StaticText(self.panel,-1,"Cybergrasp : "))
        self.grasps_cb = wx.ComboBox(self.panel,BOX_GRASP, choices=self.cybergrasp)        
        sizer.Add(self.grasps_cb)

        self.sendup_cb.Bind(wx.EVT_COMBOBOX, self.eventComboBox)
        self.data_cb.Bind(wx.EVT_COMBOBOX, self.eventComboBox)
        self.glove_cb.Bind(wx.EVT_COMBOBOX, self.eventComboBox)
        self.grasps_cb.Bind(wx.EVT_COMBOBOX, self.eventComboBox)
        

        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 15)
        self.panel.SetSizer(border)

    def getTopics(self):
        """
        Modify the list of found topics using the bash "rostopic list"
        Only shows actually published topics
        """
        process = subprocess.Popen("rostopic list -p".split(), stdout=subprocess.PIPE)
        liste =  process.communicate()[0]
        self.liste = liste.split('\n')

    def findHands(self):
        """
        Modify the list of topic which may be related to the hand
        """
        for topic in self.liste : 
            if '/sendupdate' in topic :
                self.sendupdaters.append(topic)
            if '/shadowhand_data' in topic :
                self.data.append(topic)
            
    def findAccessories(self):
        """
        Modify the list of topics which may be related to accessories
        """
        for topic in self.liste : 
            if '/cyberglove' in topic :
                self.cyberglove.append(topic)
            if '/cybergrasp' in topic :
                self.cybergrasp.append(topic)  
        if len(self.cybergrasp) > 0 :
            self.cybergrasp[0:0] = ['Shutdown']
        if len(self.cyberglove) > 0 :
            self.cyberglove[0:0] = ['Shutdown']

    def eventComboBox(self, event):
        """
        Binds the action of changing the value of any combobox

        @param event : the event thrown by the combobox
        """
        if event.GetId() == BOX_SEND:
            print self.sendup_cb.GetValue()
            self.myShadowHand.setSendUpdate_topic(self.sendup_cb.GetValue())
        if event.GetId() == BOX_DATA:
            print self.data_cb.GetValue()
            self.myShadowHand.setShadowhand_data_topic(self.data_cb.GetValue())
        if event.GetId() == BOX_GLOVE:
            #print self.glove_cb.GetValue()
            if self.glove_cb.GetCurrentSelection() == 0:
                rospy.wait_for_service("cyberglove/start")
                print "Service found"
                try:
                    start = rospy.ServiceProxy('cyberglove/start', Start)
                    print "Service proxy created" 
                    resp = start(False)
                    return resp1.state
                except rospy.ServiceException, e:
                    print 'Failed to call start service'
                #print "shutdown" 
                
        if event.GetId() == BOX_GRASP:
            print self.grasps_cb.GetValue()

