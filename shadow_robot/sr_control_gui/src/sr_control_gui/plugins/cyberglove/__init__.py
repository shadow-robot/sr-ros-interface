import os, sys

#Not very pretty....
import subprocess
process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
rootPath = process.communicate()[0]
rootPath = rootPath.split('\n')
rootPath = rootPath[0]
sys.path.append(rootPath+ "/src/sr_control_gui/plugins")
  
from PyQt4 import QtCore, QtGui, Qt
from shadow_generic_plugin import ShadowGenericPlugin

class Cyberglove(ShadowGenericPlugin):  
    name = "Cyberglove"
        
    def __init__(self):
        ShadowGenericPlugin.__init__(self)


    def activate(self):
        ShadowGenericPlugin.activate(self)
        self.set_icon(self.parent.parent.rootPath + '/src/sr_control_gui/images/icons/iconGlove.png')
