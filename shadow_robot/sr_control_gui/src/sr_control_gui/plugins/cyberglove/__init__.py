import os, sys
sys.path.append(os.getcwd() + "/plugins")
  
from PyQt4 import QtCore, QtGui, Qt
from shadow_generic_plugin import ShadowGenericPlugin

class Cyberglove(ShadowGenericPlugin):  
    name = "Cyberglove"
        
    def __init__(self):
        ShadowGenericPlugin.__init__(self)
        self.set_icon('images/icons/iconGlove.png')

    def activate(self):
        ShadowGenericPlugin.activate(self)
