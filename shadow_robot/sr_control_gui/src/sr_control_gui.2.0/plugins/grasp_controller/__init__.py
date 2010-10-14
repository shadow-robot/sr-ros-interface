import os, sys
sys.path.append(os.getcwd() + "/plugins")

from PyQt4 import QtCore, QtGui, Qt
from shadow_generic_plugin import ShadowGenericPlugin

from Grasp import Grasp
from grasps_interpoler import GraspInterpoler
from grasps_parser import GraspParser

class GraspController(ShadowGenericPlugin):  
    name = "Grasp Controller"
        
    def __init__(self):        
        ShadowGenericPlugin.__init__(self)
        self.set_icon('images/icons/iconHand.png')
        