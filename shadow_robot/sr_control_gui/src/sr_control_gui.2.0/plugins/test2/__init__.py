import os, sys
sys.path.append(os.getcwd()+"/plugins")
from generic_plugin import GenericPlugin

class Myplugin(GenericPlugin):  
    name = "Generic plugin"

    def __init__(self):
        GenericPlugin.__init__(self)
