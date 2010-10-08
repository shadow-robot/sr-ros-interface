from yapsy.IPlugin import IPlugin

from PyQt4 import QtCore, QtGui, Qt

class MyMdiSubWindow(QtGui.QMdiSubWindow):
    def __init__(self):
        """
        This MdiSubWindow removes itself from the MdiArea when it closes. It can then be reopened.
        """
        self.container = 0
        QtGui.QMdiSubWindow.__init__(self)
        
    def set_container(self, container):
        self.container = container
        
    def closeEvent(self, event):
        self.container.removeSubWindow(self)
        #self.container.closeActiveSubWindow()

class GenericPlugin(IPlugin):  
    name = "GenericPlugin"
    
    def __init__(self):
        self.id = 0
        self.parent = 0        
        self.window = MyMdiSubWindow()
        #self.window.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.window.widget()
        self.window.setWindowTitle(self.name)
        
    def set_parent(self, parent):
        self.parent = parent
        self.window.set_container(self.parent.container)
                
    def activate(self):
        self.parent.container.addSubWindow(self.window)
        self.window.show()
    
    