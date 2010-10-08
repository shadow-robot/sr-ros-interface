#!/usr/bin/env python

import logging
#enables the logging used by yapsy
logging.basicConfig(level=logging.ERROR)

from yapsy.PluginManager import PluginManager
from PyQt4 import QtCore, QtGui, QtWebKit
import os, sys

from main_widget import MainWidget

class MainWindow(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        
        ####
        # BASIC PARAMETERS
        ##        
        self.setWindowTitle("Shadow Robot Controller")
        self.resize(800, 600)
        self.setWindowIcon(QtGui.QIcon('images/icons/app_icon.png'))
                
        ####
        # TOOLBAR
        ##
        self.exit = QtGui.QAction(QtGui.QIcon('images/icons/application-exit.png'), 'Exit', self)
        self.exit.setStatusTip('Exit application')
        self.connect(self.exit, QtCore.SIGNAL('triggered()'), QtCore.SLOT('close()'))
        self.toolbar = self.addToolBar('Exit')
        self.toolbar.addAction(self.exit)

        #Shortcuts
        self.exit.setShortcut('Ctrl+Q')

        ####
        # MENUBAR
        ##        
        self.menubar = self.menuBar()
        file = self.menubar.addMenu('&File')
        file.addAction(self.exit)
            
        ####
        # MAIN WIDGET
        ##
        self.setCentralWidget(MainWidget(self))

        ####
        # STATUSBAR
        ##
        self.statusBar().showMessage('Ready', 2000)

