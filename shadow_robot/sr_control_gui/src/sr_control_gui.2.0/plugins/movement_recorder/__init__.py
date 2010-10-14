import os, sys
sys.path.append(os.getcwd() + "/plugins")

import time, threading

from PyQt4 import QtCore, QtGui, Qt
from shadow_generic_plugin import ShadowGenericPlugin

class Step(QtGui.QWidget):
    def __init__(self, step_index, parent):
        QtGui.QWidget.__init__(self)
        self.step_index = step_index
        self.parent = parent
        self.grasp = 0
        self.pause_time = 0
        self.interpolation_time = 1
        self.loop_to_step = -1
        self.number_of_loops = 0
        self.remaining_loops = 0
        
        self.widgets = []
    
    def draw(self):        
        self.frame = QtGui.QFrame(self)
        self.green = QtGui.QColor(0, 255, 0)
        self.saved_palette = self.palette()
        green_palette = self.palette()
        green_palette.setBrush(Qt.QPalette.Window, self.green)

        self.frame.setPalette(green_palette)
        
        self.grasp = self.parent.sr_library.grasp_parser.grasps.values()[0]
        label_grasp = QtGui.QLabel(self.frame)
        label_grasp.setText("Step " + str(self.step_index + 1) + ":")
        
        list_grasp = QtGui.QComboBox(self.frame)
        for grasp_name in self.parent.sr_library.grasp_parser.grasps.keys():
            list_grasp.addItem(grasp_name)    
        self.frame.connect(list_grasp, QtCore.SIGNAL('activated(QString)'), self.grasp_choosed)
        self.widgets.append(list_grasp)

    
        label_pause = QtGui.QLabel(self.frame)
        label_pause.setText(" Pause Time:")
        self.widgets.append(label_pause)

        pause_input = QtGui.QLineEdit(self.frame)
        pause_input.setValidator(Qt.QDoubleValidator(self))
        pause_input.setText("0.0")
        pause_input.setFixedWidth(35)
        pause_input.setAlignment(QtCore.Qt.AlignRight)
        self.frame.connect(pause_input, QtCore.SIGNAL('textChanged(QString)'), self.pause_changed)
        self.widgets.append(pause_input)

        label_interp = QtGui.QLabel(self.frame)
        label_interp.setText("s.   Interpolation Time:")
        self.widgets.append(label_interp)

        interp_input = QtGui.QLineEdit(self.frame)
        interp_input.setValidator(Qt.QDoubleValidator(self))
        interp_input.setText("1.0")
        interp_input.setAlignment(QtCore.Qt.AlignRight)
        interp_input.setFixedWidth(35)
        self.frame.connect(interp_input, QtCore.SIGNAL('textChanged(QString)'), self.interp_changed)
        self.widgets.append(interp_input)

        label_looping = QtGui.QLabel(self.frame)
        label_looping.setText("s.   Looping from step:")
        self.widgets.append(label_looping)
        
        loop_input = QtGui.QComboBox(self.frame)
        for i in range(0, self.step_index + 1):
            if i == 0:
                loop_input.addItem("None")
            else:
                loop_input.addItem(str(i))
        self.frame.connect(loop_input, QtCore.SIGNAL('activated(QString)'), self.choose_looping)
        self.widgets.append(loop_input)
    
        self.number_loops = QtGui.QLineEdit(self.frame)
        self.number_loops.setValidator(Qt.QIntValidator(self))
        self.number_loops.setDisabled(True)
        self.number_loops.setText("0")
        self.number_loops.setAlignment(QtCore.Qt.AlignRight)
        self.number_loops.setFixedWidth(35)
        #self.number_loops.setReadOnly(True)        
        #self.number_loops.setStyleSheet("QWidget { background-color: lightgrey }")
        self.frame.connect(self.number_loops, QtCore.SIGNAL('textChanged(QString)'), self.number_loops_changed)
        self.widgets.append(self.number_loops)
    
        label_times = QtGui.QLabel(self.frame)
        label_times.setText("times. ")
        self.widgets.append(label_times)
        
        new_step_button = QtGui.QPushButton(self.frame)
        new_step_button.setText("+")
        new_step_button.setFixedWidth(20)
        self.frame.connect(new_step_button, QtCore.SIGNAL('clicked()'), self.add_step)
        self.widgets.append(new_step_button)
        
        remove_step_button = QtGui.QPushButton(self.frame)
        remove_step_button.setText("-")
        remove_step_button.setFixedWidth(20)
        self.frame.connect(remove_step_button, QtCore.SIGNAL('clicked()'), self.remove_step)
        self.widgets.append(remove_step_button)
        
        self.layout = QtGui.QHBoxLayout()
        self.layout.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.setSpacing(2)
        self.layout.addWidget(label_grasp)
        self.layout.addWidget(list_grasp)
        self.layout.addWidget(label_pause)
        self.layout.addWidget(pause_input)
        self.layout.addWidget(label_interp)
        self.layout.addWidget(interp_input)
        self.layout.addWidget(label_looping)
        self.layout.addWidget(loop_input)
        self.layout.addWidget(self.number_loops)
        self.layout.addWidget(label_times)
        self.layout.addWidget(new_step_button)
        self.layout.addWidget(remove_step_button)
        
        self.frame.setLayout(self.layout)
        
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        #self.setWidget(self.frame)        
        self.show()
            
    def grasp_choosed(self, grasp_name):
        self.grasp = self.parent.sr_library.grasp_parser.grasps[str(grasp_name)]

    def pause_changed(self, pause_time):
        self.pause_time = float(pause_time)
        
    def interp_changed(self, interp_time):
        self.interpolation_time = float(interp_time)
    
    def add_step(self):
        self.parent.add_step()
        
    def number_loops_changed(self, number_loops):
        self.number_of_loops = int(number_loops)
        
    def choose_looping(self, looping):
        if looping == "None":
            self.number_loops.setDisabled(True)
            self.number_loops.setText("0")
            self.loop_to_step = -1
        else:
            self.number_loops.setEnabled(True)    
            self.number_loops.setText("1")
            self.loop_to_step = int(looping) - 1
                
    def remove_step(self):
        if len(self.parent.steps) > 1:
            self.parent.layout.removeWidget(self)
            self.parent.steps.remove(self)
            for widget in self.widgets:
                self.layout.removeWidget(self)
                widget.destroy()
            self.destroy()
        Qt.QTimer.singleShot(0, self.parent.window.adjustSize)
    
    def set_step_id(self, index):
        self.step_index = index
    
    def is_playing(self):
        self.frame.setAutoFillBackground(True)
        self.frame.repaint()
        
    def stopped_playing(self):
        self.frame.setAutoFillBackground(False)
        self.frame.repaint()


class SignalWidget(Qt.QWidget):
    isPlayingSig = QtCore.pyqtSignal(int)
    stoppedPlayingSig = QtCore.pyqtSignal(int)
    
    def __init__(self, parent = None):
        super(SignalWidget, self).__init__(parent)


class MovementRecorder(ShadowGenericPlugin):  
    name = "Movement Recorder"
            
    def __init__(self):
        ShadowGenericPlugin.__init__(self)
        self.set_icon('images/icons/iconArmHand.png')
        
        self.frame = QtGui.QFrame()
        self.timer = Qt.QTimer(self.frame)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.setSizeConstraint(Qt.QLayout.SetFixedSize)
        
        self.sublayout = QtGui.QHBoxLayout()
        self.command_frame = QtGui.QFrame()
                
        play_btn = QtGui.QPushButton()
        play_btn.setIcon(QtGui.QIcon('image/icons/play.png'))
        play_btn.setText("Play")
        play_btn.setFixedWidth(60)
        self.command_frame.connect(play_btn, QtCore.SIGNAL('clicked()'), self.button_play_clicked)
        self.sublayout.addWidget(play_btn)
        
        self.signal_widget = SignalWidget(self.frame)
        self.signal_widget.isPlayingSig['int'].connect(self.started_playing)
        self.signal_widget.stoppedPlayingSig['int'].connect(self.stopped_playing)
        
        self.mutex = threading.Lock()
        self.stopped = True
        
        self.thread = None
        
        stop_btn = QtGui.QPushButton()
        stop_btn.setIcon(QtGui.QIcon('image/icons/stop.png'))
        stop_btn.setText("Stop")
        stop_btn.setFixedWidth(60)
        self.command_frame.connect(stop_btn, QtCore.SIGNAL('clicked()'), self.stop)
        self.sublayout.addWidget(stop_btn)
        
        self.command_frame.setLayout(self.sublayout)
        self.layout.addWidget(self.command_frame)
    
    def started_playing(self, index):
        self.steps[index].is_playing()
        
    def stopped_playing(self, index):
        self.steps[index].stopped_playing()
        
    def add_step(self):
        step_tmp = Step(len(self.steps), self)
        self.steps.append(step_tmp)
        self.layout.addWidget(step_tmp)
        step_tmp.draw()
        Qt.QTimer.singleShot(0, self.window.adjustSize)
        for step, index in zip(self.steps, range(0, len(self.steps))):
            step.set_step_id(index)

    def activate(self):
        ShadowGenericPlugin.activate(self)
                                
        self.steps = []
        self.add_step()
        
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)

    def button_play_clicked(self):       
        if len(self.steps) < 1:
            return
        
        for step in self.steps:
            step.remaining_loops =  step.number_of_loops
         
        self.thread = threading.Thread(None, self.play)
        self.thread.start()

    def play(self):  
        self.stopped = False
        first_time = True
        index = 0
             
        while index < len(self.steps):
            if self.stopped:
                return
            
            step = self.steps[index]
            index = self.play_step(step, first_time, index)
            first_time = False
            
    def stop(self):
        self.mutex.acquire()
        self.stopped = True
        self.mutex.release()
            
    def play_step(self, step, first_time, index):
        if first_time:
            self.current_step = step
            return index + 1
        next_step = step
        
        self.signal_widget.isPlayingSig['int'].emit(index)
        self.move_step(next_step)
        self.signal_widget.stoppedPlayingSig['int'].emit(index)
        
        self.current_step = next_step   
        
        #looping?
        if step.loop_to_step != -1:
            if step.remaining_loops > 0:
                step.remaining_loops = step.remaining_loops - 1
                return step.loop_to_step
        return index + 1
    
    def move_step(self, next_step):
        interpoler = self.sr_library.create_grasp_interpoler(self.current_step.grasp, next_step.grasp)
        if self.current_step.interpolation_time == 0.0:
            self.sr_library.sendupdate_from_dict(next_step.grasp.joints_and_positions)
        else:
            for interpolation in range (0, 10 * int(self.current_step.interpolation_time)):
                self.mutex.acquire()
                if self.stopped:
                    self.mutex.release()
                    return
                self.mutex.release()
                
                targets_to_send = self.sr_library.grasp_interpoler.interpolate(100.0 * interpolation / (10 * self.current_step.interpolation_time))
                self.sr_library.sendupdate_from_dict(targets_to_send)        
                time.sleep(0.1)
                  
        time.sleep(self.current_step.pause_time)      
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
         
