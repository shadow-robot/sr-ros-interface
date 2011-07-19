#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, subprocess , time
sys.path.append('../')
from utils.utilitarian import Utilitarian

class Python_Robot_Lib(object):
  
### Constructor
#
    def __init__(self):
        self.utilitarian = Utilitarian()

### listvalues in python
#              
    def get_current_value(self, sensor):        
        command = "listvalues -i 1 -d 100 " + sensor        
        answer = self.utilitarian.run_command(command)
        #p = subprocess.Popen(command.split(),stdout=subprocess.PIPE)
        #p.wait()
        #answer = p.stdout.read()
        result = float(answer.strip('\n'))
        return result        
        #print 'result = ' + result.strip('\n')           
    
    def start_record(self, options, sensors, output_file):        
        command = 'listvalues '+ options + ' ' + sensors + ' -o ' + output_file
        p = subprocess.Popen(command.split(),stdout=subprocess.PIPE)
        return p 
        
    def stop_record(self, process):
        process.terminate()    

### Sendupdate in python
#
    def sendupdate(self, sensor, value):
        command = 'sendupdate ' + sensor + ' ' + value
        answer = self.utilitarian.run_command(command)    

### Contrlr in python
#
    def contrlr(self, smart_motor, options ):
        command = 'contrlr ' + smart_motor + ' ' + options
        answer = self.utilitarian.run_command(command)               
        
if __name__ == "__main__":
    prl = Python_Robot_Lib()
    position_sensor = 'FFJ4_Pos'
    
    #cur_pos = prl.get_current_value(position_sensor)
    #prl.get_current_value(position_sensor)
    #print cur_pos
    
    options = '-d 100'
    output_file = 'toto.txt'
    
    process = prl.start_record(options, position_sensor, output_file)
    time.sleep(1)
    prl.stop_record(process)
    print 'terminate'