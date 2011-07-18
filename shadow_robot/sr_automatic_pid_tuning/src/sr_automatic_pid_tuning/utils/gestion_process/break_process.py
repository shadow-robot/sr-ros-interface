#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#By: Emilie JEAN-BAPTISTE
##Date:23 Juin 2011

import os
import pickle
import sys
import signal

class Break_Process(object):
    def __init__(self,joint_name):
        self.joint_name=joint_name
        return

    def break_process(self):
        """
        Look at the dict containing joint's PID_SIG and break the process linked to one joint
        User has to write for example "python break_process RFJ4" in the command line
        @return: nothing
        """
        pkl_file=open("pid_sig_joints_dic.pkl","rb")
        dico_running=pickle.load(pkl_file)
        pkl_file.close()
        print("kill",self.joint_name)
        os.kill(dico_running[self.joint_name],signal.SIGSTOP)
        return



def main():
    break_the_process=Break_Process(sys.argv[1])
    break_the_process.break_process()
if __name__=="__main__":
    main()
    
