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
##Date:22 Juin 2011

import os
import pickle


class PID_Signal(object):
    def __init__(self):
        self.PID_sig_joints={}
        self.PID_process=os.getpid()
        print("PID signal is", os.getpid())
        return


    def get_pid_signal(self,joint_name):
        """
        Get the PID SIG linked to one joint's use //put it in a dictionnary//
        @return: nothing
        """
        ##creating a new dict or overwritting one ?
        pkl_file=open("pid_sig_joints_dic.pkl","rb")
        dico=pickle.load(pkl_file)
        pkl_file.close()
        ##
        if len(dico)==0:
            self.PID_sig_joints.setdefault(joint_name)
            self.PID_sig_joints[joint_name]=self.PID_process
            output=open("pid_sig_joints_dic.pkl","wb")
            pickle.dump(self.PID_sig_joints, output)
            output.close()
        else:
            dico.setdefault(joint_name)
            dico[joint_name]=self.PID_process
            output=open("pid_sig_joints_dic.pkl","wb")
            pickle.dump(dico, output)
            output.close()
        return 


