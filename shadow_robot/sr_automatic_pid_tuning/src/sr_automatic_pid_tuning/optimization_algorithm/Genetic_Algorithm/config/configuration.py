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
##Date:8 Juin 2011


class Fitness_Config(object):
    coeff_dic={'mse':1.7,
               'oscillations':1.2,
               'overshoot':1,
               'risetime':1,
               'settlingtime':0.99,
               'eps':1,
               'cst_yi':2,
               'cst_moy':1,
               'cst_yk':1,
               'j':1,
               'P_min':2,
               'P_max':11,
               'I_min':17,
               'I_max':60,
               'D_min':-70,
               'D_max':-20,
               'DB':0,
               'Shift':6,
               'Offset':0,
               'Imax_min':10,
               'Imax_max':60,
               
               }

    syringe={'syringe_val_min':-5,
             'syringe_val_max':5
             }

    secure_of_genome_and_chromosome={'P-I_positive':'True',
                                     
                                     'column_of_P_sensor':0,
                                     'column_of_I_sensor':1,
                                     'column_of_D_sensor':2,
                                     'column_of_Imax_sensor':3,
                                     'column_of_DeadBand_sensor':4,
                                     'column_of_Shift_sensor':5,
                                     'column_of_Offset_sensor':6,
                                     'column_of_Unknown_sensor':7,
                                     'P_sensor_positive_secure':1,
                                     'I_sensor_positive_secure':1,
                                     'D_sensor_positive_secure':1,
                                     'P_sensor_negative_secure':-1,
                                     'I_sensor_negative_secure':-1,
                                     'D_sensor_negative_secure':-1,
                                     'Imax_sensor_secure':1
                                     
                                      }   
