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
##Date:12 Juillet 2011

import os

class PathInstantiater(object):
    def __init__(self, path_root, path_record_data):
        self.ensure_dir(path_root)
        for path in path_record_data.values():
            directory = "/".join(path.split("/")[:-1])

            self.ensure_dir(path_root+directory)


    def ensure_dir(self, f):
        try:
            os.makedirs(f)
        except:
            pass


class Path_Config_Dict(object):
    path_root = "/tmp"
    path_record_data={"path_used_by_fitness":"/saved_data_box/movements_fitness_use/all_mvts_for_fitness_",
		      "generations_used_offline_path":"/saved_data_box/Generations_",
		      "best_chromosome_path":"/saved_data_box/debug_current_best_chomosome_",
		      "display_fit_use_path":"/saved_data_box/display_fit_use_",
		      "last_data_path":"/saved_data_box/Last_Result_",
		      "PID_SIG_SUB_path":"/saved_data_box/trash_intrinsic_use/Sig_PID_sub_",
		      "boolean_break_path":"/saved_data_box/trash_intrinsic_use/boolean_break_",
		      "complete_movement_path":"/saved_data_box/complete_mvt_" }

    __path_instantiater__ = PathInstantiater(path_root, path_record_data)
