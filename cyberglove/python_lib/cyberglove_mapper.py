#!/usr/bin/env python
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
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#
"""
Minimizes the mapping matrix using a simplex algorithm

@author: Ugo Cupcic
@contact: ugo@shadowrobot.com, contact@shadowrobot.com
"""

import roslib; roslib.load_manifest('sr_control_gui')
import rospy
from scipy.optimize import fmin  
from cyberglove_library import Cyberglove

class MappingConfiguration:
    def __init__(self, glove_data = [], hand_data = [], description = ""):
        self.glove_data = glove_data
        self.hand_data = hand_data
        self.description = description
        
    def copy(self):
        tmp = MappingConfiguration(self.glove_data, self.hand_data, self.description)
        return tmp

class MappingMinimizer:
    def __init__(self, verbose = 1):
        rospy.init_node("cyberglove_mapper_minimizer")
        #connect to the cyberglove
        self.cyberglove = Cyberglove()
        
        self.verbose = verbose
        
        glove_sensors = ["G_ThumbRotate", "G_ThumbMPJ", "G_ThumbIJ", "G_ThumbAb", "G_IndexMPJ", "G_IndexPIJ", 
                         "G_IndexDIJ", "G_MiddleMPJ", "G_MiddlePIJ", "G_MiddleDIJ", "G_MiddleIndexAb", "G_RingMPJ", 
                         "G_RingPIJ", "G_RingDIJ", "G_RingMiddleAb", "G_PinkieMPJ", "G_PinkiePIJ", "G_PinkieDIJ", 
                         "G_PinkieRingAb", "G_PalmArch", "G_WristPitch", "G_WristYaw"]
        
        index = 0        
        self.glove_name_index_map = {} 
        for sensor in glove_sensors:
            self.glove_name_index_map[sensor] = index
            index += 1
        
        # fill the table containing all the joints
        hand_joints = ["TH1","TH2","TH3","TH4","TH5","FF0","FF3","FF4","MF0","MF3","MF4","RF0","RF3","RF4","LF0",
                       "LF3","LF4","LF5","WR1","WR2"]
        
        index = 0        
        self.hand_name_index_map = {} 
        for sensor in hand_joints:
            self.hand_name_index_map[sensor] = index
            index += 1
        
        self.thumb_glove = { "G_ThumbRotate": 0,
                             "G_ThumbMPJ":    1,
                             "G_ThumbAb":     2 }
        
        self.thumb_hand = { "TH2":           0,
                            "TH4":           1,
                            "TH5":           2 }
                
        self.configurations = []
        self.initialise_configurations()
        
        self.simplex_iteration_index = 0
        
        if(self.verbose == 1):
            for conf in self.configurations:
                print "-------"
                print conf.description
                print conf.glove_data
                print conf.hand_data
            print "-------"
            print ""
            
        self.minerror = 1000000
        self.maxerror = 0
                
    def initialise_configurations(self):
        configuration = MappingConfiguration()
        
        ###
        # First configuration
        configuration.description = "All fingers and thumb are curved maximum in a punch form ... "
        configuration.glove_data = []
        #corresponding data for the hand
        configuration.hand_data = [[30.0, 52.0, 5.0],
                                   [30.0, 43.0, 0.0],
                                   [30.0, 68.0, 16.0]
                                   ]
                
        self.configurations.append(configuration.copy())
        
        ###
        # Second configuration
        configuration.description = "Thumb touching first finger tip ... "
        configuration.glove_data = []
        #corresponding data for the hand
        configuration.hand_data = [[30.0, 54.0, -5.0],
                                   [19.0, 58.0, 28.0]]
                
        self.configurations.append(configuration.copy())
        
        ###
        # Third configuration
        configuration.description = "Thumb touching middle finger tip ... "
        configuration.glove_data = []
        #corresponding data for the hand
        configuration.hand_data = [[30.0, 64.0, 9.0],
                                   [1.0, 68.0, 29.0]]
                
        self.configurations.append(configuration.copy())
        
        ###
        # Fourth configuration
        configuration.description = "Thumb touching ring finger tip ... "
        configuration.glove_data = []
        #corresponding data for the hand
        configuration.hand_data = [[30.0, 75.0, 19.0],
                                   [1.0, 75.0, 42.0]]
        
        self.configurations.append(configuration.copy())
        
        ###
        # Fifth configuration
        configuration.description = "Thumb touching little finger tip ... "
        configuration.glove_data = []
        #corresponding data for the hand
        configuration.hand_data = [[30.0, 75.0, 3.0],
                                   [1.0, 75.0, 35.0]]      
                
        self.configurations.append(configuration.copy())
        
        ###
        # Sixth configuration
        configuration.description = "Thumb touching all the finger tips ... "
        configuration.glove_data = []
        #corresponding data for the hand
        configuration.hand_data = [[-30.0, 75.0, 60.0],
                                   [-7.0, 63.0, 60.0]]       
                
        self.configurations.append(configuration.copy())

        ###
        # Seventh configuration
        configuration.description = "hand flat, thumb relaxed along the fingers "
        configuration.glove_data = []
        #corresponding data for the hand
        configuration.hand_data = [[0.0, 0.0, -60.0]]       
                
        self.configurations.append(configuration.copy())
        
        ###
        # Eigth configuration
        configuration.description = "hand flat, thumb opened "
        configuration.glove_data = []
        #corresponding data for the hand
        configuration.hand_data = [[0.0, 75.0, -60.0]]         
                
        self.configurations.append(configuration.copy())

    def evaluation_function(self, matrix):
        """
        The error is computed by adding the square error for each configuration.
        """
        error = 0
        
        #rebuild the matrix as a matrix, not a vector (the implementation of the simplex in scipy works on vectors
        matrix_tmp = []
        for i in range(len(self.thumb_glove)):
            line = []
            for j in range(len(self.thumb_hand)):
                line.append(matrix[i*len(self.thumb_hand)+j])
            matrix_tmp.append(line)
            
        matrix = matrix_tmp
        computed_vector_hand = []
        
        for config in self.configurations:
            #get the hand vectors from the glove data
            for vec_glove in config.glove_data:
                computed_vector_hand.append(self.multiply(vec_glove, matrix))
            
            #compute the square error
            for vec_comp, vec_hand in zip(computed_vector_hand, config.hand_data):
                for computed_hand_data in vec_comp:
                    for hand_data in vec_hand:
                        error += (computed_hand_data-hand_data)*(computed_hand_data-hand_data)
                
        if self.verbose == 1:
            print "error: "+str(error)
        
        if error < self.minerror:
            self.minerror = error
        
        if error > self.maxerror:
            self.maxerror = error
            
        return error
        
    def multiply(self, vector_glove, matrix):
        """
        multiply the vector by the matrix. Returns a vector.
        """
        vector_hand = []
        index_col = 0
        print vector_glove
        print matrix
        for glove_data in vector_glove:
            data = 0
            for line in matrix:
                data += (glove_data*line[index_col])
            vector_hand.append(data)
            index_col += 1
        
        return vector_hand
    
    def callback(self, xk):
        """
        Function called at each iteration
        """
        self.simplex_iteration_index += 1
        if self.simplex_iteration_index % 50 == 0:
            print "-------------------------"
            print "iteration number: "+ str(self.simplex_iteration_index)
            #print xk
        
    def minimize(self):
        start = []
        for i in range(0, len(self.thumb_hand)):
            line = []
            for j in range(0, len(self.thumb_glove)):
                line.append(0.1)
            start.append(line)
        
        
        xopt = fmin(self.evaluation_function, start, callback=self.callback, maxiter = 5000)
        
        #rebuild the result as a matrix, not a vector (the implementation of the simplex in scipy works on vectors
        output = []
        for i in range(len(self.thumb_glove)):
            line = []
            for j in range(len(self.thumb_hand)):
                line.append(xopt[i*len(self.thumb_hand)+j])
            output.append(line)
        
        print "min error: "+ str(self.minerror)
        print "max error: "+ str(self.maxerror)
        
        return output
    
    def write_full_mapping(self, output_path = "../../../param/GloveToHandMappings"):
        """
        Writes the mapping matrix to a file:
        the glove values are the lines, the hand values are the columns
        """
        
        #initialise the matrix with 0s
        mapping_matrix = []
        for i in range(0, len(self.glove_name_index_map)):
            line = []
            for j in range(0, len(self.hand_name_index_map)):
                line.append(0.0)
            mapping_matrix.append(line)
        
        # TH3 is always 0, RF4 as well
        #fill the matrix with the known values + th1 (all except the thumb)
        mapping_matrix[self.glove_name_index_map["G_IndexDIJ"]][self.hand_name_index_map["FF0"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_IndexPIJ"]][self.hand_name_index_map["FF0"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_IndexMPJ"]][self.hand_name_index_map["FF3"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_MiddleIndexAb"]][self.hand_name_index_map["FF4"]] = -1.0
        
        mapping_matrix[self.glove_name_index_map["G_MiddleDIJ"]][self.hand_name_index_map["MF0"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_MiddlePIJ"]][self.hand_name_index_map["MF0"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_MiddleMPJ"]][self.hand_name_index_map["MF3"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_RingMiddleAb"]][self.hand_name_index_map["MF4"]] = -1.0
        
        mapping_matrix[self.glove_name_index_map["G_RingDIJ"]][self.hand_name_index_map["RF0"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_RingPIJ"]][self.hand_name_index_map["RF0"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_RingMPJ"]][self.hand_name_index_map["RF3"]] = 1.0
        
        mapping_matrix[self.glove_name_index_map["G_PinkieDIJ"]][self.hand_name_index_map["LF0"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_PinkiePIJ"]][self.hand_name_index_map["LF0"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_PinkieMPJ"]][self.hand_name_index_map["LF3"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_PinkieRingAb"]][self.hand_name_index_map["LF4"]] = -1.0
        mapping_matrix[self.glove_name_index_map["G_PalmArch"]][self.hand_name_index_map["LF5"]] = 1.0
        
        mapping_matrix[self.glove_name_index_map["G_WristPitch"]][self.hand_name_index_map["WR1"]] = 1.0
        mapping_matrix[self.glove_name_index_map["G_WristYaw"]][self.hand_name_index_map["WR2"]] = 1.0
        
        mapping_matrix[self.glove_name_index_map["G_ThumbIJ"]][self.hand_name_index_map["TH1"]] = 1.0
        
        #mapping_matrix[self.glove_name_index_map["G_ThumbAb"]][self.hand_name_index_map["TH4"]] = 1.0
        #mapping_matrix[self.glove_name_index_map["G_ThumbRotate"]][self.hand_name_index_map["TH5"]] = 1.0
        #mapping_matrix[self.glove_name_index_map["G_ThumbMPJ"]][self.hand_name_index_map["TH2"]] = 1.0
        
        
        #compute the best mapping for the thumb except th1
        thumb_mapping = self.minimize()
        
        #fill the matrix with the thumb values computed with the simplex
        for glove_name in self.thumb_glove.keys():
            for hand_name in self.thumb_hand.keys():
                #indexes in the computed thumb mapping matrix
                tmp_index_glove = self.thumb_glove[glove_name]
                tmp_index_hand = self.thumb_hand[hand_name]
                
                #retrieve the mapping value for those sensors
                mapping_value = thumb_mapping[tmp_index_glove][tmp_index_hand]
                
                #indexes for the whole mapping matrix
                final_index_glove = self.glove_name_index_map[glove_name]
                final_index_hand = self.hand_name_index_map[hand_name]
                
                #update matrix
                mapping_matrix[final_index_glove][final_index_hand] = mapping_value
               #write the matrix to a file    
        file = open(output_path, "w")
        for line in mapping_matrix:
            for col in line:
                file.write(" "+str(col) )
            file.write("\n")
            
        file.close()

    def record(self, config):
        for i in range(0, 2):
            raw_input(str(i) + ": " + config.description)
            vector_data = [0] * len(self.thumb_glove)
            for sensor_name in self.thumb_glove.keys():
                data = self.cyberglove.read_calibrated_average_value(sensor_name)
                vector_data[self.thumb_glove[sensor_name]] = data
            config.glove_data.append(vector_data)
            
        if(self.verbose == 1):
            print "read values:" 
            print config.glove_data
                

##############
#    MAIN    #
##############
def main():
    cyber_mapper = MappingMinimizer()
    
    for config in cyber_mapper.configurations:
        cyber_mapper.record(config)
    
    cyber_mapper.write_full_mapping()
    
    return 0


# start the script
if __name__ == "__main__":
    main()