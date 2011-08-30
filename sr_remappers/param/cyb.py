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
file = open("cyberglovetoshadowhand.map")

first = True
matrix = []

for line in file.readlines():
    if first:
        first = False
        continue
    else:
        line = line.strip("\n")
        line = line.split(" ")
        matrix.append(line)

file.close()

matrix_t = []

for i in range(0, len(matrix[0])):
    matrix_t.append(range(0,len(matrix)))

for i in range(0, len(matrix)):
    for j in range(0, len(line)):
        matrix_t[j][i] = matrix[i][j]

file = open("cyberglovetoshadowhand_transposed.map", "w")
for line in matrix_t:
    for col in line:
        file.write(col+ " ")
    file.write("\n")
file.close()



