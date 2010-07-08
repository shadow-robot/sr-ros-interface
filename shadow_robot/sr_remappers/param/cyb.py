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



