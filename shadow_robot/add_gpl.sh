#!/bin/bash

echo 'Adding the GPL header to the cpp files'

for cpp_file in  `find . -name *.hpp`; do
    echo "   -> "${cpp_file}

    a=5

    while read line
    do
	a=$(($a+1));
	#echo -e "$line \ n"
	sed -i "${a}i ${line}" ${cpp_file}
    done <gpl_licence_c.txt
    #sed -i  '5i \ *\n * blablalbla' $i

done
