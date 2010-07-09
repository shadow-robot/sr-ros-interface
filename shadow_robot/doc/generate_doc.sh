#!/bin/bash

echo ""
echo "Generating rosdoc (html)"
echo ""

cd `rospack find rosdoc`

./rosdoc shadow_robot sr_hand sr_control_gui sr_display sr_remappers cyberglove

cd `rosstack find shadow_robot`

echo cp -r `rospack find rosdoc`/doc doc/rosdoc/
cp -r `rospack find rosdoc`/doc/* doc/rosdoc/

echo "---------" 
echo "Now generating the doc as a pdf using doxygen." 
echo "" 

cd `rosstack find shadow_robot`/doc/doxygen/
doxygen sr_hand.config
cd sr_hand/latex
make

cp refman.pdf ../../sr_hand.pdf

echo ""
echo "rosdoc generated and copied to doc/rosdoc in shadow_robot stack."
echo ""

echo ""
echo "The pdf doc for sr_hand has been generated in " `rosstack find shadow_robot`"/doc/doxygen/sr_hand.pdf"

