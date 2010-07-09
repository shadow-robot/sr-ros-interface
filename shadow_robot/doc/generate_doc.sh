#!/bin/bash

cd `rospack find rosdoc`

./rosdoc shadow_robot sr_hand sr_control_gui sr_display sr_remappers cyberglove

cd `rosstack find shadow_robot`

echo cp -r `rospack find rosdoc`/doc doc/rosdoc/
cp -r `rospack find rosdoc`/doc/* doc/rosdoc/

echo "Doc generated and copied to doc/doxygen in shadow_robot stack."
