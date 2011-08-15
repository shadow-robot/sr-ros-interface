#!/bin/bash

for i in {11..19} ; do
    echo "Flashing motor: $i"
    rosservice call SimpleMotorFlasher `rospack find sr_external_dependencies`/compiled_firmware/released_firmware/simplemotor.hex $i;
done
