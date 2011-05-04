#!/bin/bash

START_PATH=`pwd`

cd "../external"

if [ -d 0220_palm_edc ] ; then
    chmod -R a+w 0220_palm_edc
    echo "Updating 0220_palm_edc"
    pushd 0220_palm_edc
    svn up
    popd

    #removes write access
    chmod -R a-w 0220_palm_edc
else
    echo "Checking out 0220_palm_edc"
    svn co svn://Pericles/Pic32/trunk/nodes/0220_palm_edc

    #removes write access
    chmod -R a-w 0220_palm_edc
fi

cd ${START_PATH}