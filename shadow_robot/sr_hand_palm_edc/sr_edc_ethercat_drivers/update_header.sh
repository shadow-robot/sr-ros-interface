#!/bin/bash

START_PATH=`pwd`
cd external

if [ -d 0220_palm_edc ] ; then
    echo "Updating 0220_palm_edc"
    pushd 0220_palm_edc
    svn up
    popd
else
    echo "Checking out 0220_palm_edc"
    svn co svn://Pericles/Pic32/trunk/nodes/0220_palm_edc
fi

cd ${START_PATH}