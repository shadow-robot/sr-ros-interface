#!/bin/bash

START_PATH=`pwd`

cd "../include/external"

function checkout_or_update {
    DIR_NAME=${1}
    SVN_PATH=${2}
    if [ -d ${DIR_NAME} ] ; then
	chmod -R a+w ${DIR_NAME}
	echo "Updating ${DIR_NAME}"
	pushd ${DIR_NAME}
	svn up
	popd

    #removes write access
	chmod -R a-w ${DIR_NAME}
    else
	echo "Checking out ${SVN_PATH} in ${DIR_NAME}"
	svn co ${SVN_PATH}

    #removes write access
	chmod -R a-w ${DIR_NAME}
    fi
}

checkout_or_update 0220_palm_edc svn://Pericles/Pic32/trunk/nodes/0220_palm_edc
checkout_or_update simplemotor-bootloader svn://pericles/EDC/simplemotor-bootloader/

cd ${START_PATH}

