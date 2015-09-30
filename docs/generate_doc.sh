#!/bin/bash

PATH_TO_GENERATED_DOCS=source/generated

rm -rf ${PATH_TO_GENERATED_DOCS}
mkdir -p ${PATH_TO_GENERATED_DOCS}

for FILE in `find ../../ -name "*.md"`; do
    echo "--------_"
    FILE_BASENAME=`basename -s .md ${FILE}`
    PATH_TO_FILE=`dirname ${FILE}`
    echo ${PATH_TO_FILE} "  " ${FILE_BASENAME}
    STRIPPED_PATH_TO_FILE=`echo ${PATH_TO_FILE} | sed 's;../../;;g'`

    mkdir -p ${PATH_TO_GENERATED_DOCS}/${STRIPPED_PATH_TO_FILE};

    PATH_TO_RST=${PATH_TO_GENERATED_DOCS}/${STRIPPED_PATH_TO_FILE}/${FILE_BASENAME}.rst

    echo "Copying ${FILE} to ${PATH_TO_RST}"
    pandoc --from=markdown --to=rst --output=${PATH_TO_RST} ${FILE}
 done
