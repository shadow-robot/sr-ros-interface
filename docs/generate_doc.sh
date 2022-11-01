#!/bin/bash
# Copyright 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

PATH_TO_GENERATED_DOCS=source/generated

rm -rf ${PATH_TO_GENERATED_DOCS}
mkdir -p ${PATH_TO_GENERATED_DOCS}

# copy md files and translate to rst on the fly
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

# copy rst files
for FILE in `find ../../ -name "*.rst" | grep -v CHANGELOG | grep -v shadow_robot/docs/source`; do
    echo "--------_"
    FILE_BASENAME=`basename ${FILE}`
    PATH_TO_FILE=`dirname ${FILE}`
    echo ${PATH_TO_FILE} "  " ${FILE_BASENAME}
    STRIPPED_PATH_TO_FILE=`echo ${PATH_TO_FILE} | sed 's;../../;;g'`

    mkdir -p ${PATH_TO_GENERATED_DOCS}/${STRIPPED_PATH_TO_FILE};

    PATH_TO_RST=${PATH_TO_GENERATED_DOCS}/${STRIPPED_PATH_TO_FILE}/${FILE_BASENAME}

    echo "Copying ${FILE} to ${PATH_TO_RST}"
    cp ${FILE} ${PATH_TO_RST}
 done
