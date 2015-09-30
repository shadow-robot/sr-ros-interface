#!/bin/bash

for file in `find ../../ -name "*.md"`; do mkdir -p shadow_robot/docs/`dirname ${file}` ; cp ${file} shadow_robot/docs/`dirname ${file}`/ ; done

