#!/bin/bash
# This script must be run from the build folder.

executables=$(find . | grep -e "_benchmark$")

for executable in $executables; do
    $(echo $executable)
done