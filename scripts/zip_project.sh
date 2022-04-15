#!/bin/bash

if [ -n "$1" ]
then
    scripts/update_submodules.sh

    if [ -e "./$1.zip" ]
    then
        echo "delete $1.zip"
        rm "./$1.zip"
    fi
    echo "zip" $1
    zip -y --exclude=build/* --exclude=build_* --exclude=cmake-build-* --exclude=doc/doxygen/* --exclude=tests/* -x default.log.* *.git* *.idea* *cpp_types* *c_types* *py_types* CMakeLists.txt.* *.user */.* ./.* -r "$1.zip" ./
else
    echo "Input arg with archive name: ./zip_project.sh <archive_name>"
fi
