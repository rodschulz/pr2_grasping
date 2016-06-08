#!/bin/bash

type=$1

# Remove previous builds
cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd ../../
rm -rf ./build
rm -rf ./devel
rm -rf ./install

# Configure with cmake
if [ "$type" == "-r" ] ; then
	echo "Configuring project for RELEASE"
	catkin_make -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
else
	echo "Configuring project for DEBUG"
	catkin_make -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
fi
