#!/usr/bin/env bash

# Use this to build box2d on any system with a bash shell
rm -rf build
mkdir build
cd build
cmake -G'Unix Makefiles' -DCMAKE_BUILD_TYPE=Release -DBOX2D_BUILD_TESTBED=ON -DBOX2D_BUILD_UNIT_TESTS=OFF -DBOX2D_BUILD_DOCS=OFF ..
make -j
