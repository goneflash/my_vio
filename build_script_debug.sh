#!/bin/bash
if [ -d build ]; then
  rm -rf build
fi
mkdir build
cd build
cmake .. -DUSE_TEST=ON -DCMAKE_BUILD_TYPE=Debug
make -j8
make test


