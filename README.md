## Usage

1) git submodule update --init --recursive
2) mkdir build && cd build
3) cmake .. && make

If want to build tests, then use the following instead of 3) :
cmake .. -DUSE_TEST=ON && make

