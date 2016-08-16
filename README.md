# Motion tracking using accelerometer data

## Instructions

### Requirements
 - git
 - CMake 3.2+
 - C++11/14 compiler

### In Windows (MinGW)
To build:

    git clone <repository url> --recursive
    cd <repository folder>
    mkdir build && cd build
    cmake .. -G "MinGW Makefiles"
    cmake --build .

To run:

    ./myAHRS_accel.exe
