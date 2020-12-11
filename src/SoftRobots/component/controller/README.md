# GameTrakController

1- Install HIDAPI : https://github.com/signal11/hidapi  
1.1- Download the archive

1.2- **On Linux**

    sudo apt-get install libudev-dev libusb-1.0-0-dev libfox-1.6-dev
    sudo apt-get install autotools-dev autoconf automake libtool
    cd HIDAPI_DIR
    ./bootstrap
    ./configure
    make
    sudo make install

1.2- **On Mac**

    sudo port install fox
    cd HIDAPI_DIR
    ./bootstrap
    ./configure
    make
    sudo make install

1.2- **On Windows**, refer to the README file provided with the library

2- In the cmake gui, enable: SOFTROBOTS_GAMETRAKCONTROLLER = true  
3- If you're working on Windows please improve this README

# CommunicationController

1- Install ZMQ. Please ensure than your libzmq version is >= 3.2

1.1- **On Linux**

    #Debian/Ubuntu
    sudo apt-get install libzmq3-dev
    #Fedora
    sudo dnf install zeromq-devel

1.1- **On Mac** (missing)

1.1- **On Windows**, download the Windows source of libzmq and build using Visual Studio.	Put a copy of zmq.hpp from the cppzmq project (github) in the include folder of libzmq.

2- In the cmake gui, enable: SOFTROBOTS_COMMUNICATIONCONTROLLER = true  
3- If you're working or Mac please improve this README
