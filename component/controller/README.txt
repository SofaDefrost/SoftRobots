#################################################################################
For the GameTrakController:

1- Install HIDAPI : https://github.com/signal11/hidapi

	1.1- Download the archive	

	#ON LINUX
	1.2- 
	    sudo apt-get install libudev-dev libusb-1.0-0-dev libfox-1.6-dev
	    sudo apt-get install autotools-dev autoconf automake libtool
	    cd HIDAPI_DIR
	    ./bootstrap
	    ./configure
	    make
            sudo make install

	#ON MAC
	1.2- 
	    sudo port install fox
	    cd HIDAPI_DIR
	    ./bootstrap
	    ./configure
	    make
            sudo make install

	#ON WINDOWS
	Refer to the README file provided with the library
		
2- In the cmake gui, enable: SOFTROBOTS_GAMETRAKCONTROLLER = true
3- If you're working on Windows or OSX please improve this README

#################################################################################
For the CommunicationController:

1- Install ZMQ 	
	Please ensure than your libzmq version is >= 3.2
	#ON LINUX
	1.1- 
	    #Debian/Ubuntu
	    sudo apt-get install libzmq3-dev
            #Fedora 
	    sudo dnf install zeromq-devel

	#ON MAC
	1.1- ...

	#ON WINDOWS
	1.1- ...
		
2- In the cmake gui, enable: SOFTROBOTS_COMMUNICATIONCONTROLLER = true
3- If you're working on Windows or OSX please improve this README



