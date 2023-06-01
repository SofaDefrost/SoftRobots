# -*- coding: utf-8 -*-
"""

The **CommunicationController** component can be used to send data from a simulation to another using ZMQ library. To use this component you need to compile SOFA with the option **SOFTROBOTS_COMMUNICATIONCONTROLLER** enabled in cmake, and install the ZMQ library:

1.1- On Linux

#Debian/Ubuntu

`sudo apt-get install libzmq3-dev`

#Fedora

`sudo dnf install zeromq-devel`

1.1- On MacOS (missing)

1.1- On Windows, download the Windows source of libzmq and build using Visual Studio. Put a copy of zmq.hpp from the cppzmq project (github) in the include folder of libzmq.

2- In the cmake gui, enable: SOFTROBOTS_COMMUNICATIONCONTROLLER = true

3- Compile SOFA

In this directory ("SoftRobots/examples/component/controller/CommunicationController") you will find one example showing how to use the component:

- **SimulationDirect_Receiver.py** : Soft actuated accordion, direct problem
- **SimulationInverse_Sender.py** : Soft actuated accordion, inverse problem

Below is a video of the simulations running simultaneously and with a communication between them.

.. raw:: html

   <p align=center><iframe src="https://www.youtube.com/embed/o1GPLb040oA" width="560" height="315" frameborder="0" allowfullscreen="allowfullscreen"></iframe></p>


Example
*******

SimulationDirect_Receiver.py:

.. sourcecode:: python

        #For local communication
        accordion.addObject('CommunicationController', name="sub", listening='1',
							 job="receiver", port="5558", nbDataField="4", pattern="0")
        #Between two different computers, specify the ip adress of the sender
        #accordion.addObject('CommunicationController', name="sub", listening='1',
							 job="receiver", port="5558", nbDataField="4", ip="...")

SimulationInverse_Sender.py:

.. sourcecode:: python

    	accordion.addObject('CommunicationController', listening='1', job="sender", port="5558", 					nbDataField="4", pattern="0",
				data1="@cavity/pressure.volumeGrowth",
				data2="@cables/cable1.displacement",
				data3="@cables/cable2.displacement",
				data4="@cables/cable3.displacement")


Data fields
***********

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Required
     - Description
   * - **job**
     - If unspecified, the default value is sender.
   * - **pattern**
     - Pattern used for communication. publish/subscribe: Messages sent are distributed in a fan out fashion to all connected peers. Never blocks. request/reply: Message sent are waiting for reply. Allows only an alternating sequence of send/reply calls. Default is publish/subscribe. WARNING: the pattern should be the same for both sender and receiver to be effective.   
   * - **nbDataField**
     - Number of field 'data' the user want to send or receive. Default value is 1.
   * - **data**
     - Data to send or receive.
   * - **port**
     - Default value 5556.

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Optional
     - Description
   * - **HWM**
     - If publisher, you can define the High Water Mark which is a hard limit on the maximum number of outstanding messages shall queue in memory. Default 0 (means no limit).
   * - **ip**
     - IP adress of the sender. No given adress will set up a local communication.
   * - **atBeginAnimationStep**
     - If true, will send or receive datas at begin of the animation step (if false, at end of the animation step). Default true.
   * - **beginAt**
     - Time step value to start the communication at.
   * - **timeOut**
     - Set time out (in ms) before killing the communication. Default is 3000ms, 0 means no time out.

"""


