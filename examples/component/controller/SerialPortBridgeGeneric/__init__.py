# -*- coding: utf-8 -*-
"""

The “SerialPortBridgeGeneric” component is used to send data (force, displacement, pressure…) through the usb port. Usally used to send data to an Arduino card to control the real robot.

.. image:: http://project.inria.fr/softrobot/files/2016/05/Diamond2.png
   :width: 200px
.. image:: http://project.inria.fr/softrobot/files/2016/05/Diamond3.png
   :width: 200px
.. image:: http://project.inria.fr/softrobot/files/2016/05/Diamond1-300x232.png
   :width: 200px



Example
*******

.. sourcecode:: python

    	rootNode.addObject('SerialPortBridgeGeneric', name="serial", port="/dev/ttyACM0", baudRate="115200", size="5", listening=True, header=255, packetOut=...)

Data fields
***********

.. list-table::
   :header-rows: 1
   :widths: auto

   * - Required
     - Description
   * - **port**
     - Serial port name.
   * - **baudRate**
     - Transmission speed.
   * - **packetOut**
     - Data to send: vector of unsigned char, each entry should be an integer between 0 and (header-1) <= 255. The value of 'header' will be sent at the beginning of the sent data, enabling to implement a header research in the 'receiving' code, for synchronization purposes.
   * - **header**
     - Vector of unsigned char. Only one value is espected, two values if splitPacket = 1.
   * - **size**
     - Size of the arrow to send. Use to check sentData size. Will return a warning if sentData size does not match this value.
   * - **receive**
     - If true, will read from serial port (timeOut = 10ms).

.. list-table::
   :header-rows: 1
   :widths: auto

   * - Optional
     - Description
   * - **precise**
     - If true, will send the data in the format [header[0],[MSB,LSB]*2*size].
   * - **splitPacket**
     - If true, will split the packet in two for lower error rate (only in precise mode), data will have the format [header[0],[MSB,LSB]*size],[header[1],[MSB,LSB]*size].
   * - **redundancy**
     - Each packet will be send that number of times (1=default).

.. list-table::
   :header-rows: 1
   :widths: auto

   * - Properties
     - Description
   * - **packetIn**
     - Read only. Data received: vector of unsigned char, each entry should be an integer between 0 and (header-1) <= 255.

"""
