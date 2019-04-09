Install ros,
git clone https://github.com/njanirudh/Aruco_Tracker

To run the example:
- source it,
- in a first terminal, "source ros", start roscore
- in a second terminal, "source ros", start python recv.py
- in a third terminal, "source ros", runSofa test_sofaros.py

The example is a simple scene composed of two MechanicalObject, one can be manipualted 
with mouse interaction. The data position of this MechanicalObject is published as ros topic "/simulation/sender/position" using a dedicated RosSender (python script controller)

The recv is subscribed to this published ros topic "/simulation/sender/position". Each time a data 
arrive, the recv script just re-emit it into a second ros topic called "/animation/receiver/position".

A RosReceiver is added in the sofa scene to listen to this second topic and propagate the values to 
the second mechanical object. 



