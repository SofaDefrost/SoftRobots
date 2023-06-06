# Requirements

- python3
- SOFA with SofaPython3 plugin
- Ros2

# How To

To run the example:
- in a first terminal, "source ros2", start `python recv.py`
- in a second terminal, "source ros2", `runSofa test_sofaros.py`

The example is a simple scene composed of two MechanicalObject, one can be manipulated 
with mouse interaction. The data position of this MechanicalObject is published as ros topic "/simulation/sender/position" using a dedicated RosSender (python script controller)

The recv is subscribed to this published ros topic "/simulation/sender/position". Each time a data 
arrive, the recv script just re-emit it into a second ros topic called "/animation/receiver/position".

A RosReceiver is added in the sofa scene to listen to this second topic and propagate the values to 
the second mechanical object. 



