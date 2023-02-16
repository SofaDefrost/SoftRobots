from stlib3.scene import Node

parentNode = None
attachedToNode = None

def StringSensor(parentNode=None,
                 name="StringSensor",
                 cableGeometry=[[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                 rotation=[0.0,0.0,0.0],
                 translation=[0.0,0.0,0.0],
                 uniformScale=1.0):
    """One line documentation used in the summary

    Args:


    Structure:
        .. sourcecode:: qml

            Node : {
                    name : "StringSensor"
                    SensorEngine
            }

    Example:
        .. sourcecode::python


        def createScene(rootNode):
            from stlib3.scene import MainHeader
            MainHeader(rootNode)

            f = Node(rootNode, "Finger")
            StringSensor(f)

    """
    cable = Node(parentNode, "StringSensor")

    # This add a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    # the points where the cable is passing by.
    cable.addObject('MechanicalObject',
                    position=cableGeometry,
                    rotation=rotation, translation=translation, scale=uniformScale)

    cable.addObject("SensorEngine",
                        name="SensorEngine",
                        indices=range(len(cableGeometry)))

    # This add a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    # between the cable's DoFs and the parents's ones so that movements of the cable's DoFs will be mapped
    # to the finger and vice-versa;
    cable.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)

    return ss


def createScene(rootNode):
    from stlib3.scene import MainHeader
    MainHeader(rootNode)

    f = Node(rootNode, "Finger")
    StringSensor(f)
