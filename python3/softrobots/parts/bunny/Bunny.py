import os
from stlib.physics.deformable import ElasticMaterialObject
from softrobots.actuators import PneumaticCavity
from stlib.physics.constraints import FixedBox

meshpath = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def Bunny(Node, translation=[0,0,0], controlType='PressureConstraint', name='Bunny', initialValue=0.0001, youngModulus=18000):

    #Bunny
    #boxROICoordinates=[-5, -6, -5,  5, -4.5, 5] + [translation,translation]
    boxROICoordinates=[-5 + translation[0], -6 + translation[1], -5 + translation[2],  5 + translation[0], -4.5 + translation[1], 5 + translation[2]]
    Bunny = ElasticMaterialObject(name=name,
                                  attachedTo=Node,
                                  volumeMeshFileName=meshpath+'Hollow_Stanford_Bunny.vtu',
                                  surfaceMeshFileName=meshpath+'Hollow_Bunny_Body_Cavity.obj',
                                  youngModulus=youngModulus,
                                  withConstrain=True,
                                  totalMass=0.5,
                                  translation=translation
                                  )

    FixedBox(Bunny, doVisualization=True, atPositions=boxROICoordinates)


    if controlType == 'PressureConstraint':
        cavity = PneumaticCavity(name='Cavity',attachedAsAChildOf=Bunny,surfaceMeshFileName=meshpath+'Hollow_Bunny_Body_Cavity.obj',valueType='pressureGrowth', initialValue=initialValue, translation=translation)
    elif controlType=='VolumeConstraint':
        cavity = PneumaticCavity(name='Cavity',attachedAsAChildOf=Bunny,surfaceMeshFileName=meshpath+'Hollow_Bunny_Body_Cavity.obj',valueType='volumeGrowth', initialValue=initialValue, translation=translation)

    BunnyVisu = Bunny.addChild('visu')
    BunnyVisu.addObject('TriangleSetTopologyContainer', name='container')
    BunnyVisu.addObject('TriangleSetTopologyModifier')
    BunnyVisu.addObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container", output="@container")
    BunnyVisu.addObject('OglModel', color=[0.3, 0.2, 0.2, 0.6], translation=translation)
    BunnyVisu.addObject('IdentityMapping')
    return Bunny
