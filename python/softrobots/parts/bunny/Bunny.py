import os
from stlib.physics.deformable import ElasticMaterialObject
from softrobots.actuators import PneumaticCavity
from stlib.physics.constraints import FixedBox

meshpath = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def createBunny(Node, Translation=[0,0,0], ControlType='PressureConstraint', Name='Bunny', InitialValue=0.0001, YoungModulus=18000):

    #Bunny
    #BoxROICoordinates=[-5, -6, -5,  5, -4.5, 5] + [Translation,Translation]
    BoxROICoordinates=[-5 + Translation[0], -6 + Translation[1], -5 + Translation[2],  5 + Translation[0], -4.5 + Translation[1], 5 + Translation[2]] 
    Bunny = ElasticMaterialObject(name=Name,
                                  attachedTo=Node,
                                  volumeMeshFileName=meshpath+'Hollow_Stanford_Bunny.vtu',
                                  surfaceMeshFileName=meshpath+'Hollow_Bunny_Body_Cavity.obj',                                                
                                  youngModulus=YoungModulus,
                                  withConstrain=True,
                                  totalMass=0.5,
                                  translation=Translation
                                  )

    FixedBox(Bunny, doVisualization=True, atPositions=BoxROICoordinates)
       

    if ControlType == 'PressureConstraint':
        cavity = PneumaticCavity(name='Cavity',attachedAsAChildOf=Bunny,surfaceMeshFileName=meshpath+'Hollow_Bunny_Body_Cavity.obj',valueType='pressureGrowth', initialValue=InitialValue, translation=Translation)
    elif ControlType=='VolumeConstraint':
        cavity = PneumaticCavity(name='Cavity',attachedAsAChildOf=Bunny,surfaceMeshFileName=meshpath+'Hollow_Bunny_Body_Cavity.obj',valueType='volumeGrowth', initialValue=InitialValue, translation=Translation)
    
    BunnyVisu = Bunny.createChild('visu')
    BunnyVisu.createObject('TriangleSetTopologyContainer', name='container')
    BunnyVisu.createObject('TriangleSetTopologyModifier')
    BunnyVisu.createObject('TriangleSetTopologyAlgorithms', template='Vec3')
    BunnyVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3')
    BunnyVisu.createObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container", output="@container")
    BunnyVisu.createObject('OglModel', template='ExtVec3f', color='0.3 0.2 0.2 0.6', translation=Translation)
    BunnyVisu.createObject('IdentityMapping')
    return Bunny
                
                
