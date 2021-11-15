import os


class Simulation:
    dt = 0.01
    gravity = [9810, 0, 0]
    inverse = False  # To allow the inverse mode you need the SoftRobots.Inverse plugins available in binaries
    armGUI = False  # To allow a GUI to control the robot in direct mode. Requires tkinter for python3

    path = os.path.dirname(os.path.abspath(__file__))


class Arm:
    # units are mm and kg
    youngModulus = 1e8
    massDensity = 5e-6

    nbSection = 3  # number of section
    nbRibs = 6  # number of ribs per section

    beamThickness = [4, 3, 2]  # should be the size of nbSection
    beamWidth = 9
    beamLength = 60
    beamHeight = [21 + ly for ly in beamThickness]
    angle = 0.75

    radius = 2
    tx = 8  # translation of the cables
    dist = 5  # distance between rigid dof and beam nodes


class Gripper:

    positions = [[0.01962421023828305, 0.025094419746546907, -0.0055],
                 [0.028859132368167926, 0.03691458105064774, -0.0055],
                 [0.038094054498052796, 0.04873474235474856, -0.0055],
                 [0.047328976627937676, 0.0605549036588494, -0.0055],
                 [0.05656389875782255, 0.07237506496295022, -0.0055],
                 [0.06025786760977651, 0.07710312948459057, -0.0034999999999999996],
                 [0.06148919056042782, 0.07867915099180402, -0.0014999999999999996],
                 [0.06148919056042782, 0.07867915099180402, 0.0005000000000000004],
                 [0.06025786760977651, 0.07710312948459057, 0.0025000000000000005],
                 [0.05656389875782255, 0.07237506496295022, 0.0045000000000000005],
                 [0.047328976627937676, 0.0605549036588494, 0.0045000000000000005],
                 [0.038094054498052796, 0.04873474235474856, 0.0045000000000000005],
                 [0.028859132368167926, 0.03691458105064774, 0.0045000000000000005],
                 [0.01962421023828305, 0.025094419746546907, 0.0045000000000000005]];
