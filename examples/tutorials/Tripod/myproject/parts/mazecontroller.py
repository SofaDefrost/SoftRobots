import Sofa.Core
from splib3.numerics import Quat
from splib3.constants import Key


class MazeController(Sofa.Core.Controller):
    """This controller moves the goal position when the inverse control is activated
    """

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "MazeController"
        self.goalNode = args[0]
        self.anglePlanningTable = args[1]
        self.activated = args[2]
        self.time = 0

        self.mo = self.goalNode.goalMO
        self.y = self.mo.position[0][1]
        self.theta_x = 0
        self.theta_z = 0
        self.dy = 0

        self.currentKey = 0
        self.tableKeyPoints = []
        self.endAnimation = False
        self.initTrajectoryByKeyPoint()

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.I:
            self.activated = True

    def initTrajectoryByKeyPoint(self):
        working_y = 20
        timeBetweenKeyP = 0.5

        time = 0
        self.tableKeyPoints.append([0, 0, 0, 0])  # Time dy theta_x theta_z WARNING = Suppose time ordering

        for t in self.anglePlanningTable:
            time = time + timeBetweenKeyP
            self.tableKeyPoints.append([time, working_y, t[0], t[1]])
        return

    def defineTrajectoryByKeyPoint(self, time):

        if self.endAnimation:
            return

        if time > self.tableKeyPoints[self.currentKey + 1][0]:
            self.currentKey += 1

        if self.currentKey == len(self.tableKeyPoints) - 1:
            self.currentKey -= 1
            self.endAnimation = True
            return

        interpolFactor = (self.tableKeyPoints[self.currentKey + 1][0] - time) / (self.tableKeyPoints[self.currentKey + 1][0] - self.tableKeyPoints[self.currentKey][0])

        self.dy = interpolFactor * self.tableKeyPoints[self.currentKey][1] + (1 - interpolFactor) * self.tableKeyPoints[self.currentKey + 1][1]
        self.theta_x = interpolFactor * self.tableKeyPoints[self.currentKey][2] + (1 - interpolFactor) * self.tableKeyPoints[self.currentKey + 1][2]
        self.theta_z = interpolFactor * self.tableKeyPoints[self.currentKey][3] + (1 - interpolFactor) * self.tableKeyPoints[self.currentKey + 1][3]

    def onAnimateBeginEvent(self, e):
        dt = self.goalNode.getRoot().dt.value
        if self.activated:
            self.time = self.time + dt
            self.defineTrajectoryByKeyPoint(self.time)
            pos = [0.0,
                   self.y + self.dy,
                   0.0]
            quat = Quat.createFromEuler([self.theta_x, 0., self.theta_z])
            self.mo.rest_position = [[pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]]

