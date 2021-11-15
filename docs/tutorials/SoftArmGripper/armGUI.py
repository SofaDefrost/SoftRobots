import Sofa
import Sofa.Core

import tkinter
import threading
import params


class App(threading.Thread):

    nbCables = params.Arm.nbSection * 3

    def __init__(self, robot, initCables=[0.0] * nbCables):
        threading.Thread.__init__(self)
        self.daemon = True
        self.start()
        self.robot = robot

        self.cablesInit = [0.0] * self.nbCables
        for i in range(self.nbCables):
            self.cablesInit[i] = initCables[i]

    def reset(self):
        for i in range(self.nbCables):
            self.cables[i].set(self.initCables[i])

    def callback(self):
        self.root.quit()

    def run(self):
        self.root = tkinter.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)
        self.root.title("Robot Controller Interface")

        self.cables = []
        for i in range(self.nbCables):
            self.cables.append(tkinter.DoubleVar())

        tkinter.Label(self.root, text="Cable displacement").grid(row=0, columnspan=self.nbCables)

        for i in range(self.nbCables):
            tkinter.Scale(self.root, variable=self.cables[i], resolution=1, length=400, from_=0, to=400, orient=tkinter.VERTICAL).grid(row=1, column=i)

        self.root.mainloop()


class ArmGUI(Sofa.Core.Controller):

    nbCables = params.Arm.nbSection * 3

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.arm = kwargs["arm"]
        self.app = App(self.arm, kwargs.get("initCables",[0.0] * self.nbCables))

        return

    def reset(self):
        self.app.reset()

    def onAnimateBeginEvent(self, event):

        cables = [0.0] * self.nbCables
        for i in range(self.nbCables):
            cables[i] = self.app.cables[i].get()

        for i in range(params.Arm.nbSection):
            self.arm.getChild('Section'+str(i+1)).Cables.cable0.value = [cables[i*3]]
            self.arm.getChild('Section'+str(i+1)).Cables.cable1.value = [cables[i*3+1]]
            self.arm.getChild('Section'+str(i+1)).Cables.cable2.value = [cables[i*3+2]]

        return
