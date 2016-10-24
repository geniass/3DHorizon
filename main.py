from TI_IMU import TI_IMU
import serial
from SerialMock import SerialMock
import numpy as np
from MotionThread import MotionThread
from MotionProcess import MotionProcess
import queue
#from queue import Queue
from multiprocessing import Queue

from panda3d.core import loadPrcFile
from panda3d.core import AntialiasAttrib
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor


class MyApp(ShowBase):
    def __init__(self):
        loadPrcFile('./config.prc')

        ShowBase.__init__(self)

        self.render.setAntialias(AntialiasAttrib.MAuto)

        # prevents the camera being controlled by the mouse.
        # necessary because otherwise the camera goes completely mad if
        # camera.setPos is called intermittently
        self.disableMouse()

        # Load the environment model.
        self.scene = self.loader.loadModel("3dcrossmatrix.egg")
        # Reparent the model to render.
        self.scene.reparentTo(self.render)
        # Apply scale and position transforms on the model.
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)
        self.scene.analyze()
        self.scene.clearModelNodes()
        self.scene.flattenStrong()
        self.scene.analyze()

        self.pos = {'x': 0., 'y': 0., 'z': 0.}

        self.ser = serial.Serial('/dev/ttyACM0', 1000000)
        #self.ser = SerialMock()
        self.imu = TI_IMU(self.ser)

        self.motion_queue = Queue(maxsize=1)
        self.motion_thread = MotionProcess(self.imu, self.motion_queue)
        self.motion_thread.start()

        self.taskMgr.add(self.updateCamera, "updateCameraTask")

    def updateCamera(self, task):
        try:
            self.pos = self.motion_queue.get(block=False)
        except queue.Empty:
            pass
        else:
            self.camera.setHpr(0, self.pos['pitch'], self.pos['roll'])
            self.camera.setPos(0, -20.0, 3 + 10*self.pos['z'])
        return Task.cont

app = MyApp()
app.run()
