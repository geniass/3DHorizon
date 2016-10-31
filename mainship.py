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
        self.scene = self.loader.loadModel("wave.egg)
        # Reparent the model to render.
        self.scene.reparentTo(self.render)
        # Apply scale and position transforms on the model.
        self.scene.setScale(1, 1, 1)
        self.scene.setPos(0.2, 42, 3)
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

                # Load and transform the Ship.
        self.shipActor = Actor("ship.egg")
        self.shipActor.setScale(1.75, 1.75, 1.75)
        self.shipActor.setPos(-4, 48, 9)
        self.shipActor.reparentTo(self.render)

        self.finActor = Actor("findark.egg")
        self.finActor.setScale(0.3, 0.3, 0.3)
        self.finActor.setPos(-1, 35, -2)
        self.finActor.reparentTo(self.render)

        self.fin1Actor = Actor("findark.egg")
        self.fin1Actor.setScale(0.2, 0.2, 0.2)
        self.fin1Actor.setPos(-12, 40, 5)
        self.fin1Actor.reparentTo(self.render)

        self.fin2Actor = Actor("findark.egg")
        self.fin2Actor.setScale(0.2, 0.2, 0.2)
        self.fin2Actor.setPos(-26, 60, 15)
        self.fin2Actor.reparentTo(self.render)

        self.fin3Actor = Actor("findark.egg")
        self.fin3Actor.setScale(0.18, 0.18, 0.18)
        self.fin3Actor.setPos(7, 64, 18)
        self.fin3Actor.reparentTo(self.render)


        ## SHIP ANIMATION
        ## Create intervals for the ship to move forward and reverse
        shipPosInterval1 = self.shipActor.posInterval(14,
                                                        Point3(-6, 48, 9),
                                                        startPos=Point3(6, 48, 9))
        shipPosInterval2 = self.shipActor.posInterval(14,
                                                        Point3(6, 48, 9),
                                                        startPos=Point3(-6, 48, 9))
        shipHprInterval1 = self.shipActor.hprInterval(2,
                                                        Point3(0, 0, 0),
                                                        startHpr=Point3(0, 0, 0))
        shipHprInterval2 = self.shipActor.hprInterval(2,
                                                        Point3(0, 0, 0),
                                                        startHpr=Point3(0, 0, 0))
 
        # Create and play the sequence that coordinates the intervals.
        self.shipSpeed = Sequence(shipPosInterval1,
                                  shipHprInterval1,
                                  shipPosInterval2,
                                  shipHprInterval2,
                                  name="shipSpeed")
        self.shipSpeed.loop()

        ## FIN ANIMATION
        ## Create intervals for the ship to move forward, rotato by 180 degrees and move foraward in the opposite direction.
        finPosInterval1 = self.finActor.posInterval(7,
                                                      Point3(-6, 35, -2),
                                                      startPos=Point3(8, 35, -2))
        finPosInterval2 = self.finActor.posInterval(7,
                                                      Point3(8, 35, -2),
                                                      startPos=Point3(-6, 35, -2))
        finHprInterval1 = self.finActor.hprInterval(3,
                                                      Point3(180, 0, 0),
                                                      startHpr=Point3(0, 0, 0))
        finHprInterval2 = self.finActor.hprInterval(3,
                                                      Point3(0, 0, 0),
                                                      startHpr=Point3(180, 0, 0))
 
        # Create and play the sequence that coordinates the intervals.
        self.finSpeed = Sequence(finPosInterval1,
                                  finHprInterval1,
                                  finPosInterval2,
                                  finHprInterval2,
                                  name="finSpeed")
        self.finSpeed.loop()

        ## FIN1 ANIMATION
        ## Create intervals for the ship to move forward, rotato by 180 degrees and move foraward in the opposite direction.
        fin1PosInterval1 = self.fin1Actor.posInterval(10,
                                                        Point3(-12, 40, 5),
                                                        startPos=Point3(2, 40, 5))
        fin1PosInterval2 = self.fin1Actor.posInterval(10,
                                                        Point3(2, 40, 5),
                                                        startPos=Point3(-12, 40, 5))
        fin1HprInterval1 = self.fin1Actor.hprInterval(3,
                                                        Point3(180, 0, 0),
                                                        startHpr=Point3(0, 0, 0))
        fin1HprInterval2 = self.fin1Actor.hprInterval(3,
                                                        Point3(0, 0, 0),
                                                        startHpr=Point3(180, 0, 0))
 
        # Create and play the sequence that coordinates the intervals.
        self.fin1Speed = Sequence(fin1PosInterval1,
                                  fin1HprInterval1,
                                  fin1PosInterval2,
                                  fin1HprInterval2,
                                  name="fin1Speed")
        self.fin1Speed.loop()

        ## FIN2 ANIMATION
        ## Create intervals for the ship to move forward, rotato by 180 degrees and move foraward in the opposite direction.
        fin2PosInterval1 = self.fin2Actor.posInterval(6,
                                                        Point3(-26, 60, 15),
                                                        startPos=Point3(4, 60, 15))
        fin2PosInterval2 = self.fin2Actor.posInterval(6,
                                                        Point3(4, 60, 15),
                                                        startPos=Point3(-26, 60, 15))
        fin2HprInterval1 = self.fin2Actor.hprInterval(2,
                                                        Point3(180, 0, 0),
                                                        startHpr=Point3(0, 0, 0))
        fin2HprInterval2 = self.fin2Actor.hprInterval(2,
                                                        Point3(0, 0, 0),
                                                        startHpr=Point3(180, 0, 0))
 
        # Create and play the sequence that coordinates the intervals.
        self.fin2Speed = Sequence(fin2PosInterval1,
                                  fin2HprInterval1,
                                  fin2PosInterval2,
                                  fin2HprInterval2,
                                  name="fin2Speed")
        self.fin2Speed.loop()

        ## FIN3 ANIMATION
        ## Create intervals for the ship to move forward, rotato by 180 degrees and move foraward in the opposite direction.
        fin3PosInterval1 = self.fin3Actor.posInterval(16,
                                                        Point3(-30, 64, 18),
                                                        startPos=Point3(17, 64, 18))
        fin3PosInterval2 = self.fin3Actor.posInterval(16,
                                                        Point3(17, 64, 18),
                                                        startPos=Point3(-30, 64, 18))
        fin3HprInterval1 = self.fin3Actor.hprInterval(5,
                                                        Point3(180, 0, 0),
                                                        startHpr=Point3(0, 0, 0))
        fin3HprInterval2 = self.fin3Actor.hprInterval(5,
                                                        Point3(0, 0, 0),
                                                        startHpr=Point3(180, 0, 0))
 
        # Create and play the sequence that coordinates the intervals.
        self.fin3Speed = Sequence(fin3PosInterval1,
                                  fin3HprInterval1,
                                  fin3PosInterval2,
                                  fin3HprInterval2,
                                  name="fin3Speed")
        self.fin3Speed.loop()

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
