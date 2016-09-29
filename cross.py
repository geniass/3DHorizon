from math import pi, sin, cos
 
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence
from panda3d.core import Point3
 
class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
 
        # Disable the camera trackball controls.
        self.disableMouse()
 
        # Load the environment model.
        # self.scene = self.loader.loadModel("models/environment")
        self.scene = self.loader.loadModel("3dcrossmatrix.egg")
        # Reparent the model to render.
        self.scene.reparentTo(self.render)
        # Apply scale and position transforms on the model.
        self.scene.setScale(0.18, 0.18, 0.18)
        self.scene.setPos(0.2, 42, 3)
        # self.scene.setScale(0.25, 0.25, 0.25)
        # self.scene.setPos(-8, 42, 0)

        # Add the spinCameraTask procedure to the task manager.
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")
 
        # Load and transform the panda actor.
        self.pandaActor = Actor("BlenderModels/Ship.x")
        self.pandaActor.setScale(0.5, 0.5, 0.5)
        self.pandaActor.reparentTo(self.render)
        # # Loop its animation.
        # self.pandaActor.loop("walk")
 
        # # Create the four lerp intervals needed for the panda to
        # # walk back and forth.
        # pandaPosInterval1 = self.pandaActor.posInterval(13,
        #                                                 Point3(0, -10, 0),
        #                                                 startPos=Point3(0, 10, 0))
        # pandaPosInterval2 = self.pandaActor.posInterval(13,
        #                                                 Point3(0, 10, 0),
        #                                                 startPos=Point3(0, -10, 0))
        # pandaHprInterval1 = self.pandaActor.hprInterval(3,
        #                                                 Point3(180, 0, 0),
        #                                                 startHpr=Point3(0, 0, 0))
        # pandaHprInterval2 = self.pandaActor.hprInterval(3,
        #                                                 Point3(0, 0, 0),
        #                                                 startHpr=Point3(180, 0, 0))
 
        # # Create and play the sequence that coordinates the intervals.
        # self.pandaPace = Sequence(pandaPosInterval1,
        #                           pandaHprInterval1,
        #                           pandaPosInterval2,
        #                           pandaHprInterval2,
        #                           name="pandaPace")
        # self.pandaPace.loop()
 
    # Define a procedure to move the camera.
    def spinCameraTask(self, task):
        angleDegrees = task.time * 0.000000000000000001
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(25.0 * sin(angleRadians), - 25.0 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont
 
app = MyApp()
app.run()