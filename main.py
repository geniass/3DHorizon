from math import pi, sin, cos
from TI_IMU import TI_IMU
import csv

 
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor

f = open('test.csv', 'rb')
reader = csv.reader(f, delimiter = "," )
allData = []
for i in reader:
    t = (i[3],i[4],i[5])
    allData.append(t)
f.close()
 
class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
 
        # Load the environment model.
        self.scene = self.loader.loadModel("models/environment")
        # Reparent the model to render.
        self.scene.reparentTo(self.render)
        # Apply scale and position transforms on the model.
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)
 
        # Add the spinCameraTask procedure to the task manager.
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")
 
        # Load and transform the panda actor.
        self.pandaActor = Actor("models/panda-model",
                                {"walk": "models/panda-walk4"})
        self.pandaActor.setScale(0.005, 0.005, 0.005)
        self.pandaActor.reparentTo(self.render)
        # Loop its animation.
        self.pandaActor.loop("walk")

        #
        # with serial.Serial(args.comport, args.baud) as ser:
        #     imu = TI_IMU(ser)
        #     imu.start()


    # Define a procedure to move the camera.
    def spinCameraTask(self, task):
        angleDegrees = task.time * 0.1
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20.0 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont


app = MyApp()
app.run()