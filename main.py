from math import pi, sin, cos
from TI_IMU import TI_IMU
import serial
import csv
import time


from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor

f = open('60hz.csv', 'r')
reader = csv.reader(f, delimiter = "," )
allData = []
for i in reader:
    t = (float(i[3]),float(i[4]),float(i[5]))   #i[3] is roll, i[4] is pitch and [5] is yaw
    allData.append(t)
f.close()


class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.setFrameRateMeter(True)

        # Load the environment model.
        self.scene = self.loader.loadModel("3dcrossmatrix.egg")
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

        #Initialize index for incrementing rows of data
        self.inde = 0

        #Initialize timer to match refresh rate of screen with refresh rate of sensor
        self.start_time = time.time()

        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.imu = TI_IMU(self.ser)
        self.imu.start()

        while not self.imu.data_ready():
            self.state = self.imu.get_state()

    # Define a procedure to move the camera.
    def spinCameraTask(self, task):
        if self.imu.data_ready():
            self.state = self.imu.get_state()
        angleDegrees = task.time * 0.1
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20.0 * cos(angleRadians), 3)
        row = allData[self.inde]
        self.camera.setHpr(angleDegrees, self.state[4], self.state[3])
        if self.inde + 1 < len(allData):
                self.inde += 1
        else:
                self.inde = 0
        return Task.cont



app = MyApp()
app.run()
