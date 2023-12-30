import math

from scipy.spatial.transform import Rotation as R
import numpy as np
from controller import Robot

from numpy import clip

# motor are [Right_Up,Left_Up,Right_Down,Left_Down]

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)

        # end is only there to prevent out of index exception
        self.decisionTree = ['r', 'l', 'end']

        self.timestep = int(self.getBasicTimeStep())
        self.motors=[]
        self.velocities=[]
        for i in range(4):
            self.motors.append(self.getDevice("wheel{0}".format(i+1)))
            self.motors[i].setPosition(float("+inf"))
            self.motors[i].setVelocity(0)
            self.velocities.append(0)

        self.sensors_index = [0,1,2,3]
        self.sensors_coefficient = [3, 2, -2, -3]

        self.front_dist_sen = []
        for index in self.sensors_index:
            self.front_dist_sen.append(self.getDevice("DisSenFront_" + str(index)))
            self.front_dist_sen[index].enable(self.timestep)

        self.sensors = []
        for index in self.sensors_index:
            self.sensors.append(self.getDevice("IR_" + str(index)))
            self.sensors[index].enable(self.timestep)

        self.inUn = self.getDevice("inUn")
        self.inUn.enable(self.timestep)

        self.compass = self.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.BackRightest = self.getDevice("BackRightest")
        self.BackRightest.enable(self.timestep)

        self.BackRight = self.getDevice("BackRight")
        self.BackRight.enable(self.timestep)

        self.Rightest = self.getDevice("Rightest")
        self.Rightest.enable(self.timestep)

        self.middle = self.getDevice("Middle")
        self.middle.enable(self.timestep)

        self.Leftest = self.getDevice("Leftest")
        self.Leftest.enable(self.timestep)

        self.BackLeft = self.getDevice("BackLeft")
        self.BackLeft.enable(self.timestep)

        self.BackLeftest = self.getDevice("BackLeftest")
        self.BackLeftest.enable(self.timestep)

        self.PerfectLeft=self.getDevice('PerfectLeft')
        self.PerfectLeft.enable(self.timestep)

        self.PerfectRight=self.getDevice('PerfectRight')
        self.PerfectRight.enable(self.timestep)

        self.last_side_error=0
        self.all_side_errors=0
        self.all_side_times=0

        self.notBoxed=True

        self.last_error = 0
        self.all_errors = 0
        self.all_times = 0

        self.movment_velocity = 14.81

        self.numOfVs = 0

        self.currentState = "Line"

        self.refRot = None
        self.prevState = None

        self.step(self.timestep)

    def getRot(self):
        rot = self.inUn.getQuaternion()
        r = R.from_quat(rot)
        return r.as_rotvec()

    def read_sensors_value(self):
        value = 0
        once = False
        for index, sensor in enumerate(self.sensors):
            if sensor.getValue() > 900 :
                value += self.sensors_coefficient[index]
                once = True
        return value/5,once

    def read_side_sensors_value(self):
        coeff=0.01
        return (self.PerfectRight.getValue() - self.PerfectLeft.getValue()) * coeff ,\
         self.PerfectRight.getValue() != 1000 and self.PerfectLeft.getValue() != 1000


    def stearing(self,perc):
        self.velocities[1] -= perc
        self.velocities[3] -= perc
        self.velocities[0] += perc
        self.velocities[2] += perc

        self.numOfVs += 1
    
    def forward(self,perc=1):
        self.velocities[1] += perc
        self.velocities[3] += perc
        self.velocities[0] += perc
        self.velocities[2] += perc
        
        self.numOfVs += 1

    def turnRight(self,perc=1,deg=None):
        if deg != None and self.currentState != "Turning":
            self.prevState = self.currentState
            self.currentState="Turning"
            self.refRot = self.getRot()
        
        if deg == None or self.currentState == "Turning":
            self.velocities[1] += perc
            self.velocities[3] += perc
            self.velocities[0] -= perc
            self.velocities[2] -= perc
            
            self.numOfVs += 1

        if (deg != None and abs(self.getRot()-self.refRot)[2] + 1e-8 > (math.pi*deg/180)):
            self.currentState=self.prevState
            print("Done")

    
    def turnLeft(self,perc=1):
        self.velocities[1] -= perc
        self.velocities[3] -= perc
        self.velocities[0] += perc
        self.velocities[2] += perc
        
        self.numOfVs += 1


    def sideRight(self,perc=1):
        self.velocities[1] += perc
        self.velocities[3] -= perc
        self.velocities[0] -= perc
        self.velocities[2] += perc
        
        self.numOfVs += 1


    def sideLeft(self,perc=1):
        self.velocities[1] -= perc
        self.velocities[3] += perc
        self.velocities[0] += perc
        self.velocities[2] -= perc
        
        self.numOfVs += 1

    def setVelocities(self):
        if self.numOfVs == 0: return
        for i in range(len(self.motors)):
            self.motors[i].setVelocity(
                clip(self.movment_velocity * (self.velocities[i] / self.numOfVs), -self.movment_velocity,
                     self.movment_velocity))

    def followPID(self,fun,nextState):
        goal = 0
        reading,found = fun()

        if not(found):
            self.currentState=nextState
            return

        error = goal - reading
        Kp = 3
        P = Kp * error


        error_rate = error - self.last_error
        self.last_error = error
        Kd = 1
        D = Kd * error_rate

        self.all_errors *= self.all_times
        self.all_errors += error
        self.all_times += 1
        self.all_errors /= self.all_times
        Ki = 1
        I = Ki * self.all_errors


        stearingVal = (P + D + I)/(Kp+Kd+Ki)

        self.stearing(stearingVal)

    def line_follow(self):
        self.followPID(self.read_sensors_value,"Maze")

    def wall_follow(self):
        self.followPID(self.read_side_sensors_value,"DecisionTree")

    # phase can be s,t,f
    def decision_tree(self, phase):
        if phase == 's':
            self.decision = self.decisionTree[0]
            self.decisionTree = self.decisionTree[1:]

            self.reference_rotation = self.get_compass_bearing()
            self.current_rotation = self.reference_rotation

            if self.decision == 'l':
                self.goal_rotation = 90
                self.sign = 1
            elif self.decision == 'r':
                self.goal_rotation = 90
                self.sign = -1
            elif self.decision == 'b':
                self.goal_rotation = 180
                self.sign = 1

            return 't'

        if phase == 't':
            if abs(self.current_rotation - self.reference_rotation) < self.goal_rotation:
                self.current_rotation += abs(self.get_compass_bearing() - self.current_rotation)
                print(self.current_rotation)
                self.stearing(0.5 * self.sign)
                return 't'
            return 'f'

        if phase == 'f':
            # Both sensors are reaching their full distance without an obstacle
            if self.read_side_sensors_value() == 0:
                self.forward()
                return 'f'
            else:
                self.currentState = "Maze"
                return 's'

    def get_compass_bearing(self):
        north = self.compass.getValues()
        rad = math.atan2(north[1], north[0])
        bearing = (rad - 1.5708) / math.pi * 180
        if bearing < 0:
            bearing += 360
        return bearing
    def checkBox(self):
        value = 0
        for index, sensor in enumerate(self.front_dist_sen):
            if sensor.getValue() < 900 :
                value += self.sensors_coefficient[index]
        if abs(value/5) > 0.1 :
            self.notBoxed=False
            self.currentState = "Boxing"

    def checkAvoidBox(self,side="Right"):
        if ((side=="Right" and self.Rightest.getValue() < 900) or 
            (side=="Left" and self.Leftest.getValue() < 900)):
                self.currentState="ForwardOnly"
    
    def checkPassBox(self,side="Right"):
        if ((side=="Right" and self.BackRight.getValue() < 900) or 
            (side=="Left" and self.BackLeft.getValue() < 900)):
                self.currentState="unBoxing"
    
    def DontHit(self,perc,side="Right"):
        if ((side=="Right" and self.BackRightest.getValue() < 900) or 
            (side=="Left" and self.BackLeftest.getValue() < 900)):
                self.forward(perc)

    def checkLine(self):
        _,found = self.read_sensors_value()
        if found:
            self.currentState = "Line"

    def resetState(self):
        self.velocities=[0 for i in range(len(self.velocities))]
        self.numOfVs = 0

    def loop(self):
        phase = 's'
        while self.step(self.timestep) != -1:
            self.resetState()

            # self.turnRight(1,90)

            if self.currentState == "Line":
                self.line_follow()
                self.forward()
                if self.notBoxed:
                    self.checkBox()
            elif self.currentState == "Boxing":
                self.sideLeft(0.5)
                self.checkAvoidBox()
            elif self.currentState == "ForwardOnly":
                self.forward()
                self.checkPassBox()
            elif self.currentState == "unBoxing":
                self.DontHit(0.5)
                self.sideRight(0.5)
                self.turnRight(0.5)
                self.checkLine()
            elif self.currentState == "Maze":
                self.wall_follow()
                self.forward()
            elif self.currentState == "DecisionTree":
                phase = self.decision_tree(phase)

            self.setVelocities()
            


robot = RobotController()
robot.loop()