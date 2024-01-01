import math

from scipy.spatial.transform import Rotation as R
import numpy as np
from controller import Robot

from typing import *
from numpy.typing import NDArray
from kuka_program import KukaProgram
from numpy import clip

# motor are [Right_Up,Left_Up,Right_Down,Left_Down]

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)

        # end is only there to prevent out of index exception
        self.decisionTree = ['r', 'l', 'f', 'l', 'l', 'l', 'r', 'r', 'ul', 'l', 'l', 'f', 'end']

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

        self.compass = self.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.BackRightest = self.getDevice("BackRightest")
        self.BackRightest.enable(self.timestep)

        self.BackRight = self.getDevice("BackRight")
        self.BackRight.enable(self.timestep)

        self.Rightest = self.getDevice("Rightest")
        self.Rightest.enable(self.timestep)

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

        self.greenSensor = self.getDevice('green sensor')
        self.greenSensor.enable(self.timestep)
        self.redSensor = self.getDevice('red sensor')
        self.redSensor.enable(self.timestep)
        self.blueSensor = self.getDevice('blue sensor')
        self.blueSensor.enable(self.timestep)

        self.last_side_error=0
        self.all_side_errors=0
        self.all_side_times=0

        self.last_error = 0
        self.all_errors = 0
        self.all_times = 0

        self.movment_velocity = 14.81

        self.numOfVs = 0

        self.currentState = "Line"
        self.subCurrentState = "Line"

        self.prevState = None
        self.boxAvoided = True
        self.leftDirection = False

        
        self.CONTROL_TIMESTEP = int(self.timestep * 4)

        self.kuka_program = KukaProgram(self, self.CONTROL_TIMESTEP)
        self.kuka_program.enable()
        self.kuka_program.reset()


        self.step(self.timestep)

    def read_light_sensors(self):
        green = self.greenSensor.getValue()
        blue = self.blueSensor.getValue()
        red = self.redSensor.getValue()

        sensors = [red, green, blue]

        if max(*sensors) == red:
            return sensors, 'r'
        elif max(*sensors) == green:
            return sensors, 'g'
        else:
            return sensors, 'b'

    def read_sensors_value(self):
        value = 0
        once = False
        for index, sensor in enumerate(self.sensors):
            if sensor.getValue() > 900 :
                value += self.sensors_coefficient[index]
                once = True
        return value/5,once

    def read_front_dist_sensors_value(self):
        value = 0
        for index, sensor in enumerate(self.front_dist_sen):
            if sensor.getValue() < 900 :
                value += self.sensors_coefficient[index]
        return value

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

    def turnMove(self,Left,perc=1):
        if Left:
            self.velocities[1] -= perc
            self.velocities[3] -= perc
            self.velocities[0] += perc
            self.velocities[2] += perc
        else:
            self.velocities[1] += perc
            self.velocities[3] += perc
            self.velocities[0] -= perc
            self.velocities[2] -= perc
        
        self.numOfVs += 1


    def sideMove(self,Left,perc=1):
        if Left:
            self.velocities[1] -= perc
            self.velocities[3] += perc
            self.velocities[0] += perc
            self.velocities[2] -= perc

        else:
            self.velocities[1] += perc
            self.velocities[3] -= perc
            self.velocities[0] -= perc
            self.velocities[2] += perc
        
        self.numOfVs += 1

    def setVelocities(self):
        if self.numOfVs == 0: self.numOfVs=1
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
            elif self.decision == 'f':
                return 'f'
            elif self.decision == 'ul':
                self.goal_rotation = 180
                self.sign = 1
            elif self.decision == 'end':
                self.currentState="Solved"

            return 't'

        if phase == 't':
            if abs(self.get_compass_bearing() - self.reference_rotation) < self.goal_rotation:
                if self.decision != 'b':
                    self.forward(0.8)
                self.stearing(0.5 * self.sign)
                return 't'
            return 'f'

        if phase == 'f':
            self.forward()
            if self.checkMaze():
                self.currentState = "Maze"
                return 's'
            else:
                return 'f'

    def get_compass_bearing(self):
        north = self.compass.getValues()
        rad = math.atan2(north[1], north[0])
        bearing = (rad - 1.5708) / math.pi * 180
        return bearing

    def checkBox(self):
        value = self.read_front_dist_sensors_value()
        if abs(value/5) > 0.1 :
            self.notBoxed=False
            self.prevState = self.currentState
            self.currentState = "Box"
            self.subCurrentState = "Boxing"
            self.boxAvoided = False

    def checkAvoidBox(self,fromLeft=False):
        if ((fromLeft and self.Rightest.getValue() < 900) or 
            (not(fromLeft) and self.Leftest.getValue() < 900)):
                self.subCurrentState="ForwardOnly"
    
    def checkPassBox(self,fromLeft=False):
        if ((fromLeft and self.BackRight.getValue() < 900) or 
            (not(fromLeft) and self.BackLeft.getValue() < 900)):
                self.subCurrentState="unBoxing"
    
    def DontHit(self,perc,fromLeft=False):
        if ((fromLeft and self.BackRightest.getValue() < 900) or 
            (not(fromLeft) and self.BackLeftest.getValue() < 900)):
                self.forward(perc)

    def checkLine(self):
        _,found = self.read_sensors_value()
        return found

    def checkMaze(self):
        _,found = self.read_side_sensors_value()
        return found

    def resetState(self):
        self.velocities=[0 for i in range(len(self.velocities))]
        self.numOfVs = 0

    def getAway(self,fromLeft=True):
        if self.subCurrentState == "Boxing":
            self.sideMove(fromLeft,0.5)
            self.checkAvoidBox(fromLeft)
        elif self.subCurrentState == "ForwardOnly":
            self.forward()
            self.checkPassBox(fromLeft)
        elif self.subCurrentState == "unBoxing":
            self.DontHit(0.5,fromLeft)
            self.sideMove(not(fromLeft),0.5)
            self.turnMove(not(fromLeft),0.5)
            if (self.prevState=="Line"and self.checkLine()) or (self.prevState=="Maze" and self.checkMaze()):
                self.currentState = self.prevState

    def search_box(self):
        side = self.kuka_program.detect_box('all')[0]
        if side is not None:
            self.currentState="Catch"
        return side

    def box(self):
        box_side = self.search_box()
        if box_side is None: return None

        box_arm = 'back' if box_side == 'back' else 'front'

        print(f'Found box at side "{box_side}", using arm "{box_arm}" to capture it.')

        print('- Aligning box.')
        box_angle = self.kuka_program.align_box(box_side, box_arm)

        print('- Capturing box.')
        self.kuka_program.capture_box(box_angle, box_side, box_arm)

        # Or both steps at once:
        # kuka_program.align_and_capture_box(box_side, box_arm)

        if box_arm == 'front':
            print('- Exchanging box.')
            self.kuka_program.exchange_box(to='back')
            self.kuka_program.sleep(.5)

        print('- Resetting Robot.')
        self.kuka_program.reset()
        # self.kuka_program.disable()
        self.currentState="Line"

    def loop(self):
        phase = 's'
        while self.step(self.timestep) != -1:
            self.resetState()

            if self.currentState == "Line":
                self.line_follow()
                self.forward()
                self.checkBox()
            elif self.currentState == "Box":
                if self.read_front_dist_sensors_value()>0 and not(self.boxAvoided):
                    self.leftDirection = True
                    self.boxAvoided=True
                elif not(self.boxAvoided):
                    self.leftDirection = False
                    self.boxAvoided=True
                self.getAway(self.leftDirection)
            elif self.currentState == "Catch":
                self.box()
            elif self.currentState == "Maze":
                self.checkBox()
                self.wall_follow()
                self.forward()
            elif self.currentState == "DecisionTree":
                phase = self.decision_tree(phase)

            self.setVelocities()
            


robot = RobotController()
robot.loop()