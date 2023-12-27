from controller import Robot

# motor are [Right_Up,Left_Up,Right_Down,Left_Down]

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)
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

        self.notBoxed=True

        self.last_error = 0
        self.all_errors = 0
        self.all_times = 0

        self.movment_velocity = 14.81

        self.numOfVs = 0

        self.currentState = "Line"

        self.step(self.timestep)

    def read_sensors_value(self):
        value = 0
        once = False
        for index, sensor in enumerate(self.sensors):
            if sensor.getValue() > 900 :
                value += self.sensors_coefficient[index]
                once = True
        return value/5,once

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

    def turnRight(self,perc=1):
        self.velocities[1] += perc
        self.velocities[3] += perc
        self.velocities[0] -= perc
        self.velocities[2] -= perc
        
        self.numOfVs += 1

    
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
            self.motors[i].setVelocity(self.movment_velocity*(self.velocities[i]/self.numOfVs))

    def line_follow(self):
        goal = 0
        reading,found = self.read_sensors_value()

        if not(found):
            self.currentState="Maze"
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
    def resetState(self):
        self.velocities=[0 for i in range(len(self.velocities))]
        self.numOfVs = 0

    def loop(self):
        while self.step(self.timestep) != -1:
            self.resetState()

            if self.currentState=="Line":
                self.line_follow()
                self.forward()
                if self.notBoxed:
                    self.checkBox()
            elif self.currentState=="Boxing":
                self.sideLeft(0.5)
                self.checkAvoidBox()
            elif self.currentState=="ForwardOnly":
                self.forward()
                self.checkPassBox()
            elif self.currentState=="Maze":
                self.sideRight()

            self.setVelocities()
            


robot = RobotController()
robot.loop()