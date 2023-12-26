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

        self.front_dist_sen_index = [0,1,2]
        self.front_dist_sen = []
        for index in self.front_dist_sen_index:
            self.front_dist_sen.append(self.getDevice("DisSenFront_" + str(index)))
            self.front_dist_sen[index].enable(self.timestep)

        self.sensors_index = [0,1,2,3]
        self.sensors = []
        self.sensors_coefficient = [3, 2, -2, -3]
        for index in self.sensors_index:
            self.sensors.append(self.getDevice("IR_" + str(index)))
            self.sensors[index].enable(self.timestep)

        self.last_error = 0
        self.all_errors = 0
        self.all_times = 0

        self.movment_velocity = 14.81

        self.step(self.timestep)

    def read_dist_sensors_value(self):
        value = 0
        for index, sensor in enumerate(self.front_dist_sen):
            if sensor.getValue() > 200 : value += self.sensors_coefficient[index]
        return value

    def read_sensors_value(self):
        value = 0
        for index, sensor in enumerate(self.sensors):
            if sensor.getValue() > 900 : value += self.sensors_coefficient[index]
        return value/5

    def stearing(self,perc):
        self.velocities[1] += -perc
        self.velocities[3] += -perc
        self.velocities[0] += perc
        self.velocities[2] += perc
    
    def forward(self):
        self.velocities[1] += 1
        self.velocities[3] += 1
        self.velocities[0] += 1
        self.velocities[2] += 1

    def setVelocities(self):
        for i in range(len(self.motors)): 
            self.motors[i].setVelocity(self.movment_velocity*(self.velocities[i]/2))

    def line_follow(self):
        goal = 0
        reading = self.read_sensors_value()

        error = goal - reading
        Kp = 1
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


        stearingVal = (P + D + I)/3

        self.stearing(stearingVal)

    def turnRight(self):
        self.motors[1].setVelocity(self.movment_velocity)
        self.motors[3].setVelocity(self.movment_velocity)
        self.motors[0].setVelocity(-self.movment_velocity)
        self.motors[2].setVelocity(-self.movment_velocity)
    
    def turnLeft(self):
        self.motors[1].setVelocity(-self.movment_velocity)
        self.motors[3].setVelocity(-self.movment_velocity)
        self.motors[0].setVelocity(self.movment_velocity)
        self.motors[2].setVelocity(self.movment_velocity)

    def loop(self):
        while self.step(self.timestep) != -1:
            self.velocities=[0 for i in range(len(self.velocities))]
            self.line_follow()
            self.forward()

            self.setVelocities()
            


robot = RobotController()
robot.loop()