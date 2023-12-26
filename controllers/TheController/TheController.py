from controller import Robot

# motor are [Right_Up,Left_Up,Right_Down,Left_Down]

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)
        self.timestep = int(self.getBasicTimeStep())
        self.motors=[]
        for i in range(4):
            self.motors.append(self.getDevice("wheel{0}".format(i+1)))
            self.motors[i].setPosition(float("+inf"))
            self.motors[i].setVelocity(0)

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

        self.movment_velocity = 14.81

        self.step(self.timestep)

    def read_dist_sensors_value(self):
        value = 0
        for index, sensor in enumerate(self.front_dist_sen):
            # print(sensor.getValue())
            if sensor.getValue() > 200 : value += self.sensors_coefficient[index]
        return value

    def read_sensors_value(self):
        value = 0
        for index, sensor in enumerate(self.sensors):
            # print(sensor.getValue())
            if sensor.getValue() > 900 : value += self.sensors_coefficient[index]
        return value/5

    def stearing(self,perc):
        self.motors[1].setVelocity(self.movment_velocity*(1-perc))
        self.motors[3].setVelocity(self.movment_velocity*(1-perc))
        self.motors[0].setVelocity(self.movment_velocity*perc)
        self.motors[2].setVelocity(self.movment_velocity*perc)
    
    def forward(self):
        self.motors[1].setVelocity(self.movment_velocity)
        self.motors[3].setVelocity(self.movment_velocity)
        self.motors[0].setVelocity(self.movment_velocity)
        self.motors[2].setVelocity(self.movment_velocity)

    def line_follow(self):
        goal = 0
        reading = self.read_sensors_value()
        print("reading:",reading)

        error = goal - reading
        Kp = 1
        P = Kp * error
        print("P:",P)


        error_rate = error - self.last_error
        self.last_error = error
        Kd = 1
        # D = Kd * error_rate
        D = 0
        self.stearing(P + D)

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
            # self.read_sensors_value()
            self.line_follow()
            
            # print(self.read_sensors_value())


robot = RobotController()
robot.loop()