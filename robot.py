import wpilib
import wpilib.buttons
class MyRobot(wpilib.IterativeRobot):


    ##############SET UP FOR XBOX CONTROLLER###################


    def robotInit(self):

        self.drive1=wpilib.Talon(0)
        self.drive2=wpilib.Talon(1)
        self.shooter=wpilib.Talon(2)
        self.cam=wpilib.Talon(3)

        self.arm1=wpilib.DoubleSolenoid(0,1,2)
        self.arm2=wpilib.Solenoid(0,3,4)

        self.robot_drive = wpilib.RobotDrive(self.drive1,self.drive2)
        self.controller = wpilib.Joystick(0)
        #A button
        self.joystick_button=wpilib.buttons.JoystickButton(self.controller, 1)
        #B Button
        self.second_button=wpilib.buttons.JoystickButton(self.controller, 2)
        #Right and left bumper
        self.right_bumper = wpilib.buttons.JoystickButton(self.controller,5)
        self.left_bumper = wpilib.buttons.JoystickButton(self.controller,6)

        #Utrasonic Sensor
        self.sensor = wpilib.AnalogInput(3)

        #Init variable for ultrasonic sensor
        wpilib.SmartDashboard.putNumber('Distance', self.sensor.getValue())


    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        #starting out the state at neutral motors
        self.state=4

        self.counter = 0

        #timer for the fire function
        self.timer = wpilib.Timer()
        self.timer.start()


    def teleopPeriodic(self):
        #SMRT Dashboard updating
        self.updater()

        #Getting the right stick of the xbox controller#
        self.right_stickX=self.controller.getRawAxis(4)
        #Getting the triggers#
        self.right_trig=self.controller.getRawAxis(2)
        self.left_trig=-1*self.controller.getRawAxis(3)

        #Push solenoid forward anyways
        if self.right_bumper.get():
            self.arm1.set(1)
            self.arm2.set(1)
        #Retract solenoid anyways
        elif self.left_bumper.get():
            self.arm1.set(2)
            self.arm2.set(2)

        #Boost Button(Every likes it)
        if self.joystick_button.get(): #If "a" is pressed, fire the piston on port 4#
            multi=1
        else:
            multi=.5


        if self.second_button.get(): #FIRE THE PISTON AND MOTOR#
            self.state = 0

        self.fire()


        #Lets drive!
        self.robot_drive.arcadeDrive(multi*self.controller.getY(),multi*self.controller.getX(),True)


    def fire(self):
        """
        This function is the automated shooter.

        This was programmed well before the final shooter was in place so errors are going to happen with this

        """
        if self.state == 0:

            self.timer.reset()
            self.state=1

        elif self.state == 1:
            self.shooter.set(1)
            print("Im here")
            if self.timer.hasPeriodPassed(.5):
                self.state = 2

        elif self.state == 2:
            self.shooter.set(1)
            self.arm1.set(1)
            self.arm2.set(1)
            if self.timer.hasPeriodPassed(1):
                self.arm1.set(2)
                self.arm2.set(2)
                self.state=3
        elif self.state == 3:
            self.shooter.set(1)
            if self.timer.hasPeriodPassed(2):
                self.state=4
        elif self.state==4:
            self.shooter.set(0)


    def updater(self):
        wpilib.SmartDashboard.putNumber('Distance', self.sensor.getValue())


    def disabledPeriodic(self):
        self.shooter.set(0)
if __name__ == "__main__":
    wpilib.run(MyRobot)
