#!/usr/bin/env python3
import wpilib
import wpilib.buttons
from robotpy_ext.common_drivers import units
class MyRobot(wpilib.IterativeRobot):


    ##############SET UP FOR XBOX CONTROLLER###################
    ##############Last Update: 1/26/16#########################


    def robotInit(self):

        self.drive1=wpilib.Talon(0)
        self.drive2=wpilib.Talon(1)
        self.shooter=wpilib.Talon(2)
        self.cam=wpilib.Talon(3)

        #Solenoid me
        self.arm1=wpilib.DoubleSolenoid(0,1,2)
        self.arm2=wpilib.DoubleSolenoid(0,3,4)

        #Testing some ultrasonic sensors
        self.ultrasonic=wpilib.Ultrasonic(1,0, units.inch)
        self.ultrasonic.setAutomaticMode(True)

        #TWO CONTROLLERS
        self.controller = wpilib.Joystick(0)
        self.second_controller=wpilib.Joystick(1)

        #A button
        self.joystick_button=wpilib.buttons.JoystickButton(self.second_controller, 1)
        #B Button
        self.second_button=wpilib.buttons.JoystickButton(self.second_controller, 2)

        #Right and left bumper
        self.right_bumper = wpilib.buttons.JoystickButton(self.second_controller,5)
        self.left_bumper = wpilib.buttons.JoystickButton(self.second_controller,6)
        #Right bumper for boost on main controller
        self.main_fast=wpilib.buttons.JoystickButton(self.controller, 6)


        #Saving for later
        #Utrasonic Sensor
        #self.sensor = wpilib.AnalogInput(3)
        #self.ultrasonic = xl_max_sonar_ez.MaxSonarEZAnalog(3, units.inch)

        #Make all the variables needed
        self.shooter_piston=1
        self.speedShooter=0
        self.speedCam=0
        #Init variable for ultrasonic sensor
        self.updater()

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        #starting out the state at neutral motors
        self.state=4

        #timer for the fire function
        self.timer = wpilib.Timer()
        self.timer.start()


    def teleopPeriodic(self):
        #SMRT Dashboard updating
        self.updater()

        #Cleans up all the .getRawAxis stuff
        self.getControllerStates()



        #Booster
        if self.main_fast.get():
            multi=1
        else:
            multi=.65
        #Starts the fire stuff
        if self.second_button.get(): #FIRE THE PISTON AND MOTOR#
            self.state = 0
        #Intaking while A is pressed on second controller




        self.fire()

        #Push solenoid forward anyways
        if self.right_bumper.get():
            self.shooter_piston=1

        #Retract solenoid anyways
        elif self.left_bumper.get():
            self.shooter_piston=2



        #Set Everything that needs to be set
        self.arm1.set(self.shooter_piston)
        self.arm2.set(self.shooter_piston)
        self.shooter.set(self.speedShooter)
        self.cam.set(self.speedCam)
        #Lets drive!
        self.drive1.set(((-1*multi)*self.controller.getRawAxis(1)))
        self.drive2.set(((1*multi)*self.controller.getRawAxis(5)))

    def getControllerStates(self):
        #Gets the values of triggers for the Cam
        self.left=-1*(self.controller.getRawAxis(2))
        self.right=self.controller.getRawAxis(3)
        self.speedCam=self.left+self.right

    def fire(self):
        """
        This function is the automated shooter.

        This was programmed well before the final shooter was in place so errors are going to happen with this

        """
        if self.state == 0:

            self.timer.reset()
            self.state=2
            self.speedShooter=0


        elif self.state == 2:
            self.controller.setRumble(1, .5)
            self.second_controller.setRumble(1,.5)
            self.shooter_piston=2
            self.speedShooter=1
            
            if self.timer.hasPeriodPassed(3):
                self.speedShooter=1
                self.shooter_piston=1
                self.state=3
        elif self.state == 3:
            self.speedShooter=1
            if self.timer.hasPeriodPassed(1.5):
                self.state=4
        elif self.state==4 and self.joystick_button.get():
            self.intake()
        elif self.state==4:
            self.speedShooter=0
            self.shooter_piston=1
            self.controller.setRumble(1, 0)
            self.second_controller.setRumble(1, 0)

    def intake(self):
        #This might be a problem if the pistons fire before the motors are ready
        self.speedShooter=.25
        self.shooter_piston=2



    def updater(self):
        ##Put all smartdashboard things here
        wpilib.SmartDashboard.putNumber('Distance', self.ultrasonic.getRangeInches())

    def disabledPeriodic(self):
        ##Updated values when disabled
        self.updater()


if __name__ == "__main__":
    wpilib.run(MyRobot)
