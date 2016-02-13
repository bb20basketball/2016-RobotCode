#!/usr/bin/env python3
import wpilib
import wpilib.buttons
from robotpy_ext.common_drivers import units, navx
class MyRobot(wpilib.IterativeRobot):


    ##############SET UP FOR XBOX CONTROLLER###################
    ##############Last Update: 1/26/16#########################


    def robotInit(self):

        self.drive1=wpilib.Talon(1)
        self.drive2=wpilib.Talon(2)
        self.shooter=wpilib.Talon(3)
        self.cam=wpilib.Talon(4)
        #navx
        self.navx = navx.AHRS.create_spi()
        #Robot Driving Arcade
        self.arcade_drive=wpilib.RobotDrive(self.drive1,self.drive2)

        #Solenoid me
        self.arm1=wpilib.DoubleSolenoid(0,1,2)
        self.arm2=wpilib.DoubleSolenoid(0,3,4)
        self.servo=wpilib.Servo(0)
        #Testing some ultrasonic sensors
        self.ultrasonic=wpilib.Ultrasonic(1,0, units.inch)
        self.ultrasonic.setAutomaticMode(True)

        #TWO CONTROLLERS
        self.controller = wpilib.Joystick(0)
        self.second_controller=wpilib.Joystick(1)

        #A button Second Controller
        self.joystick_button=wpilib.buttons.JoystickButton(self.second_controller, 1)
        #A button on Main
        self.turn_button=wpilib.buttons.JoystickButton(self.controller, 1)
        #X button for cancelling a rogue navx
        self.cancel=wpilib.buttons.JoystickButton(self.controller, 3)
        #Y Button on Second Controller
        self.second_button=wpilib.buttons.JoystickButton(self.second_controller, 4)

        #Right bumper
        self.right_bumper = wpilib.buttons.JoystickButton(self.second_controller,6)
        self.left_bumper = wpilib.buttons.JoystickButton(self.second_controller,5)
        #Right bumper for boost on main controller
        self.main_fast=wpilib.buttons.JoystickButton(self.controller, 6)

        #For controlling the servos, A and B button
        self.left_servo=wpilib.buttons.JoystickButton(self.second_controller,3)
        self.right_servo=wpilib.buttons.JoystickButton(self.second_controller,2)

        #Saving for later
        #Utrasonic Sensor
        #self.sensor = wpilib.AnalogInput(3)
        #self.ultrasonic = xl_max_sonar_ez.MaxSonarEZAnalog(3, units.inch)

        #Make all the variables needed
        self.shooter_piston=1
        self.speedShooter=0
        self.speedCam=0
        #Init variable for ultrasonic sensor
        #Timer stuff
        self.timer = wpilib.Timer()
        self.timer.start()
        
        #Shooter speeds
        self.shooter_high=.45
        self.updater()
        
    def autonomousInit(self):
        
        self.auto_motor=0
        self.auto_state=0
        self.auto_drive1=0
        self.auto_drive2=0
        self.state=4
        self.arcade_drive.setSafetyEnabled(False)


    def autonomousPeriodic(self):
        #reset the timer for autonomous
        if self.auto_state == 0:
            self.timer.reset()
            self.auto_state=1
            self.auto_drive1=0
            self.auto_drive2=0
        #drive forward for x amount of time
        elif self.auto_state==1:
            self.auto_drive1=.25
            self.auto_drive2=.25
            if self.timer.hasPeriodPassed(1):
                self.auto_state=2
        #turn 20 degrees to face target
        elif self.auto_state==2:
            self.auto_drive1=0
            self.auto_drive2=0
            if self.turn(20):
                self.auto_state=3
        #Drive forward again
        elif self.auto_state==3:
            self.auto_drive1=.25
            self.auto_drive2=.25
            if self.timer.hasPeriodPassed(3):
                self.auto_state=4
        #do a complete 180 to get ready to shoot
        elif self.auto_state==4:
            self.auto_drive1=0
            self.auto_drive2=0
            if self.turn(-207):
                self.auto_state=5
        #FIRE THE SEQUENCE
        elif self.auto_state==5:
            self.auto_drive1=0
            self.auto_drive2=0
            self.state=0
            self.auto_state=6
            
        self.fire()
        self.updater()

        #Set all the motors and pistons
        self.shooter.set(self.speedShooter)
        self.arm1.set(self.shooter_piston)
        self.arm2.set(self.shooter_piston)
        self.drive1.set((-1*self.auto_drive1))
        self.drive2.set((1*self.auto_drive2))

                
    def turn(self, degrees):
        """
        For autonomous, to turn to set angle, requires reset on the navx to zero it out
        """
        current=self.navx.getYaw()
        if current < (degrees+5) and current > degrees:
            self.auto_drive2=0
            return True
        else:
            self.auto_drive2=.7


    def teleopInit(self):
        #starting out the state at neutral motors
        self.state=4
        self.total_pan=0
        self.turn_state=2
        self.desired=0
        self.arcade_drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        #SMRT Dashboard updating
        self.updater()

        #Starts the fire stuff
        if self.second_button.get(): #FIRE THE PISTON AND MOTORS#
            self.state = 0

        self.fire()
        self.cameraControl()

        #Retract solenoid anyways
        if self.right_bumper.get():
            self.shooter_piston=2
        elif self.left_bumper.get():
            self.shooter_piston=1

        #Gets rid of some of the .getRawAxis stuff
        self.getControllerStates()
        if self.turn_button.get():
            self.turn_state=0
            yaw=self.navx.getYaw()
            if yaw > 0: 
                self.desired=self.navx.getYaw()-227
                if self.desired<-179:
                    self.desired=self.desired+360
            else:
                self.desired=self.navx.getYaw()+133

        #Set Everything that needs to be set
        self.arm1.set(self.shooter_piston)
        self.arm2.set(self.shooter_piston)
        self.shooter.set(self.speedShooter)
        self.cam.set(self.speedCam)
        #Lets drive!
        self.arcade_drive.arcadeDrive((self.boost*self.controller.getY()), (self.boost*(-1*self.controller.getX())), True)
        
        self.teleopTurn()
        
    def getControllerStates(self):
        
        #Gets the values of triggers for the Cam
        self.left=-1*(self.controller.getRawAxis(2))
        self.right=self.controller.getRawAxis(3)
        self.boost=(((self.left+self.right)*.5)-.1)+.6
        #programming for lack of accuracy on the triggers
        if self.boost>1:
            self.boost=1
        elif self.boost<0:
            self.boost=0

        #Triggers for the second controller for manual speed control over the shooter
        self.second_left=-1*(self.second_controller.getRawAxis(2))
        self.second_right=(self.second_controller.getRawAxis(3))

        #IF you are using the controller, then it will do it
        if self.second_right>.1 or self.second_left<-.1:
            self.speedShooter=self.second_left+self.second_right

        self.right_stick = -1*(self.controller.getRawAxis(5))
        if self.right_stick > -.2 and self.right_stick < .2:
            self.speedCam=0
        else:
            self.speedCam=self.right_stick
            
    def cameraControl(self):
        
        if self.right_servo.get():
            self.total_pan=self.total_pan-.025
        elif self.left_servo.get():
            self.total_pan=self.total_pan+.025

        if self.total_pan>1:
            self.total_pan=1
        elif self.total_pan<0:
            self.total_pan=0

        self.servo.set(self.total_pan)
        
    def fire(self):
        """
        This function is the automated shooter. Fires piston out, spins motor to speed, fires back
        """
        if self.state == 0:
            self.timer.reset()
            self.state=1
            self.speedShooter=0

        elif self.state == 1:
            self.controller.setRumble(1, .9)
            self.second_controller.setRumble(1, .9)
            self.shooter_piston=2
            self.speedShooter=0

            if self.timer.hasPeriodPassed(.75):
                self.speedShooter=self.shooter_high
                self.state=2

        elif self.state==2:
            self.shooter_piston=2
            self.speedShooter=self.shooter_high
            if self.timer.hasPeriodPassed(3):
                self.speedShooter=self.shooter_high
                self.shooter_piston=1
                self.state=3

        elif self.state == 3:
            self.speedShooter=self.shooter_high
            if self.timer.hasPeriodPassed(.75):
                self.state=4

        elif self.state==4 and self.joystick_button.get():
            self.intake()

        elif self.state==4:
            self.speedShooter=0
            #self.shooter_piston=1 I could change this and see if that helps stop the immediate retract
            self.controller.setRumble(1, 0)
            self.second_controller.setRumble(1, 0)

    def intake(self):
        #This might be a problem if the pistons fire before the motors are ready
        self.speedShooter=.2
        self.shooter_piston=2

    def amIStuck(self):
        if self.navx.getVelocityY()<1:
            print("I am stuck")

    def teleopTurn(self):
        """
        Takes the current position and tries to do a 180 for the shooter
        """
        current=self.navx.getYaw()
        if self.turn_state==0:
            #self.navx.zeroYaw()
            desired = self.desired
            self.turn_state=1
        elif self.turn_state==1:
            if current < (self.desired+10) and current > self.desired or current==0:#Trying this to see if NavX freaks out, it will stop
                self.turn_state=2
            else:
                if self.cancel.get()
                    self.turn_state=2
                self.drive2.set(.7)
                self.drive1.set(.5)

    def updater(self):
        ##Put all smartdashboard things here
        wpilib.SmartDashboard.putNumber('Distance', self.ultrasonic.getRangeInches())
        wpilib.SmartDashboard.putNumber('Yaw', self.navx.getYaw())
        wpilib.SmartDashboard.putNumber('Velocity', self.navx.getVelocityY())
        wpilib.SmartDashboard.putNumber('Vision', 1)#Will use this one eventually

    def disabledPeriodic(self):
        ##Updated values when disabled
        self.updater()


if __name__ == "__main__":
    wpilib.run(MyRobot)
