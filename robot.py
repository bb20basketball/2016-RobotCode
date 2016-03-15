#!/usr/bin/env python3
import wpilib
import wpilib.buttons
from robotpy_ext.common_drivers import units, navx
import networktables
class MyRobot(wpilib.IterativeRobot):


    ##############SET UP FOR XBOX CONTROLLER###################
    ##############For two talented drivers: one for driving, one for shooting#########################


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
        self.bottoms_up=wpilib.Solenoid(0,5)
        #Camera servo in the front to be able to navigate around
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
        self.cancel=wpilib.buttons.JoystickButton(self.controller, 2)
        #Y Button on Second Controller
        self.second_button=wpilib.buttons.JoystickButton(self.second_controller, 4)
        #Left Bumper for firing the back piston
        self.fire_buttoms_up=wpilib.buttons.JoystickButton(self.controller, 5)

        #You can press X to line up your shot if need be
        self.auto_alineX=wpilib.buttons.JoystickButton(self.controller, 3)
        self.auto_alineY=wpilib.buttons.JoystickButton(self.controller, 4)

        #Right bumper
        self.right_bumper = wpilib.buttons.JoystickButton(self.second_controller,6)
        self.left_bumper = wpilib.buttons.JoystickButton(self.second_controller,5)
        #Right bumper for boost on main controller
        self.main_fast=wpilib.buttons.JoystickButton(self.controller, 6)

        #For controlling the servos, A and B button
        self.low_goal=wpilib.buttons.JoystickButton(self.second_controller,3)

        #So you can always know where is front
        self.ball_servo=wpilib.buttons.JoystickButton(self.second_controller, 9)
        self.forward_servo=wpilib.buttons.JoystickButton(self.second_controller,10)

        #In case one of your slow drivers can't line up in time, you can hold off the sequence
        self.hold_button=wpilib.buttons.JoystickButton(self.second_controller, 2)

        #Allows you to fine tine the shooter speed on the go if conditions change
        self.higher_speed=wpilib.buttons.JoystickButton(self.second_controller, 8)
        self.lower_speed=wpilib.buttons.JoystickButton(self.second_controller, 7)

        #Make all the variables needed
        self.shooter_piston=1
        self.speedShooter=0
        self.speedCam=0
        self.piston_update="I DON'T KNOW"
        #Init variable for ultrasonic sensor
        #Timer stuff
        self.timer = wpilib.Timer()
        self.timer.start()

        self.vision_table = networktables.NetworkTable.getTable('/GRIP/myContoursReport')
        self.vision_x= networktables.NumberArray()
        self.vision_y=networktables.NumberArray()
        self.vision_numberX=0
        self.vision_numberY=0

        #Gets sends the options to the SmartDashboard
        self.auto_chooser=wpilib.SendableChooser()
        self.auto_chooser.addObject("High Goal, Low Bar", "1")
        self.auto_chooser.addDefault("Crosser Low Bar", "2")
        self.auto_chooser.addObject("Reacher", "3")
        self.auto_chooser.addObject("Crosser RT", "4")
        wpilib.SmartDashboard.putData('Choice', self.auto_chooser)

        self.shooter_counter=0
        #Shooter speed
        self.shooter_high=.47
        self.updater()
        self.multiplier=1
        self.fire_counter=False
        self.ready=False
        self.auto_aline_auto=False
        self.ready_aline=False
        
    def autonomousInit(self):
        
        self.auto_motor=0
        self.auto_state=0
        self.ready_aline=False
        self.auto_aline_auto=False
        self.auto_drive1=0
        self.auto_drive2=0
        self.state=4
        self.arcade_drive.setSafetyEnabled(False)
        #Gets the choice
        self.final_choice=self.auto_chooser.getSelected()

    def autonomousPeriodic(self):

        if self.final_choice=="1":
            self.high_goal()
        elif self.final_choice=="2":
            self.low_bar_crosser()
        elif self.final_choice=="3":
            self.reacher()
        elif self.final_choice=="4":
            self.any_crosser()

    def high_goal(self):

        if self.auto_state == 0:
            self.timer.reset()
            self.navx.zeroYaw()
            self.auto_state=1
            self.drive1.set(0)
            self.drive2.set(0)
        #drive forward for x amount of time
        elif self.auto_state==1:
            self.drive1.set(-.455)
            self.drive2.set(.5)
            if self.timer.hasPeriodPassed(4):
                self.auto_state=2
        #turn 20 degrees to face target
        elif self.auto_state==2:
            if self.turn(-170):
                self.auto_state=3
        #Drive forward again
        elif self.auto_state==3:
            self.drive1.set(.52)
            self.drive2.set(-.5)
            if self.timer.hasPeriodPassed(6):
                self.auto_state=4
        #do a complete 180 to get ready to shoot
        elif self.auto_state==4:
            self.auto_aline_auto=True
            if self.ready_aline:
                self.auto_state=5
        elif self.auto_state==5:
            self.drive1.set(0)
            self.drive2.set(0)
            self.auto_state=6
            self.state=0

        self.fire()
        self.updater()
        self.vision()
        #Set all the motors and pistons
        self.shooter.set(self.speedShooter)
        self.arm1.set(self.shooter_piston)
        self.arm2.set(self.shooter_piston)
        #self.drive1.set((-1*self.auto_drive1))
        #self.drive2.set((1*self.auto_drive2))

    def low_bar_crosser(self):
        if self.auto_state == 0:
            self.timer.reset()
            self.navx.zeroYaw()
            self.auto_state=1
            self.auto_drive1=0
            self.auto_drive2=0
        elif self.auto_state==1:
            self.auto_drive2=.5
            self.auto_drive1=.5
            if self.timer.hasPeriodPassed(4):
                self.auto_state=2
        elif self.auto_state==2:
            self.auto_drive2=0
            self.auto_drive1=0

        self.shooter.set(self.speedShooter)
        self.arm1.set(self.shooter_piston)
        self.arm2.set(self.shooter_piston)
        self.drive1.set((-1*self.auto_drive1))
        self.drive2.set((1*self.auto_drive2))

    def any_crosser(self):
        if self.auto_state == 0:
            self.timer.reset()
            self.navx.zeroYaw()
            self.auto_state=1
            self.auto_drive1=0
            self.auto_drive2=0
        elif self.auto_state==1:
            self.auto_drive2=.5
            self.auto_drive1=.5
            if self.timer.hasPeriodPassed(2):
                self.auto_state=2

        elif self.auto_state==2:
            self.auto_drive2=1
            self.auto_drive1=1
            if self.timer.hasPeriodPassed(1.5): #For now
                self.auto_state=3
        elif self.auto_state==3:
            self.auto_drive2=0
            self.auto_drive1=0

        self.shooter.set(self.speedShooter)
        self.arm1.set(self.shooter_piston)
        self.arm2.set(self.shooter_piston)
        self.drive1.set((-1*self.auto_drive1))
        self.drive2.set((1*self.auto_drive2))

    def reacher(self):

        if self.auto_state == 0:
            self.timer.reset()
            self.navx.zeroYaw()
            self.auto_state=1
            self.auto_drive1=0
            self.auto_drive2=0
        elif self.auto_state==1:
            self.auto_drive2=.5
            self.auto_drive1=.5
            if self.timer.hasPeriodPassed(2):
                self.auto_state=2
        elif self.auto_state==2:
            self.auto_drive2=0
            self.auto_drive1=0

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
        if current > (degrees-10) and current < degrees:
            self.drive2.set(0)
            self.drive1.set(0)
            return True
        else:
            self.drive2.set(.7)
            self.drive1.set(-.5)


    def change_speed(self):
        """
        Changes speed of shooter with the buttons by the Xbox logo
        I use the counter to debounce the button a little so it doesn't hop up too much at a time
        """
        if self.shooter_counter==0:
           if self.lower_speed.get():
               self.shooter_high-=.005
           elif self.higher_speed.get():
               self.shooter_high+=.005
        elif self.lower_speed.get() or self.higher_speed.get() and self.shooter_counter==1:
            self.shooter_counter=2
        elif self.lower_speed.get() or self.higher_speed.get() and self.shooter_counter==2:
            self.shooter_counter=0

        if self.shooter_high>1:
            self.shooter_high=1
        elif self.shooter_high<.1:
            self.shooter_high=.1

    def teleopInit(self):
        #starting out the state at neutral motors
        self.state=4
        self.total_pan=0
        self.turn_state=2
        self.desired=0
        self.arcade_drive.setSafetyEnabled(True)

    def teleopPeriodic(self):

        #SMRT Dashboard updating
        self.change_speed()
        self.updater()

        #Starts the fire stuff
        if self.second_button.get() and self.fire_counter==False: #FIRE THE PISTON AND MOTORS#
            self.state = 0
            self.multiplier=1
        elif self.low_goal.get() and self.fire_counter==False:
            self.state=0
            self.multiplier=-.75

        self.fire()
        self.push_me_up()
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
        self.vision()
        #Set Everything that needs to be set
        self.arm1.set(self.shooter_piston)
        self.arm2.set(self.shooter_piston)
        self.shooter.set(self.speedShooter)
        self.cam.set(self.speedCam)
        #Lets drive!
        try:
            self.arcade_drive.arcadeDrive((self.boost*self.controller.getY()), (self.boost*(-1*self.controller.getX())))
        except:
            if not self.isFMSAttached():
                raise
        #I call it last because it needs to override the arcadedrive
        self.teleopTurn()

        
    def getControllerStates(self):
        
        #Gets the values of triggers for the Cam
        self.left=-1*(self.controller.getRawAxis(2))
        self.right=self.controller.getRawAxis(3)
        self.boost=(((self.left+self.right)*.4)-.1)+.7
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
        #Uses the right stick on the second controller to control the camera

        if self.second_controller.getPOV(0) in [45, 90, 135]:
            self.total_pan=self.total_pan+.025
        elif self.second_controller.getPOV(0) in [230, 270, 305]:
            self.total_pan=self.total_pan-.025

        if self.total_pan>1:
            self.total_pan=1
        elif self.total_pan<0:
            self.total_pan=0
        if self.forward_servo.get():
            self.total_pan=.5 #Or whatever the front is....subject to fine tuning
        elif self.ball_servo.get():
            self.total_pan=0
        self.servo.set(self.total_pan)

    def vision(self):
        """
        IT WORKS....mostly
        Press X to align the side to side
        Press Y to get the distance needed
        """
        #get data
        try:
            #self.vision_table.retrieveValue('centerX', self.vision_x)
            #self.vision_table.retrieveValue('centerY', self.vision_y)
            self.vision_x=[160, 170]
        except KeyError:
            pass
        else:
            if len(self.vision_x)>0 and self.auto_alineX.get() or self.auto_aline_auto:
                if len(self.vision_x)==1:
                    self.vision_numberX=self.vision_x[0]
                else:
                    good=self.vision_x[0]
                    normal=abs(self.vision_x[0]-160)
                    for i in self.vision_x[1:]:
                        total=abs(i-160)
                        if total <= normal:
                            normal=total
                            good=i
                    self.vision_numberX=good
                if self.vision_numberX > 180:
                    self.auto_calc=(((self.vision_numberX-180)/140)*.25)+.25
                    self.drive1.set(-1*self.auto_calc)
                    self.drive2.set(self.auto_calc)

                elif self.vision_numberX < 150:
                    self.auto_calc=(((150-self.vision_numberX)/150)*.25)+.25
                    self.drive1.set(self.auto_calc)
                    self.drive2.set(-1*self.auto_calc)
                else:
                    self.ready_aline=True

            elif len(self.vision_x)>0 and self.auto_alineY.get():
                self.vision_numberY=self.vision_y[0]

                if self.vision_numberY > 230:
                    self.auto_calc=(((self.vision_numberY-230)/90)*.25)+.25
                    self.drive1.set(-1*self.auto_calc)
                    self.drive1.set(self.auto_calc)

                elif self.vision_numberY < 200:
                    self.auto_calc=(((200-self.vision_numberY)/200)*.25)+.25
                    self.drive1.set(self.auto_calc)
                    self.drive2.set(-1*self.auto_calc)
                else:
                    self.ready_aline=False
    def fire(self):
        """
        This function is the automated shooter. Fires piston out, spins motor to speed, fires back
        """
        if self.state == 0:
            self.timer.reset()
            self.state=1
            self.ready=False
            self.speedShooter=0

        elif self.state == 1:
            self.fire_counter=True
            self.controller.setRumble(1, .9)
            self.second_controller.setRumble(1, .9)
            self.shooter_piston=2
            self.speedShooter=0

            if self.timer.hasPeriodPassed(.75):
                self.speedShooter=self.shooter_high*self.multiplier
                self.state=2

        elif self.state==2:
            self.shooter_piston=2
            self.speedShooter=self.shooter_high*self.multiplier
            if self.timer.hasPeriodPassed(3) or self.hold_button.get() or self.ready==True:
                if not self.hold_button.get():
                    self.speedShooter=self.shooter_high*self.multiplier
                    self.shooter_piston=1
                    self.state=3
                self.ready=True


        elif self.state == 3:
            self.speedShooter=self.shooter_high*self.multiplier
            if self.timer.hasPeriodPassed(.75):
                self.state=4

        elif self.state==4 and self.joystick_button.get():
            self.intake()

        elif self.state==4:
            self.fire_counter=False
            self.speedShooter=0
            #self.shooter_piston=1 I could change this and see if that helps stop the immediate retract
            self.controller.setRumble(1, 0)
            self.second_controller.setRumble(1, 0)

    def push_me_up(self):
        if self.fire_buttoms_up.get():
            self.bottoms_up.set(True)
        else:
            self.bottoms_up.set(False)

    def intake(self):
        #This might be a problem if the pistons fire before the motors are ready
        self.speedShooter=.17
        self.shooter_piston=2

    def amIStuck(self):
        if self.navx.getVelocityY()<.2 and self.navx.getVelocityY() >-.2:
            return True

    def teleopTurn(self):
        """
        Takes the current position and tries to do a 180 for the shooter
        """
        current=self.navx.getYaw()
        if self.turn_state==0:
            #Sees if the navx freaks out and spits out 0's >>>>>>>>>>>>>>>>>>>>>>>
            if current < (self.desired+5) and current > (self.desired-5) or current==0:
                self.turn_state=2
            else:
                if self.cancel.get(): #Press B to cancel the turning
                    self.turn_state=2
                self.drive2.set(.7)
                self.drive1.set(.5)


    def updater(self):
        ##Put all smartdashboard things here
        if self.shooter_piston==1:
            self.piston_update="PISTON IN"
        else:
            self.piston_update="PISTON OUT"
        wpilib.SmartDashboard.putString('Piston', self.piston_update)
        wpilib.SmartDashboard.putNumber('Roll', self.navx.getRoll())#Just seeing if this data is useful to us
        wpilib.SmartDashboard.putNumber('Pitch', self.navx.getPitch())
        wpilib.SmartDashboard.putNumber('Yaw', self.navx.getYaw())
        wpilib.SmartDashboard.putNumber('Velocity', self.navx.getVelocityY())
        wpilib.SmartDashboard.putNumber('Speed', self.shooter_high)

    def disabledPeriodic(self):
        ##Updated values when disabled
        self.updater()


if __name__ == "__main__":
    wpilib.run(MyRobot)
