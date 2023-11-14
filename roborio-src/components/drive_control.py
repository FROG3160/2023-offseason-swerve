from magicbot import state, default_state, tunable
from magicbot.state_machine import StateMachine
from components.drivetrain import SwerveChassis
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d
from wpimath.trajectory import TrapezoidProfileRadians
from components.controllers import PPHolonomic, FROGXboxDriver
from components.field import FROGFieldLayout
from ntcore.util import ChooserControl
from wpilib import SendableChooser, SmartDashboard
from wpilib.shuffleboard import Shuffleboard
import os, math, config
from components.sensors import FROGGyro

povSpeed = 0.1
povSpeeds = {
    0: (povSpeed, 0),
    45: (povSpeed, -povSpeed),
    90: (0, -povSpeed),
    135: (-povSpeed, -povSpeed),
    180: (-povSpeed, 0),
    225: (-povSpeed, povSpeed),
    270: (0, povSpeed),
    315: (povSpeed, povSpeed)
}
class DriveControl(StateMachine):
    swerveChassis: SwerveChassis
    limelight: FROGLimeLightVision
    driverController: FROGXboxDriver
    gyro: FROGGyro

    def __init__(self) -> None:
        self._vX = 0
        self._vY = 0
        self._vT = 0
        self._throttle = 0
        self._endPose: Pose2d = None
        self._pathName = None
        self._object = None
        self.resetController = True
        self.pathChooser = SendableChooser()
        for path in [n.rsplit('.', 1)[0] for n in os.listdir(os.path.join(os.path.dirname(__file__), '..', r"paths"))]:
            self.pathChooser.addOption(path, path)

        profiledRotationConstraints = TrapezoidProfileRadians.Constraints(
            config.cfgProfiledMaxVelocity, config.cfgProfiledMaxAccel
        )
        self.profiledRotationController = ProfiledPIDControllerRadians(
            config.cfgProfiledP,
            config.cfgProfiledI,
            config.cfgProfiledD,
            profiledRotationConstraints,
        )
        self.profiledRotationController.enableContinuousInput(-math.pi, math.pi)

        rotationControllerTab = Shuffleboard.getTab("Rotation Controller")
        rotationControllerTab.add(title="profiledRotationController", defaultValue=self.profiledRotationController)

        SmartDashboard.putData("Path", self.pathChooser)

    def setup(self):
        self.holonomic = PPHolonomic(self.swerveChassis.kinematics)
        self.resetRotationController()

    def resetRotationController(self):
        self.profiledRotationController.reset(math.radians(self.gyro.getYawCCW()), self.gyro.getRadiansPerSecCCW())

    def holonomicDriveToWaypoint(self, waypoint:Pose2d):
        self._endPose = waypoint
        self.engage(initial_state='driveToWayPoint')

    def holonomicDrivePath(self, pathName = None):
        if pathName:
            self._pathName = pathName
        else:
            self.logger.info(f'Getting path from chooser.')
            self._pathName = self.pathChooser.getSelected()
        self.engage(initial_state='drivePath')

    def driveToChargingReverse(self):
        self.engage(initial_state='balanceOnChargingReverse')

    def driveToChargingForward(self):
        self.engage(initial_state='balanceOnChargingForward')

    def lockWheels(self):
        self.engage(initial_state='lock')
        
    def angleErrorToRotation(error):
    #return math.copysign(math.exp(0.0352*abs(error))*0.0496, error)
    # 0.0001*(B2^2) + 0.0024 *B2 - 0.0039
        abs_error = abs(error)
        vT = (abs_error**2) * 0.0001 + abs_error * 0.0024 - 0.0039
        if vT < 0:
            return 0
        else:
            return math.copysign(vT, error)
        
    def rollToSpeedFactor(self, gyroRoll):
        pass
        


    # State fieldOriented (as the default state) This will be the first state.
    @default_state
    def fieldOriented(self, initial_call):
        if initial_call:
            self.swerveChassis.enableMinSpeed()
        rightStickY = self.driverController.getRightY()
        if rightStickY > 0.5:
            if self.resetController:
                # this is the first time we hit this conditional, so 
                # reset the controller
                self.logger.info('Resetting profiledRotationController')
                self.resetController = False
                self.resetRotationController()
            #Rotate to 0 degrees, point downfield
            vT = self.profiledRotationController.calculate(
                math.radians(self.gyro.getYawCCW()), math.radians(0)
            )
        elif rightStickY < -0.5:
            if self.resetController:
                # this is the first time we hit this conditional, so 
                # reset the controller
                self.logger.info('Resetting profiledRotationController')
                self.resetController = False
                self.resetRotationController()
            #Rotate to 180 degrees
            vT = self.profiledRotationController.calculate(
                math.radians(self.gyro.getYawCCW()), math.radians(180)
            )            
        else:
            # set to true so the first time the other if conditionals evaluate true
            # the controller will be reset
            self.resetController = True
            vT = self.driverController.getSlewLimitedFieldRotation()
        pov = self.driverController.getPOV()
        if pov != -1:
            vX, vY = povSpeeds[pov]
        else:
            vX = self.driverController.getSlewLimitedFieldForward()
            vY = self.driverController.getSlewLimitedFieldLeft()

        

        self.swerveChassis.fieldOrientedDrive(
            #self._vX, self._vY, self._vT, self._throttle
            vX,
            vY,
            vT,
            self.driverController.getFieldThrottle(),
        )

    # State robotOriented (for driving to cones, cubes and posts).
    @state()
    def robotOriented(self, initial_call):
        if initial_call:
            self.swerveChassis.enableMinSpeed()
        self.swerveChassis.robotOrientedDrive(self._vX, self._vY, self._vT)

   
    @state()
    def balanceOnChargingForward(self, initial_call):
        if initial_call:
            self.swerveChassis.enableMinSpeed()
        rollAngle = 8
        goalHeading = 0
        
        vT = self.profiledRotationController.calculate(
                math.radians(self.gyro.getYawCCW()), math.radians(goalHeading)
            )
        
        if self.limelight.getBotPoseEstimateForAlliance()[0].X() > 3.89 + 0.04:
            vX = 0
        elif self.gyro.getRoll() < -rollAngle:
            vX = 0.2
        else:
            vX = 0.5
        
        self.logger.info(f"Roll angle: {self.gyro.getRoll()}")
        self.logger.info(f'X position: {self.limelight.getBotPoseEstimateForAlliance()[0].X()}')
        self.logger.info(f"Speeds: {vX}, 0, {vT}")

        self.swerveChassis.fieldOrientedDrive(vX, 0, vT)

    @state()
    def balanceOnChargingReverse(self, initial_call):
        if initial_call:
            self.swerveChassis.enableMinSpeed()
        rollAngle = 8
        goalHeading = 0
        
        vT = self.profiledRotationController.calculate(
                math.radians(self.gyro.getYawCCW()), math.radians(goalHeading)
            )
        
        if self.limelight.getBotPoseEstimateForAlliance()[0].X() < 3.89 + 0.04:
            vX = 0
        elif self.gyro.getRoll() > rollAngle:
            vX = -0.2

        else:
            vX = -0.5
        
        self.logger.info(f"Roll angle: {self.gyro.getRoll()}")
        self.logger.info(f'X position: {self.limelight.getBotPoseEstimateForAlliance()[0].X()}')
        self.logger.info(f"Speeds: {vX}, 0, {vT}")

        self.swerveChassis.fieldOrientedDrive(vX, 0, vT)
        


    @state()
    def drivePath(self, initial_call):
        if initial_call:
            self.holonomic.loadPathPlanner(self._pathName)
            self.swerveChassis.disableMinSpeed()
        newspeeds = self.holonomic.getChassisSpeeds(
            self.swerveChassis.estimator.getEstimatedPosition()
        )


        self.swerveChassis.holonomicDrive(newspeeds)

    # State locked (for turning the wheels in to keep it from moving).
    @state()
    def lock(self, initial_call):
        if initial_call:
            self.swerveChassis.lockChassis()

    