from wpilib import Joystick, XboxController, Timer, DriverStation
from wpilib.interfaces import GenericHID
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    HolonomicDriveController,
)
from ..utils import remap
import wpimath
from wpimath.units import feetToMeters
from wpimath.trajectory import (
    TrajectoryGenerator,
    TrajectoryConfig,
    TrapezoidProfileRadians,
    Trajectory,
)
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
import math
import config
from wpimath.kinematics import ChassisSpeeds
from pathplannerlib import (
    PathConstraints,
    PathPlanner,
    PathPoint,
    PathPlannerTrajectory,
    controllers,
)
from magicbot import feedback
from wpilib import SmartDashboard
import os
from wpimath.units import inchesToMeters
from wpilib import Timer
from wpimath.filter import SlewRateLimiter

MAX_TRAJECTORY_SPEED = feetToMeters(5)
MAX_TRAJECTORY_ACCEL = feetToMeters(5)

RIGHT_RUMBLE = GenericHID.RumbleType.kRightRumble
LEFT_RUMBLE = GenericHID.RumbleType.kLeftRumble

# config for saitek joystick
# self.driverController = FROGStick(0, 0, 1, 3, 2)
# config for Logitech Extreme 3D
# self.driverController = FROGStick(0, 0, 1, 2, 3)
SAITEK_AXIS_CONFIG = {"xAxis": 0, "yAxis": 1, "rAxis": 3, "tAxis": 2}
LOGITECH_EXTREME_AXIS_CONFIG = {"xAxis": 0, "yAxis": 1, "rAxis": 2, "tAxis": 3}


class FROGStickDriver(Joystick):
    """Extended class of wpilib.Joystick

    Returns:
        FROGStick: Custom Joystick class
    """

    DEADBAND = 0.025
    SPEED_DIVISOR = 1
    ROTATION_DIVISOR = 1.6
    ROTATION_MIN = 0
    ROTATION_MAX = 0.5
    DEBOUNCE_PERIOD = 0.5

    def __init__(
        self, port: int, xAxis: int = 1, yAxis: int = 2, rAxis: int = 3, tAxis: int = 4
    ) -> None:
        """Constructor for FROGStick

        :param port: The port on the Driver Station that the joystick
        is plugged into (0-5).
        :param xAxis: channel for the X axis
        :param yAxis: channel for the Y axis
        :param rAxis: channel for the rotation (twist) axis
        :param tAxis: channel for the throttle axis
        """

        super().__init__(port)
        self.setThrottleChannel(tAxis)
        self.setTwistChannel(rAxis)
        self.setXChannel(xAxis)
        self.setYChannel(yAxis)
        self.button_latest = {}
        self.timer = Timer

    def getFieldForward(self):
        """Get's the joystick's Y axis and
        inverts it so pushing forward is positive
        and translates to chassis moving away from
        driver.

        Returns:
            float: -1 to 1
        """
        # inverts the joystick's Y axis so pushing
        # forward is positive and pulling back is
        # negative

        return wpimath.applyDeadband(-self.getY(), self.DEADBAND)

    def getFieldLeft(self):
        """Get's the joystick's X axis and
        inverts it so pushing left is positive
        and translates to chassis moving to the
        left of the driver.

        Returns:
            float: -1 to 1
        """
        # inverts the joystick's X axis so pushing
        # left is positive and pushing right is negative
        return wpimath.applyDeadband(-self.getX(), self.DEADBAND)

    def getFieldRotation(self):
        """Get's the joystick's Twist axis and
        inverts it so twisting CCW is positive
        and translates to chassis rotating CCW.

        Returns:
            float: -1 to 1
        """
        # inverts the joystick's twist axis so CCW
        # is positive and CW is negative
        return wpimath.applyDeadband(-self.getTwist(), self.DEADBAND)

    def get_speed(self):
        # Dampens the -1 to 1 values of the joystick to provide a smoothed acceleration
        speed = self.getY()
        speed = -1 * (
            speed**3 / self.SPEED_DIVISOR if abs(speed) > self.DEADBAND else 0
        )
        return speed

    def getFieldThrottle(self):
        val = -super().getThrottle()
        throttle = (val + 1) / 2
        return throttle

    def get_rotation(self):
        return (
            self.getTwist() / self.ROTATION_DIVISOR
            if abs(self.getTwist()) > self.DEADBAND
            else 0
        )

    def getRangedCubedRotation(self):
        return remap(
            self.getTwist() ** 3,
            self.SPEED_DIVISOR,
            1,
            self.ROTATION_MIN,
            self.ROTATION_MAX,
        )

    def getRangeRotation(self):
        return remap(
            self.getTwist(),
            self.SPEED_DIVISOR,
            1,
            self.ROTATION_MIN,
            self.ROTATION_MAX,
        )

    def get_button(self, num):
        val = self.getRawButton(num)
        return val

    def get_debounced_button(self, num):
        """Returns the value of the joystick button. If the button is held down, then
        True will only be returned once every ``debounce_period`` seconds"""
        val = False
        now = self.timer.getFPGATimestamp()
        if self.getRawButton(num):
            if (now - self.button_latest.get(num, 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest[num] = now
                val = True
        return val


class FROGXboxDriver(XboxController):
    DEADBAND = 0.15
    ROTATION_DIVISOR = 1
    DEBOUNCE_PERIOD = 0.5
    MODE = 0  # run auto routines

    def __init__(self, channel):
        super().__init__(channel)
        self.button_latest = {}
        self.timer = Timer()
        self.xSlew = SlewRateLimiter(1.5)
        self.ySlew = SlewRateLimiter(1.5)
        self.rotSlew = SlewRateLimiter(1.5)

    def getFieldRotation(self):
        return wpimath.applyDeadband(-self.getRightX(), self.DEADBAND)
    
    def getSlewLimitedFieldRotation(self):
        return self.rotSlew.calculate(
            self.getFieldRotation()
        )

    def getFieldForward(self):
        return wpimath.applyDeadband(-self.getLeftY(), self.DEADBAND)
    
    def getSlewLimitedFieldForward(self):
        return self.xSlew.calculate(
            self.getFieldForward()
        )

    def getFieldLeft(self):
        return wpimath.applyDeadband(-self.getLeftX(), self.DEADBAND)
    
    def getSlewLimitedFieldLeft(self):
        return self.ySlew.calculate(
            self.getFieldLeft()
        )

    def getFieldThrottle(self):
        return wpimath.applyDeadband(self.getRightTriggerAxis(), 0)

    def getPOVDebounced(self):
        val = -1
        now = self.timer.getFPGATimestamp()
        pov = self.getPOV()
        if pov > -1:
            if (now - self.button_latest.get("POV", 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest["POV"] = now
                val = pov
        if (now - self.button_latest.get("POV", 0)) < self.DEBOUNCE_PERIOD:
            self.setRumble(RIGHT_RUMBLE, 1)
        else:
            self.setRumble(RIGHT_RUMBLE, 0)
        # self.update_nt("button_pov", val)
        return val

class FROGXboxOperator(XboxController):
    DEBOUNCE_PERIOD = 0.5

    def __init__(self, channel):
        super().__init__(channel)
        self.timer = Timer()
        self.button_latest = {}
        self.manualMode = False

    def getPOVDebounced(self):
        val = -1
        now = self.timer.getFPGATimestamp()
        pov = self.getPOV()
        if pov > -1:
            if (now - self.button_latest.get("POV", 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest["POV"] = now
                val = pov
        if (now - self.button_latest.get("POV", 0)) < self.DEBOUNCE_PERIOD:
            self.setRumble(RIGHT_RUMBLE, 1)
        else:
            self.setRumble(RIGHT_RUMBLE, 0)
        return val

    def changeMode(self):
        return [True, False][self.manualMode]

    def getManualMode(self):
        return self.manualMode


class PPHolonomic(controllers.PPHolonomicDriveController):

    def __init__(self, kinematics):
        # the holonomic controller
        self.xController = config.ppXPIDController
        self.yController = config.ppYPIDController
        self.rotationController = config.ppRotationPIDController
        # self.angleController.enableContinuousInput(-1 * math.pi, math.pi)
        super().__init__(self.xController, self.yController, self.rotationController)
        self.timer = Timer()
        self.trajectoryType = False
        self.kinematics = kinematics
        self.max_trajectory_speed = config.MAX_TRAJECTORY_SPEED
        self.max_trajectory_accel = config.MAX_TRAJECTORY_ACCEL
        self.setTolerance(config.ppTolerance)
        self.pastMarker = None
        self.nextMarker = None


    def initialize(self, trajectoryType):
        self.timer.stop()
        self.timer.reset()
        self.firstCall = True
        self.trajectoryType = trajectoryType
        if trajectoryType == 'pathPlanner':
            self.eventMarkers = self.trajectory.getMarkers()
        else:
            self.eventMarkers = []
        if self.eventMarkers:
            self._loadNextMarker()

    def setTrajectoryConstraints(self, max_trajectory_speed, max_trajectory_accel):
        self.max_trajectory_speed = max_trajectory_speed
        self.max_trajectory_accel = max_trajectory_accel

    def loadPID(self):
        self.xController.setP(
            SmartDashboard.getNumber("TranslationControllerP", self.xController.getP())
        )
        self.yController.setP(
            SmartDashboard.getNumber("TranslationControllerP", self.yController.getP())
        )
        self.xController.setI(
            SmartDashboard.getNumber("TranslationControllerI", self.xController.getI())
        )
        self.yController.setI(
            SmartDashboard.getNumber("TranslationControllerI", self.yController.getI())
        )
        self.xController.setD(
            SmartDashboard.getNumber("TranslationControllerD", self.xController.getD())
        )
        self.yController.setD(
            SmartDashboard.getNumber("TranslationControllerD", self.yController.getD())
        )
        self.rotationController.setP(
            SmartDashboard.getNumber("RotationControllerP", self.rotationController.getP())
        )

    def postError(self):
        SmartDashboard.putNumber('PID_X_ERROR', self.xController.getVelocityError())
        SmartDashboard.putNumber('PID_Y_ERROR', self.yController.getVelocityError())
        SmartDashboard.putNumber('PIDRotERROR', self.rotationController.getVelocityError())

    def initPoseToPose(self, startPose, endPose):
        startPoint = PathPoint(startPose.translation(), startPose.rotation())
        endPoint = PathPoint(endPose.translation(), endPose.rotation())
        self.initSimpleTrajectory(startPoint, endPoint)

    def initSimpleTrajectory(self, startPoint: PathPoint, endPoint: PathPoint):
        """Initializes a PathPlanner trajectory"""
        self.trajectory = PathPlanner.generatePath(
            PathConstraints(self.max_trajectory_speed, self.max_trajectory_accel),
            False,
            [startPoint, endPoint],
        )
        self.initialize("pathPlanner")

    def loadPathPlanner(self, pathName):
        """Loads a PathPlanner trajectory from a preconfigured path.

        Args:
            pathName (_type_): The name of the path, without the .path extension.
        """
        self.trajectory = PathPlanner.loadPath(
            os.path.join(os.path.dirname(__file__), r"..", r"paths", pathName),
            PathConstraints(self.max_trajectory_speed, self.max_trajectory_accel),
            False,
        )
        self.trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(
            self.trajectory, DriverStation.getAlliance()
        )
        # list of event markers pathplannerlib._pathplannerlib.EventMarker
        # each marker has names, position, and time.  e.g.
        # marker.names
        # > ['DropArm']
        # marker.time
        # > 1.1736999735993974
        # marker.position
        # > Translation2d(x=2.210424, y=4.428544)
        self.initialize("pathPlanner")

    def getPastMarker(self):
        return self.pastMarker
    
    def _loadNextMarker(self):
        if self.nextMarker:
            self.pastMarker = self.nextMarker
        if self.eventMarkers:
            self.nextMarker = self.eventMarkers.pop(0)
        else:
            self.nextMarker = None

    def getChassisSpeeds(self, currentPose: Pose2d) -> ChassisSpeeds:
        """Calculates the chassis speeds of the trajectory at the current time.

        Args:
            currentPose (Pose2d): current pose of the Robot

        Returns:
            ChassisSpeeds: translation and rotational vectors desired
        """
        #self.postError()
        SmartDashboard.putBoolean("AT TARGET", self.atReference())
        if not self.trajectoryType is None:
            if self.firstCall:
                self.timer.start()
                self.firstCall = False
            currentTime = self.timer.get()
            if self.nextMarker:
                if currentTime >= self.nextMarker.time:
                        self.pastMarker = self.nextMarker
                        self._loadNextMarker()
                
            # get the pose of the trajectory at the current time
            referenceState = self.trajectory.sample(currentTime)
            return self.calculate(currentPose, referenceState)
        if self.atReference():
            self.timer.stop()
            self.trajectoryType = None