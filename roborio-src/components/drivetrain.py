import math
from logging import Logger

import config
from components.controllers import PPHolonomic
from components.field import FROGFieldLayout
from components.sensors import FROGGyro
from components.vision import FROGLimeLightVision
from phoenix6 import (TalonFX, CANcoder, TalonFXConfiguration,
                      CANcoderConfiguration,
                      PositionDutyCycle, VelocityDutyCycle)
from phoenix6.configs.cancoder_configs import AbsoluteSensorRangeValue, SensorDirectionValue
from magicbot import feedback
from ..utils import DriveUnit, remap
from wpilib import Field2d, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import (ChassisSpeeds, SwerveDrive4Kinematics,
                                SwerveModulePosition, SwerveModuleState)
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.units import feetToMeters, metersToInches

from .sensors import FROGGyro

# Motor Control modes
VELOCITY_MODE = VelocityDutyCycle
POSITION_MODE = PositionDutyCycle

MAX_TRAJECTORY_SPEED = feetToMeters(5)
MAX_TRAJECTORY_ACCEL = feetToMeters(5)


class GearStages:
    def __init__(self, gear_stages: list):
        """
        Constructs a GearStages object that stores data about the gear stages.
        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
        """
        self.gearing = math.prod(gear_stages)

    # TODO: need to re-evalute the names of these methods.  How can we make it more
    # explicit?
    def toMotor(self, rotations):
        """Calculates motor rotations given the rotation at the other end of the gears."""
        return rotations / self.gearing

    def fromMotor(self, rotations):
        """Calculates final gear rotation given the motor's rotation"""
        return rotations * self.gearing


class DriveUnit:
    def __init__(self, gear_stages: list, diameter: float):
        """Constructs a DriveUnit object that stores data about the motor, gear stages, and wheel
           that makes up a complete power train.
        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
            diameter (float): Diameter of the attached wheel in meters
        """
        self.gearing = GearStages(gear_stages)
        self.circumference = math.pi * diameter

    def speedToVelocity(self, speed: float) -> float:
        """Converts the system linear speed to a motor velocity
        Args:
            speed (float): desired linear speed in meters per second
        Returns:
            float: motor rotations per second
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = self.gearing.toMotor(wheel_rotations_sec)
        return motor_rotations_sec

    def velocityToSpeed(self, rotations_per_sec: float) -> float:
        """Converts motor velocity to the system linear speed
        
        Args:
            velocity (float): motor velocity in encoder counts per 100ms
        Returns:
            float: system linear speed in meters per second
        """
        wheel_rotations_sec = self.gearing.fromMotor(rotations_per_sec)
        return wheel_rotations_sec * self.circumference

    def positionToDistance(self, rotations: float) -> float:
        """Takes encoder count and returns distance

        Args:
            position (int): number of encoder counts

        Returns:
            float: distance in meters
        """
        wheel_rotations = self.gearing.fromMotor(rotations)
        return wheel_rotations * self.circumference


class SwerveModule:
    def __init__(
        self,
        name: str,
        drive_motor_id: int,
        steer_motor_id: int,
        steer_sensor_id: int,
        steer_sensor_offset: float,
        location: Translation2d,
        driveMotorPID: TalonFXConfiguration
    ):
        # set initial states for the component
        self.name = name
        self.drive = TalonFX(drive_motor_id)
        self.steer = TalonFX(steer_motor_id)
        self.encoder = CANcoder(steer_sensor_id)
        self.steerOffset = steer_sensor_offset
        self.location = location
        self.driveMotorPID = driveMotorPID
        self.drive_unit = DriveUnit(
            config.MODULE_DRIVE_GEARING,
            config.FALCON_MAX_RPM,
            config.MODULE_WHEEL_DIAMETER,
            config.FALCON_TICKS_PER_ROTATION,
        )
        self.configModuleComponents()
        self.useMinSpeed = True

        # self.velocity = 0
        # self.angle = 0
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def getEncoderAbsolutePosition(self) -> float:
        """gets the absolute position from the CANCoder
        Returns:
            float: position of the sensor in degrees (-180 to 180)
        """
        return self.encoder.getAbsolutePosition()

    # TODO: Determine which way we want these
    # to read.  Right now they are inverted
    # to visually show positive angles to the
    # right (clockwise) to match the smartdashboard
    # @feedback()
    # def getCommandedDegrees(self):
    #     return -self.requestedState.angle.degrees()

    # TODO: rewrite this whole thing so execute updates attributes
    # TODO: and the attributes are read by these methods.
    @feedback
    def getCurrentRotation(self) -> Rotation2d:
        if degrees := self.getEncoderAbsolutePosition():
            return Rotation2d.fromDegrees(degrees)
        else:
            return Rotation2d.fromDegrees(0)

    def getSteerPosition(self):
        return self.steer.getSelectedSensorPosition(0)

    def getActualVelocity(self):
        return self.drive.getSelectedSensorVelocity()

    def getCurrentDistance(self) -> float:
        """Gets distance traveled by the system.

        Returns:
            float: distance in meters
        """
        return self.drive_unit.positionToDistance(
            self.drive.getSelectedSensorPosition()
        )

    @feedback
    def getCurrentSpeed(self) -> float:
        return self.drive_unit.velocityToSpeed(self.drive.getSelectedSensorVelocity())

    # TODO: see TODO on getCurrentRotation()
    def getCurrentState(self):
        return SwerveModuleState(
            self.getCurrentSpeed(),
            self.getCurrentRotation(),
        )

    def getCurrentPosition(self):
        return SwerveModulePosition(
            self.getCurrentDistance(), self.getCurrentRotation()
        )

    def configModuleComponents(self):
        # configure CANCoder
        cancoder_config = CANcoderConfiguration()
        cancoder_config.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        cancoder_config.magnet_sensor.magnet_offset = self.steerOffset
        cancoder_config.magnet_sensor.sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
        self.encoder.configurator.apply(cancoder_config)
        # TODO:  See if we can, or need to set initialization strategy for the CANCoder
        # self.encoder.configSensorInitializationStrategy(
        #     SensorInitializationStrategy.BootToAbsolutePosition
        # )

        self.steer.configAllSettings(config.cfgSteerMotor)
        self.steer.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250)
        self.steer.setInverted(TalonFXInvertType.CounterClockwise)  # was Clockwise
        # define the remote CANCoder as Remote Feedback 0
        self.steer.configRemoteFeedbackFilter(
            self.encoder.getDeviceNumber(), RemoteSensorSource.CANCoder, 0
        )
        # configure Falcon to use Remote Feedback 0
        self.steer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
        self.steer.configIntegratedSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        )
        self.steer.configIntegratedSensorAbsoluteRange(
            AbsoluteSensorRange.Signed_PlusMinus180
        )
        self.steer.setSensorPhase(False)
        self.steer.setNeutralMode(NeutralMode.Brake)

        # configure drive motor
        self.drive.configAllSettings(self.driveMotorPID)
        self.drive.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250)
        self.drive.setInverted(TalonFXInvertType.Clockwise)
        self.drive.setSensorPhase(False)
        self.drive.configClosedloopRamp(0.25)
        self.drive.configVoltageCompSaturation(config.VOLTAGE_COMPENSATION)
        self.drive.enableVoltageCompensation(True)
        self.drive.setNeutralMode(NeutralMode.Brake)

    def enableMinSpeed(self):
        self.useMinSpeed = True
    
    def disableMinSpeed(self):
        self.useMinSpeed = False

    def setState(self, requested_state: SwerveModuleState):
        if self.enabled:
            #
            # using built-in optimize method instead of our custom one from last year
            self.requestedState = SwerveModuleState.optimize(requested_state, self.getCurrentRotation())

            self.steer.set(
                POSITION_MODE,
                requested_state.angle * config.CANCODER_TICKS_PER_RADIAN
            )

            self.drive.set(
                VELOCITY_MODE,
                self.drive_unit.speedToVelocity(self.requestedState.speed),
            )
        else:
            self.drive.set(0)

    def setDriveF(self, newF):
        self.drive.config_kF(0, newF)

    # def initSendable(self, builder) -> None:

    #     super().initSendable(builder)

    def periodic(self) -> None:
        pass
        SmartDashboard.putNumber(
            "{}_steerAngle".format(self.name), self.getCurrentRotation().degrees()
        )
        SmartDashboard.putNumber(
            "{}_driveSpeed".format(self.name), self.getActualVelocity()
        )
        SmartDashboard.putNumber(
            "{}_requestedAngle".format(self.name), self.requestedState.angle.degrees()
        )
        SmartDashboard.putNumber(
            "{}_requestedSpeed".format(self.name), self.drive_unit.speedToVelocity(self.requestedState.speed*self.requestedState.invert_speed)
        )
        SmartDashboard.putNumber(
            "{}_speedError".format(self.name), abs(self.drive_unit.speedToVelocity(self.requestedState.speed)) - abs(self.getActualVelocity())
        )


class SwerveChassis:
    moduleFrontLeft: SwerveModule
    moduleFrontRight: SwerveModule
    moduleBackLeft: SwerveModule
    moduleBackRight: SwerveModule

    logger: Logger

    fieldLayout: FROGFieldLayout
    #ISSUE https://github.com/FROG3160/2023-offseason-swerve/issues/38 Add FROGLimeLightVision from last year's code
    limelight: FROGLimeLightVision

    gyro: FROGGyro

    def __init__(self):
        self.enabled = False
        # TODO: Adjust for field placement

        #self.limelightPoseEstimator = FROGLimeLightVision()
        self.center = Translation2d(0, 0)


    def setup(self):

        self.modules = (
            self.moduleFrontLeft,
            self.moduleFrontRight,
            self.moduleBackLeft,
            self.moduleBackRight,
        )

        self.moduleStates = (
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
        )
        self.current_speeds = ChassisSpeeds(0, 0, 0)

        self.kinematics = SwerveDrive4Kinematics(
            # the splat operator (asterisk) below expands
            # the list into positional arguments for the
            # kinematics object.  We are taking the location
            # property of each swerveModule object and passing
            # it to SwerveDrive4Kinematics the order defined by
            # self.modules above.  Order is critical here.
            # We will receive back drive and steer values for
            # each SwerveModule in the same order we use here.
            *[m.location for m in self.modules]
        )

        self.trajectoryConfig = TrajectoryConfig(
            MAX_TRAJECTORY_SPEED, MAX_TRAJECTORY_ACCEL
        )
        self.trajectoryConfig.setKinematics(self.kinematics)

        self.gyro.resetGyro()

        self.chassisSpeeds = ChassisSpeeds(0, 0, 0)
        self.startingPose2d = Pose2d()

        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            tuple(
                [SwerveModulePosition(0, x.getCurrentRotation()) for x in self.modules]
            ),
            self.startingPose2d,
        )
        # TODO: Adjust the stdDevs
        self.estimator.setVisionMeasurementStdDevs((0.5, 0.5, math.pi/2))
        self.field = Field2d()

    def disable(self):
        self.enabled = False
        for module in self.modules:
            module.disable()

    def disableMinSpeed(self):
        for module in self.modules:
            module.disableMinSpeed()

    def enableMinSpeed(self):
        for module in self.modules:
            module.enableMinSpeed()

    def enable(self):
        self.enabled = True
        for module in self.modules:
            module.enable()

    def lockChassis(self):
        self.moduleFrontLeft.setState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.moduleBackRight.setState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.moduleFrontRight.setState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.moduleBackLeft.setState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))

    def setModuleStates(self, states):
        self.moduleStates = states

    def setFieldPosition(self, pose: Pose2d):
        self.estimator.resetPosition(
            self.gyro.getRotation2d(),
            tuple(self.getModulePositions()),
            pose,
        )

    def getModuleStates(self):
        return [module.getCurrentState() for module in self.modules]

    def getModulePositions(self):
        return [module.getCurrentPosition() for module in self.modules]

    def setStatesFromSpeeds(self):
        states = self.kinematics.toSwerveModuleStates(self.chassisSpeeds, self.center)
        states = self.kinematics.desaturateWheelSpeeds(
            states, config.MAX_METERS_PER_SEC
        )
        self.moduleStates = states

    def fieldOrientedDrive(self, vX: float, vY: float, vT: float, throttle=1.0):
        xSpeed = vX * config.MAX_METERS_PER_SEC * throttle
        ySpeed = vY * config.MAX_METERS_PER_SEC * throttle
        rotSpeed = vT * config.MAX_CHASSIS_RADIANS_SEC * throttle
        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotSpeed, self.gyro.getRotation2d()
        )

    def getChassisVelocityFPS(self):
        return math.sqrt( self.chassisSpeeds.vx_fps**2 + self.chassisSpeeds.vy_fps**2)
    
    def getHeadingRadians(self):
        return math.atan2( self.chassisSpeeds.vy, self.chassisSpeeds.vx )

    def holonomicDrive(self, chassisSpeeds) -> None:
        self.chassisSpeeds = chassisSpeeds

    def robotOrientedDrive(self, vX, vY, vT):
        self.logger.info(f'Velocities: {vX}, {vY}, {vT}')
        self.chassisSpeeds = ChassisSpeeds(vX, vY, vT)

    def execute(self):
        if self.enabled:
            self.setStatesFromSpeeds()#apply chassis Speeds

            for module, state in zip(self.modules, self.moduleStates):
                module.setState(state)
        self.periodic()

    def periodic(self) -> None:
        self.estimatorPose = self.estimator.update(
            Rotation2d.fromDegrees(self.gyro.getYawCCW()),
            tuple(self.getModulePositions()),
        )
        visionPose, visionTime = self.limelight.getBotPoseEstimateForAlliance()
        if visionPose:
            if (
                abs(visionPose.x - self.estimatorPose.x) < 0.5
                and abs(visionPose.y - self.estimatorPose.y) < 0.5
            ):
                stddevupdate = remap(visionPose.x,2.0, 8.0, 0.3, 2.0)
                self.estimator.addVisionMeasurement(
                    visionPose.toPose2d(), visionTime,
                    (stddevupdate, stddevupdate, math.pi/2)
                )
