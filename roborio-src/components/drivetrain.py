import math
from logging import Logger

import config
from components.controllers import PPHolonomic
from components.field import FROGFieldLayout
from components.sensors import FROGGyro
from components.vision import FROGLimeLightVision
from ctre import (AbsoluteSensorRange, ControlMode, FeedbackDevice,
                  NeutralMode, RemoteSensorSource,
                  SensorInitializationStrategy, StatusFrameEnhanced,
                  TalonFXInvertType, WPI_CANCoder, WPI_TalonFX, TalonFXConfiguration)
from magicbot import feedback
from utils.utils import DriveUnit, remap
from wpilib import Field2d, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import (ChassisSpeeds, SwerveDrive4Kinematics,
                                SwerveModulePosition, SwerveModuleState)
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.units import feetToMeters, metersToInches

from .sensors import FROGGyro

# Motor Control modes
VELOCITY_MODE = ControlMode.Velocity
POSITION_MODE = ControlMode.Position

MAX_TRAJECTORY_SPEED = feetToMeters(5)
MAX_TRAJECTORY_ACCEL = feetToMeters(5)


# TODO: move this function to utils.  We can use it for other systems.
def constrain_radians(rads):
    """Returns radians between -2*pi and 2*pi
    Args:
        rads (float): angle in radians"""
    return math.atan2(math.sin(rads), math.cos(rads))


class FROGSwerveModuleState(SwerveModuleState):
    """Extends SwerveModuleState with method to optimize
    the steering angle

    """
    def optimize(self, current_steer_position):
        # def optimize_steer_angle(new_state: SwerveModuleState, current_radians):
        """This function takes the desired module state and the current
        angle of the wheel and calculates a new position that keeps the
        amount of rotation needed to under 90 degrees in either direction.
        Args:
            new_state (SwerveModuleState): the module state
            current_steer_position (float): current angle in radians.
                This value does not have to be between -pi and pi.
        Returns:
            SwerveModuleState

        """
        invert_speed = 1

        # all angles are in radians
        desired_angle = self.angle.radians()

        # get current angle by getting the steer position in encoder ticks
        # and dividing it by the ticks per radian of the encoder
        current_angle = constrain_radians(current_steer_position / config.CANCODER_TICKS_PER_RADIAN)

        n_offset = desired_angle - current_angle

        # if our offset is greater than 90 degrees (pi/2 radians), we need to 
        # flip 180 and reverse speed.
        # TODO:  Add code to check/monitor how many times we go through this while
        #   loop.  This may be a cause for teleop loop overruns.  The while loop
        #   was originally implemented because sometimes the first calculation
        #   would still give us an angle larger than 90 degrees.  We may want to
        #   find a way to determine how much of a change needs to be made before we
        #   run the calculations in the while loop.
        while abs(n_offset) > math.pi / 2:
            if n_offset < -math.pi / 2:
                n_offset += math.pi
                invert_speed *= -1
            elif n_offset > math.pi / 2:
                n_offset -= math.pi
                invert_speed *= -1
        new_angle = constrain_radians(current_angle + n_offset)

        if abs(n_offset) > math.pi / 2:
            print(">>>>>>>>>>>>>>>>>ERROR<<<<<<<<<<<<<<<<<<<<")
        
        self.invert_speed = invert_speed
        self.position_offset = n_offset
        # sets the new angle of the Swerve Module State
        self.angle = Rotation2d(new_angle)


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
    def __init__(self, gear_stages: list, motor_rpm: int, diameter: float, cpr: int):
        """Constructs a DriveUnit object that stores data about the motor, gear stages, and wheel
           that makes up a complete power train.
        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
            motor_rpm (int): Maximum rpm of the attached motor
            diameter (float): Diameter of the attached wheel in meters
            cpr (int): Number of encoder counts per revolution
        """
        self.gearing = GearStages(gear_stages)
        self.motor_rpm = motor_rpm
        self.cpr = cpr
        self.circumference = math.pi * diameter

    def speedToVelocity(self, speed: float) -> float:
        """Converts the system linear speed to a motor velocity
        Args:
            speed (float): desired linear speed in meters per second
        Returns:
            float: motor velocity in encoder counts per 100ms
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = self.gearing.toMotor(wheel_rotations_sec)
        ticks_per_sec = motor_rotations_sec * self.cpr
        return ticks_per_sec / 10

    def velocityToSpeed(self, velocity: float) -> float:
        """Converts motor velocity to the system linear speed
        Args:
            velocity (float): motor velocity in encoder counts per 100ms
        Returns:
            float: system linear speed in meters per second
        """
        ticks_per_sec = velocity * 10
        motor_rotations_sec = ticks_per_sec / self.cpr
        wheel_rotations_sec = self.gearing.fromMotor(motor_rotations_sec)
        return wheel_rotations_sec * self.circumference

    def positionToDistance(self, position: int) -> float:
        """Takes encoder count and returns distance

        Args:
            position (int): number of encoder counts

        Returns:
            float: distance in meters
        """
        motor_rotations = position / self.cpr
        wheel_rotations = self.gearing.fromMotor(motor_rotations)
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
        super().__init__()
        # set initial states for the component
        self.name = name
        self.drive = WPI_TalonFX(drive_motor_id)
        self.steer = WPI_TalonFX(steer_motor_id)
        self.encoder = WPI_CANCoder(steer_sensor_id)
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


    # def getCommandedVelocity(self):
    #     return self.calculated_velocity

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

    # def getDrivePosition(self) -> float:
    #     return self.drive.getSelectedSensorPosition()

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

    def getSteerPosition(self):
        return self.steer.getSelectedSensorPosition(0)

    # def resetRemoteEncoder(self):
    #     self.encoder.setPositionToAbsolute()

    def configModuleComponents(self):
        # configure CANCoder
        # No worky: self.encoder.configAllSettings(cfgSteerEncoder)
        # TODO: ^^ see if we can use the configAllSettings method again
        # TODO:  Review all other config settings for the devices
        self.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.encoder.configSensorDirection(False)
        # adjust 0 degree point with offset
        self.encoder.configMagnetOffset(self.steerOffset)
        self.encoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        )
        # set position to Absolute

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

       # SmartDashboard.putNumber("Drive kF", config.cfgDriveMotor.slot0.kF)

        # self.current_states = None
        # self.current_speeds = None

    def enableMinSpeed(self):
        self.useMinSpeed = True
    
    def disableMinSpeed(self):
        self.useMinSpeed = False

    def setState(self, state: SwerveModuleState):
        # TODO: Remove the following config change once tuning is done
        self.requestedState = FROGSwerveModuleState(state.speed, state.angle)
        
        if self.enabled:
            #
            # using built-in optimize method instead of our custom one from last year
            current_steer_position = self.getSteerPosition()
            self.requestedState.optimize(current_steer_position)
            # self.periodic()

            self.steer.set(
                POSITION_MODE,
                current_steer_position
                + (self.requestedState.position_offset * config.CANCODER_TICKS_PER_RADIAN),
            )

            # constrain speed within min and max speeds
            if self.useMinSpeed:
                driveSpeed = math.copysign(
                    remap(abs(self.requestedState.speed), 0, config.MAX_METERS_PER_SEC, config.MIN_METERS_PER_SEC, config.MAX_METERS_PER_SEC),
                    self.requestedState.speed
                )
            else:
                driveSpeed = self.requestedState.speed
            self.drive.set(
                VELOCITY_MODE,
                self.drive_unit.speedToVelocity(driveSpeed * self.requestedState.invert_speed),
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
            FROGSwerveModuleState(),
            FROGSwerveModuleState(),
            FROGSwerveModuleState(),
            FROGSwerveModuleState(),
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

        #self.holonomicController = FROGHolonomic(self.kinematics)
        # self.holonomicController = PPHolonomic(self.kinematics)
        # self.holonomicController.logger = self.logger

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
        # self.odometry.resetPosition(
        #     self.gyro.getRotation2d(),
        #     self.starting_pose,
        #     *self.getModulePositions()
        # )


    # def getSimpleTrajectory(self):
    #     self.startTrajectoryPose = self.estimator.getEstimatedPosition()
    #     self.endTrajectoryPose = self.startTrajectoryPose + Transform2d(
    #         feetToMeters(6), feetToMeters(3), 0
    #     )
    #     self.logger.info(
    #         "Auto Drive - Start Pose: %s\n End Pose:%s",
    #         self.startTrajectoryPose,
    #         self.endTrajectoryPose,
    #     )
    #     return TrajectoryGenerator.generateTrajectory(
    #         self.startTrajectoryPose,  # Starting position
    #         [],  # Pass through these points
    #         self.endTrajectoryPose,  # Ending position
    #         self.trajectoryConfig,
    #     )

    # def getSwerveCommand(self):
    #     self.xController = PIDController(1, 0, 0)
    #     self.yController = PIDController(1, 0, 0)
    #     self.angleController = ProfiledPIDControllerRadians(
    #         1, 0, 0, TrapezoidProfileRadians.Constraints(math.pi, math.pi)
    #     )
    #     self.angleController.enableContinuousInput(-1 * math.pi, math.pi)
    #     self.holonomicController = HolonomicDriveController(
    #         self.xController, self.yController, self.angleController
    #     )
    #     return Swerve4ControllerCommand(
    #         self.getSimpleTrajectory(),
    #         self.estimator.getEstimatedPosition,  # CALLABLE getPose
    #         self.kinematics,
    #         self.holonomicController,
    #         self.setModuleStates,
    #         [self],
    #     )

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
        # TODO: Remove this call once we have tuned the drivetrain
        #       It allows us to adjust PID values on the fly.
        #self.holonomicController.loadPID()
        """Sets ChassisSpeeds from return of the holonomic controller."""
        # self.chassisSpeeds = self.holonomicController.getChassisSpeeds(
        #     self.estimator.getEstimatedPosition()
        # )
        self.chassisSpeeds = chassisSpeeds

    def robotOrientedDrive(self, vX, vY, vT):
        self.logger.info(f'Velocities: {vX}, {vY}, {vT}')
        self.chassisSpeeds = ChassisSpeeds(vX, vY, vT)

    # def driveToObject(self):
    #     if self.limelight.hasGrabberTarget():
    #         self.robotOrientedDrive(
    #             self.limelight.drive_vX,
    #             self.limelight.drive_vY,
    #             self.limelight.drive_vRotate
    #         )
    #     else:
    #         self.robotOrientedDrive(0,0,0)

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
            # SmartDashboard.putNumber(
            #     "Vision_X", visionPose.X()
            # )
            # SmartDashboard.putNumber(
            #     "Vision_Y", visionPose.Y()
            # )
            # SmartDashboard.putNumber(
            #     "Vision_Z", visionPose.Z()
            # )
            # SmartDashboard.putNumber(
            #     "Vision_T", visionPose.rotation().toRotation2d().degrees()
            # )
            if (
                abs(visionPose.x - self.estimatorPose.x) < 0.5
                and abs(visionPose.y - self.estimatorPose.y) < 0.5
            ):
                stddevupdate = remap(visionPose.x,2.0, 8.0, 0.3, 2.0)
                # self.logger.info(f'Adding vision measuerment with StdDev of {stddevupdate} and distance of {visionPose.x} ')
                self.estimator.addVisionMeasurement(visionPose.toPose2d(), visionTime, (stddevupdate, stddevupdate, math.pi/2))


        # SmartDashboard.putNumber(
        #     "Estimator_X", self.estimatorPose.X())
        # SmartDashboard.putNumber(
        #     "Estimator_Y", self.estimatorPose.Y())
        # SmartDashboard.putNumber(
        #     "Estimator_T", self.estimatorPose.rotation().degrees()
        # )