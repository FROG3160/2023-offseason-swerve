import math
from wpimath.geometry import Translation2d, Translation3d, Transform3d, Rotation3d, Pose2d, Rotation2d
from wpimath.units import feetToMeters, metersToInches, degreesToRadians, inchesToMeters
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from ctre import (
    TalonFXConfiguration,
    SensorInitializationStrategy,
    BaseTalonPIDSetConfiguration,
    FeedbackDevice,
    CANCoderConfiguration,
    AbsoluteSensorRange,
)
from phoenix6 import (TalonFX, CANcoder, TalonFXConfiguration,
                      CANcoderConfiguration,
                      PositionDutyCycle, VelocityDutyCycle)
from phoenix6.configs.cancoder_configs import AbsoluteSensorRangeValue, SensorDirectionValue
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue

TRACK_WIDTH = 20 / 12
WHEELBASE = 20 / 12

# SwerveDriveSpecialties modules have the following max speeds (in ft/sec):
# L1 - 13.5, L2 - 16.3, L3 - 18
MAX_FEET_PER_SEC = 16
MIN_FEET_PER_SEC = 0.55
MAX_METERS_PER_SEC = feetToMeters(MAX_FEET_PER_SEC)
MIN_METERS_PER_SEC = feetToMeters(MIN_FEET_PER_SEC)
VOLTAGE_COMPENSATION = 10.5

MAX_CHASSIS_REV_SEC = 2
MAX_CHASSIS_RADIANS_SEC = MAX_CHASSIS_REV_SEC * math.tau

MODULE_DRIVE_GEARING = [(14.0 / 50.0), (28.0 / 16.0), (15.0 / 45.0)]  # Mk4 L3
MODULE_WHEEL_DIAMETER = 0.1000125  # 3 15/16 inches in meters

class FROGMotorConfig(TalonFXConfiguration):
    def __init__(self, feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
                 feedback_remote_sensor_id=None, k_p=0, k_i=0, k_d=0, k_v=0):
        super.__init__()
        self.feedback.feedback_sensor_source = feedback_sensor_source
        self.feedback.feedback_remote_sensor_id = feedback_remote_sensor_id
        self.slot0.k_p = k_p
        self.slot0.k_i = k_i
        self.slot0.k_d = k_d
        self.slot0.k_v = k_v

class FROGTalonFXMotor(TalonFX):
    def __init__(self, id=None, feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
                 feedback_remote_sensor_id=None, k_p=0, k_i=0, k_d=0, k_v=0):
        super.__init__(device_id=id)
        self.config = TalonFXConfiguration()
        self.config.feedback.feedback_sensor_source = feedback_sensor_source
        self.config.feedback.feedback_remote_sensor_id = feedback_remote_sensor_id
        self.config.slot0.k_p = k_p
        self.config.slot0.k_i = k_i
        self.config.slot0.k_d = k_d
        self.config.slot0.k_v = k_v
        self.configurator.apply(self.config)
        
#
# **Swerve Module Drive Motor Config
#
# frDriveMotorPID = TalonFXConfiguration()
# frDriveMotorPID.initializationStrategy = SensorInitializationStrategy.BootToZero
# frDriveMotorPID.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
# frDriveMotorPID.slot0.kP = 0.0
# frDriveMotorPID.slot0.kI = 0.0
# frDriveMotorPID.slot0.kD = 0.0
# frDriveMotorPID.slot0.kF = 0.04425

frDriveMotorConfig = FROGMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
    k_v = 0.04425

)
frDriveMotor = FROGTalonFXMotor(
    id=21,
    feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
    k_v = 0.04425
)
frDriveMotor.set_control()
# flDriveMotorPID = TalonFXConfiguration()
# flDriveMotorPID.initializationStrategy = SensorInitializationStrategy.BootToZero
# flDriveMotorPID.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
# flDriveMotorPID.slot0.kP = 0.0
# flDriveMotorPID.slot0.kI = 0.0
# flDriveMotorPID.slot0.kD = 0.0
# flDriveMotorPID.slot0.kF = 0.0455

flDriveMotorConfig = FROGMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
    k_v = 0.0455

)

# blDriveMotorPID = TalonFXConfiguration()
# blDriveMotorPID.initializationStrategy = SensorInitializationStrategy.BootToZero
# blDriveMotorPID.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
# blDriveMotorPID.slot0.kP = 0.0
# blDriveMotorPID.slot0.kI = 0.0
# blDriveMotorPID.slot0.kD = 0.0
# blDriveMotorPID.slot0.kF = 0.0439

blDriveMotorConfig = FROGMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
    k_v = 0.0439

)

# brDriveMotorPID = TalonFXConfiguration()
# brDriveMotorPID.initializationStrategy = SensorInitializationStrategy.BootToZero
# brDriveMotorPID.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
# brDriveMotorPID.slot0.kP = 0.0
# brDriveMotorPID.slot0.kI = 0.0
# brDriveMotorPID.slot0.kD = 0.0
# brDriveMotorPID.slot0.kF = 0.0438

brDriveMotorConfig = FROGMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
    k_v = 0.0438

)

class SwerveSteerEncoderConfig(CANcoderConfiguration):
    def __init__(self, steer_offset):
        super.__init__()
        self.magnet_sensor
        self.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        self.magnet_sensor.magnet_offset = steer_offset
        self.magnet_sensor.sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)



class SwerveSteerMotorConfig(TalonFXConfiguration):
    def __init__(self):
        super.__init__()


class SwerveModuleConfig:
    def __init__(self,
                 name: str,
                 drive_motor_id: int,
                 steer_motor_id: int,
                 steer_sensor_id: int,
                 steer_sensor_offset: float,
                 location_x: float,
                 location_y: float,
                 drive_p: float = 0,
                 drive_i: float = 0,
                 drive_d: float = 0,
                 drive_v: float = 0): #used to be kF (feed-forward)
        
        self.name = name
        self.drive_motor_id = drive_motor_id
        self.steer_motor_id = steer_motor_id
        self.steer_sensor_id = steer_sensor_id
        self.steer_sensor_offset = steer_sensor_offset
        self.location = Translation2d.fromFeet(location_x, location_y)
        self.drive_motor_config = TalonFXConfiguration()
        # self.drive_motor_config.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.IntegratedSensor)
        self.drive_motor_config.feedback.feedback_sensor_source(FeedbackSensorSourceValue.ROTOR_SENSOR)
        self.drive_motor_config.slot0.k_p = drive_p
        self.drive_motor_config.slot0.k_i = drive_i
        self.drive_motor_config.slot0.k_d = drive_d
        self.drive_motor_config.slot0.k_v = drive_v

    def __repr__(self):
        return(f'{self.name}:, {self.location}')

MODULE_FRONT_LEFT = SwerveModuleConfig(
    'FrontLeft',
    11,
    21,
    31,
    -5.625,
    WHEELBASE / 2,
    TRACK_WIDTH / 2,
    0,
    0,
    0,
    0.0455
    )

MODULE_FRONT_LEFT = {
    "name": "FrontLeft",
    "drive_motor_id": 11,
    "steer_motor_id": 21,
    "steer_sensor_id": 31,
    "steer_sensor_offset": -5.625,
    "location": Translation2d.fromFeet(WHEELBASE / 2, TRACK_WIDTH / 2),
    .drive_motor_config": flDriveMotorPID
}

MODULE_FRONT_RIGHT = {
    "name": "FrontRight",
    "drive_motor_id": 12,
    "steer_motor_id": 22,
    "steer_sensor_id": 32,
    "steer_sensor_offset": -150.293,
    "location": Translation2d.fromFeet(WHEELBASE / 2, -TRACK_WIDTH / 2),
    .drive_motor_config": frDriveMotorPID
}
MODULE_BACK_LEFT = {
    "name": "BackLeft",
    "drive_motor_id": 13,
    "steer_sensor_id": 33,
    "steer_motor_id": 23,
    "steer_sensor_offset": 179.825,
    "location": Translation2d.fromFeet(-WHEELBASE / 2, TRACK_WIDTH / 2),
    .drive_motor_config": blDriveMotorPID
}
MODULE_BACK_RIGHT = {
    "name": "BackRight",
    "drive_motor_id": 14,
    "steer_motor_id": 24,
    "steer_sensor_id": 34,
    "steer_sensor_offset": 46.230,
    "location": Translation2d.fromFeet(-WHEELBASE / 2, -TRACK_WIDTH / 2),
    .drive_motor_config": brDriveMotorPID
}

MAX_TRAJECTORY_SPEED = 3
MAX_TRAJECTORY_ACCEL = 3
ppXPIDController = PIDController(1,0,0)
ppYPIDController = PIDController(1,0,0)
ppRotationPIDController = PIDController(1,0,0)
ppTolerance = Pose2d(0.03, 0.03, Rotation2d.fromDegrees(2))

#
# **Swerve Module Steer Motor Config
#
cfgSteerMotor = TalonFXConfiguration()
cfgSteerMotor.primaryPID = BaseTalonPIDSetConfiguration(FeedbackDevice.RemoteSensor0)
cfgSteerMotor.slot0.kP = 1.2  # TODO: Confirm PID
cfgSteerMotor.slot0.kI = 0.0001
cfgSteerMotor.slot0.kD = 0.0
cfgSteerMotor.slot0.kF = 0.0
""" TODO: test if we can remove this.  The current value amounts to
  about .44 degrees allowed error. """
cfgSteerMotor.slot0.allowableClosedloopError = 5

#
# **Swerve Module CanCoder Config
#
cfgSteerEncoder = CANCoderConfiguration()
cfgSteerEncoder.sensorDirection = False  # CCW spin of magnet is positive
cfgSteerEncoder.initializationStrategy = (
    SensorInitializationStrategy.BootToAbsolutePosition
)
cfgSteerEncoder.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180

cfgProfiledMaxVelocity = math.pi*8
cfgProfiledMaxAccel = math.pi*4

cfgProfiledP = 0.4
cfgProfiledI = 0.0
cfgProfiledD = 0.0

#
# Sweve Drive Constants
#
FIELD_ORIENTED = 0
ROBOT_ORIENTED = 1

FALCON_TICKS_PER_ROTATION = 2048
FALCON_MAX_RPM = 6380
CANCODER_TICKS_PER_ROTATION = 4096
CANCODER_TICKS_PER_DEGREE = CANCODER_TICKS_PER_ROTATION / 360
CANCODER_TICKS_PER_RADIAN = CANCODER_TICKS_PER_ROTATION / math.tau