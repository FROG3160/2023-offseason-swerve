import math
from wpimath.geometry import Translation2d, Translation3d, Transform3d, Rotation3d, Pose2d, Rotation2d
from wpimath.units import feetToMeters, metersToInches, degreesToRadians, inchesToMeters
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from phoenix6 import (TalonFX, CANcoder, TalonFXConfiguration,
                      CANcoderConfiguration,
                      PositionDutyCycle, VelocityDutyCycle)
from phoenix6.configs.cancoder_configs import (AbsoluteSensorRangeValue, SensorDirectionValue, CANcoderConfiguration,
                                               AbsoluteSensorRange)
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue
from phoenix6.configs.config_groups import MagnetSensorConfigs

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


class FROGFXMotorConfig(TalonFXConfiguration):
    def __init__(self, feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
                 feedback_remote_sensor_id=None, k_p=0, k_i=0, k_d=0, k_v=0):
        super.__init__()
        self.feedback.feedback_sensor_source = feedback_sensor_source
        self.feedback.feedback_remote_sensor_id = feedback_remote_sensor_id
        self.slot0.k_p = k_p
        self.slot0.k_i = k_i
        self.slot0.k_d = k_d
        self.slot0.k_v = k_v
        #TODO Research and test setting allowableClosedloopError for phoenix6 Library #33
        # self.allowableClosedloopError

class FROGFXMotor(TalonFX):
    def __init__(self, id=None, motor_config=FROGFXMotorConfig()):
        super.__init__(device_id=id)
        self.config = motor_config
        self.configurator.apply(self.config)
        
#
# **Swerve Module Drive Motor Config
#
flDriveMotorConfig = FROGFXMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
    k_v = 0.0455
)
frDriveMotorConfig = FROGFXMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
    k_v = 0.04425
)
blDriveMotorConfig = FROGFXMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
    k_v = 0.0439
)
brDriveMotorConfig = FROGFXMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
    k_v = 0.0438
)

#
# **Swerve Module Steer Motor Config
#
frSteerMotorConfig = FROGFXMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
    feedback_remote_sensor_id=31,
    k_p=1.2,
    k_i=0.0001
)
flSteerMotorConfig = FROGFXMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
    feedback_remote_sensor_id=32,
    k_p=1.2,
    k_i=0.0001
)
blSteerMotorConfig = FROGFXMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
    feedback_remote_sensor_id=33,
    k_p=1.2,
    k_i=0.0001
)
brSteerMotorConfig = FROGFXMotorConfig(
    feedback_sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
    feedback_remote_sensor_id=34,
    k_p=1.2,
    k_i=0.0001
)

# **Swerve Module Drive Motors
flDriveMotor = FROGFXMotor(
    id=21,
    motor_config=flDriveMotorConfig
)
frDriveMotor = FROGFXMotor(
    id=22,
    motor_config=frDriveMotorConfig
)
blDriveMotor = FROGFXMotor(
    id=23,
    motor_config=frDriveMotorConfig
)
brDriveMotor = FROGFXMotor(
    id=24,
    motor_config=frDriveMotorConfig
)

# **Swerve Module Steer Motors



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

MODULE_FRONT_RIGHT = SwerveModuleConfig(
    "FrontRight",
    12,
    22,
    32,
    -150.293,
    WHEELBASE / 2,
    -TRACK_WIDTH / 2,
    0,
    0,
    0,
    0.04425
)

MODULE_BACK_LEFT = SwerveModuleConfig(
    "BackLeft",
    13,
    23,
    33,
    179.825,
    -WHEELBASE / 2, 
    TRACK_WIDTH / 2,
    0,
    0,
    0,
    0.0439
)
    
MODULE_BACK_RIGHT = SwerveModuleConfig(
    "BackRight",
    14,
    24,
    34,
    46.230,
    -WHEELBASE / 2,
    -TRACK_WIDTH / 2,
    0,
    0,
    0,
    0.0438
)


MAX_TRAJECTORY_SPEED = 3
MAX_TRAJECTORY_ACCEL = 3
ppXPIDController = PIDController(1,0,0)
ppYPIDController = PIDController(1,0,0)
ppRotationPIDController = PIDController(1,0,0)
ppTolerance = Pose2d(0.03, 0.03, Rotation2d.fromDegrees(2))


#
# **Swerve Module CanCoder Config
#
cfgSteerEncoder = CANcoderConfiguration().with_magnet_sensor(MagnetSensorConfigs().with_magnet_offset())
cfgSteerEncoder.magnet_sensor.with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
cfgSteerEncoder.magnet_sensor.with_absolute_sensor_range(AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF)

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