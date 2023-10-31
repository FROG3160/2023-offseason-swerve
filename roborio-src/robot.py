import wpilib
import config
from magicbot import MagicRobot, tunable, feedback
from components.drivetrain import SwerveChassis, SwerveModule
from components.controllers import FROGXboxDriver, FROGXboxOperator
from components.field import FROGFieldLayout
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
from wpimath import applyDeadband
from wpimath.units import feetToMeters
from pathplannerlib import PathPoint
from wpimath.units import inchesToMeters
from components.sensors import FROGColor, FROGGyro
from wpimath.units import degreesToRadians
from ctre import ControlMode
from wpilib.interfaces import GenericHID
from components.drive_control import DriveControl

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue

class FROGbot(MagicRobot):
    swerveChassis: SwerveChassis
    driveControl: DriveControl

    gyro: FROGGyro

    def createObjects(self) -> None:
        self.moduleFrontLeft = SwerveModule(**config.MODULE_FRONT_LEFT)
        self.moduleFrontRight = SwerveModule(**config.MODULE_FRONT_RIGHT)
        self.moduleBackLeft = SwerveModule(**config.MODULE_BACK_LEFT)
        self.moduleBackRight = SwerveModule(**config.MODULE_BACK_RIGHT)

        self.driverController = FROGXboxDriver(0)
        self.operatorController = FROGXboxOperator(1)

        self.fieldLayout = FROGFieldLayout()

        self.btnLockChassis = self.driverController.getRightBumper
        self.btnUnlockChassis = self.driverController.getLeftBumper
        self.btnResetEstimator = self.driverController.getBackButtonPressed
        self.btnResetGyro = self.driverController.getStartButtonPressed
        self.btnDrivePath = self.driverController.getBButton

        self.chassisLocked = False

    def setAlliance(self):
        self.alliance = wpilib.DriverStation.getAlliance()
        self.fieldLayout.setAlliance(self.alliance)
        self.logger.info(f"FROGBot.fieldLayout alliance is {self.fieldLayout.alliance}")
        self.logger.info(f"SwerveChassis.fieldLayout alliance is {self.swerveChassis.fieldLayout.alliance}")
        self.logger.info(f"FROGLimeLight.fieldlayout alliance is {self.swerveChassis.limelight.fieldLayout.alliance}")
    
    def robotInit(self) -> None:
        """Runs at the startup of the robot code.  Anything that needs to be set
        before running Autonomous or Teleop modes should be added here"""
        super().robotInit()  #calls createObjects()
        self.setAlliance()
        #TODO: test if we can place the robot position setting here from AutonomousInit
  
    def autonomousInit(self):
        """Runs at the beginning autonomous mode.  Add anything that is needed
        to put the robot in a known state to start Autonomous mode.
        """
        self.setAlliance()
        self.swerveChassis.enable()
        # TODO: Test and change this to use the initial bot post from vision?
        # this call gets the pose3d from the tuple returned by getBotPoseEstimateForAlliance

    def teleopInit(self):
        self.setAlliance()
        self.swerveChassis.enable()

    def teleopPeriodic(self):
        if self.btnResetEstimator():
            print("Resetting Estimator to Vision Pose Estimate")
            visionPose, timestamp = self.limelight.getBotPoseEstimateForAlliance()
            if visionPose:
                self.swerveChassis.setFieldPosition(visionPose.toPose2d())
        if self.btnResetGyro():
            self.swerveChassis.gyro.resetGyro()

        if self.btnLockChassis():
            self.chassisLocked = True
            self.swerveChassis.enabled = False

        if self.btnUnlockChassis():
            self.chassisLocked = False
            self.swerveChassis.enabled = True

        if self.chassisLocked:
            self.driveControl.lockWheels()

        elif self.btnGoToGridPosition():
            self.logger.info(f'Driving to position {self.gridPosition}')
            self.driveControl.holonomicDriveToWaypoint(
                self.fieldLayout.getPosition(self.gridPosition).toPose2d()
            )

        elif self.btnDrivePath():
            self.driveControl.holonomicDrivePath()

        else:
            pass

if __name__ == "__main__":
    wpilib.run(FROGbot)