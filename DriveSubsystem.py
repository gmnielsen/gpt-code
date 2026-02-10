import math
import wpilib
from rev import CANSparkMax, CANSparkLowLevel
from wpilib import SlewRateLimiter, DifferentialDriveOdometry
import wpilib.drive
from commands2 import SubsystemBase


class DriveSubsystem(SubsystemBase):
    WHEEL_RADIUS_METERS = 0.0762  # 3 inch radius
    GEAR_RATIO = 10.71             # Example gear ratio

    def __init__(self):
        super().__init__()

        # ---- Motors ----
        self.left_front = CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless)
        self.left_rear = CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless)
        self.right_front = CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless)
        self.right_rear = CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless)

        # ---- SparkMAX ramp rate ----
        ramp_time = 0.2
        for m in (self.left_front, self.left_rear, self.right_front, self.right_rear):
            m.setOpenLoopRampRate(ramp_time)

        # ---- Motor groups ----
        self.left_group = wpilib.MotorControllerGroup(self.left_front, self.left_rear)
        self.right_group = wpilib.MotorControllerGroup(self.right_front, self.right_rear)

        self.right_group.setInverted(True)

        # ---- Differential Drive ----
        self.drive = wpilib.drive.DifferentialDrive(self.left_group, self.right_group)

        # ---- Sensors ----
        self.gyro = wpilib.ADXRS450_Gyro()
        self.gyro.reset()

        # Encoders from SparkMAX
        self.left_encoder = self.left_front.getEncoder()
        self.right_encoder = self.right_front.getEncoder()

        # Set position conversion factor (rotations -> meters)
        position_conversion = 2 * math.pi * self.WHEEL_RADIUS_METERS / self.GEAR_RATIO
        self.left_encoder.setPositionConversionFactor(position_conversion)
        self.right_encoder.setPositionConversionFactor(position_conversion)

        # Odometry (uses gyro + wheel positions)
        self.odometry = DifferentialDriveOdometry(self.gyro.getRotation2d())

        # ---- Software Slew-Rate Limiters ----
        self.speed_limiter = SlewRateLimiter(3.0)
        self.turn_limiter = SlewRateLimiter(3.0)

    # ---- Main Drive Method ----
    def arcade_drive(self, forward, rotation):
        smooth_forward = self.speed_limiter.calculate(forward)
        smooth_rotation = self.turn_limiter.calculate(rotation)

        self.drive.arcadeDrive(smooth_forward, smooth_rotation, squaredInputs=True)
        self.update_odometry()

    # ---- ODOMETRY UPDATE ----
    def update_odometry(self):
        self.odometry.update(
            self.gyro.getRotation2d(),
            self.left_encoder.getPosition(),
            self.right_encoder.getPosition()
        )

    # ---- DRIVE STRAIGHT HELPER ----
    def drive_straight(self, speed):
        kP = 0.03
        heading_error = -self.gyro.getAngle()
        correction = kP * heading_error

        self.arcade_drive(speed, correction)