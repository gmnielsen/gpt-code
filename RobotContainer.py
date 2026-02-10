import wpilib
from wpilib import XboxController
from wpilib.interfaces import GenericHID
from commands2 import RunCommand
from DriveSubsystem import DriveSubsystem


class RobotContainer:
    def __init__(self):
        # Controller
        self.controller = XboxController(0)

        # Subsystems
        self.drive = DriveSubsystem()

        # Default drive command
        self.drive.setDefaultCommand(
            RunCommand(
                lambda: self.drive.arcade_drive(
                    forward=self.get_trigger_speed(),
                    rotation=self.get_turn_value()
                ),
                self.drive
            )
        )

    # ---- Input Processing ----
    def get_trigger_speed(self):
        # Right trigger forward (0 to 1), left trigger reverse (0 to 1)
        rt = self.controller.getRightTriggerAxis()
        lt = self.controller.getLeftTriggerAxis()
        return rt - lt

    def get_turn_value(self):
        return self.controller.getRightX()
