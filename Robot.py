import wpilib
from RobotContainer import RobotContainer


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.container = RobotContainer()

    def teleopPeriodic(self):
        # Run the scheduler to execute commands
        wpilib.CommandScheduler.getInstance().run()

