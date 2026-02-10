# this code was created by ChatGPT, an AI language model developed by OpenAI, under the instruction of Dr. Gary Nielsen


import wpilib
from RobotContainer import RobotContainer


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.container = RobotContainer()

    def teleopPeriodic(self):
        # Run the scheduler to execute commands
        wpilib.CommandScheduler.getInstance().run()

