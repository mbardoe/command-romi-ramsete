from commands2 import RamseteCommand
from wpimath.controller import RamseteController, PIDController, SimpleMotorFeedforwardMeters

from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from subsystems.drivetrain import Drivetrain

import constants


class AutonomousTrajectory(RamseteCommand):

    def __init__(self, drive: Drivetrain):
        # Create a voltage constraint to ensure we don't accelerate too fast.
        self.drivetrain = drive
        autoVoltageConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            constants.kDriveKinematics,
            maxVoltage=10,  # 10 volts max.
        )

        # Below will generate the trajectory using a set of programmed configurations

        # Create a configuration for the trajectory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(
            constants.kMaxSpeedMetersPerSecond,
            constants.kMaxAccelerationMetersPerSecondSquared,
        )

        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.kDriveKinematics)

        # Apply the previously defined voltage constraint.
        config.addConstraint(autoVoltageConstraint)

        # Start at the origin facing the +x direction.
        initialPosition = Pose2d(0, 0, Rotation2d(0))

        # Here are the movements we also want to make during this command.
        # These movements should make an "S" like curve.
        movements = [Translation2d(1, 1), Translation2d(2, -1)]

        # End at this position, three meters straight ahead of us, facing forward.
        finalPosition = Pose2d(3, 0, Rotation2d(0))

        # An example trajectory to follow. All of these units are in meters.
        self.exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            initialPosition,
            movements,
            finalPosition,
            config,
        )

        # Below creates the RAMSETE command
        super().__init__(
            # The trajectory to follow.
            self.exampleTrajectory,
            # A reference to a method that will return our position.
            self.drivetrain.getPose,
            # Our RAMSETE controller.
            RamseteController(constants.kRamseteB, constants.kRamseteZeta),
            # A feedforward object for the robot.
            SimpleMotorFeedforwardMeters(
                constants.ksVolts,
                constants.kvVoltSecondsPerMeter,
                constants.kaVoltSecondsSquaredPerMeter,
            ),
            # Our drive kinematics.
            constants.kDriveKinematics,
            # A reference to a method which will return a DifferentialDriveWheelSpeeds object.
            self.drivetrain.getWheelSpeeds,
            # The turn controller for the left side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # The turn controller for the right side of the drivetrain.
            PIDController(constants.kPDriveVel, 0, 0),
            # A reference to a method which will set a specified
            # voltage to each motor. The command will pass the two parameters.
            self.drivetrain.tankDriveVolts,
            # The subsystems the command should require.
            [self.drivetrain],
        )

        # Reset the robot's position to the starting position of the trajectory.
        self.drivetrain.resetOdometry(self.exampleTrajectory.initialPose())

        # Return the command to schedule. The "andThen()" will halt the robot after
        # the command finishes.
        # return self.andThen(lambda: self.robotDrive.tankDriveVolts(0, 0))

    def end(self, interrupted: bool):
        print("Reached End")
        self.andThen(lambda: self.robotDrive.tankDriveVolts(0, 0))
