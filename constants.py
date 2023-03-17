from wpimath.kinematics import DifferentialDriveKinematics

import math

# physical values of Romi

kTrackWidthMeter = 0.142072613  # in meters
kTrackWidthInch = kTrackWidthMeter * 39.3701
kCountsPerRevolution = 1440.0
kWheelDiameterInch = 2.75591
kWheelDiameterMeter = kWheelDiameterInch / 39.3701

# Robot characterization values for a feedforward controller

ksVolts = 0.8
kvVoltSecondsPerMeter = 6.33
kaVoltSecondsSquaredPerMeter = 0.0389

# Kinematics values
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeter)

# Autonomous Constants
kMaxSpeedMetersPerSecond = 0.8
kMaxAccelerationMetersPerSecondSquared = 0.8
kPDriveVel = 0.085

# Values for the Ramsete Controller as found on the wpilib
# page. https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html

kRamseteB: float = 2.0
kRamseteZeta: float = 0.7
