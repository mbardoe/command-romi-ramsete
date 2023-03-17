from wpimath.kinematics import DifferentialDriveKinematics

import math

# physical values of Romi

kTrackWidthMeter = .141  # in meters
kTrackWidthInch = .141 * 39.3701
kCountsPerRevolution = 1440.0
kWheelDiameterInch = 2.75591
kWheelDiameterMeter = kWheelDiameterInch / 39.3701

# Robot characterization values for a feedforward controller

ksVolts = 2.744863
kvVoltSecondsPerMeter = 2.028428 / (math.pi * kWheelDiameterMeter)
kaVoltSecondsSquaredPerMeter = 0.015473 / (math.pi * kWheelDiameterMeter)

# Kinematics values
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeter)

# Autonomous Constants
kMaxSpeedMetersPerSecond = 2.5 * (math.pi * kWheelDiameterMeter)
kMaxAccelerationMetersPerSecondSquared = 5 * (math.pi * kWheelDiameterMeter)
kPDriveVel = 0.1  # this one I just guessed on...

# Values for the Ramsete Controller as found on the wpilib
# page. https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html

kRamseteB: float = 2.0
kRamseteZeta: float = 0.7
