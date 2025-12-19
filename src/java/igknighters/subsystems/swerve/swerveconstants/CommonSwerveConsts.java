package igknighters.subsystems.swerve.swerveconstants;

import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public abstract class CommonSwerveConsts {
    public abstract CommandSwerveDrivetrain createDrivetrain();

    public abstract double getMaxSpeedMetersPerSecond();
}
