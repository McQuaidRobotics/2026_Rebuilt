package igknighters.subsystems.swerve.swerveconstants;

import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import wayfinder.setpointGenerator.SwerveSetpointGenerator;

public abstract class CommonSwerveConsts {
    public abstract CommandSwerveDrivetrain createDrivetrain(Subsystem requirement);

    public abstract double getMaxSpeedMetersPerSecond();

    public abstract SwerveSetpointGenerator createSetpointGenerator();
}
