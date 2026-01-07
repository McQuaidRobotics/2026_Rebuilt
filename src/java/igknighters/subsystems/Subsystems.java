package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.LimeLightVision.LimeLights;
import igknighters.subsystems.Luma.Luma;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public class Subsystems {
    public final CommandSwerveDrivetrain swerve;
    public final LimeLights vision;
    public final Luma luma;
    public final Led led;
    public final ExclusiveSubsystem[] lockedResources;
    public final SharedSubsystem[] locklessResources;

    public Subsystems(CommandSwerveDrivetrain drivetrain, LimeLights vision, Luma luma, Led led) {
        this.swerve = drivetrain;
        this.vision = vision;
        this.luma = luma;
        this.led = led;
        this.lockedResources = new ExclusiveSubsystem[] {this.swerve, led};
        this.locklessResources = new SharedSubsystem[] {vision, luma};

        CommandScheduler.getInstance().registerSubsystem(this.lockedResources);
        for (SharedSubsystem subsystem : this.locklessResources) {
            CommandScheduler.getInstance()
                    .registerSubsystem(
                            new Subsystem() {
                                @Override
                                public void periodic() {
                                    subsystem.periodic();
                                }

                                @Override
                                public String getName() {
                                    return subsystem.getName();
                                }
                            });
        }
    }

    public static interface ExclusiveSubsystem extends Subsystem {}

    public static interface SharedSubsystem {
        default void periodic() {}

        default void simulationPeriodic() {}

        default String getName() {
            return this.getClass().getSimpleName();
        }
    }
}
