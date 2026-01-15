package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.commands.ShooterCommands;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public class Subsystems {
    public final CommandSwerveDrivetrain swerve;
    public final LimeLightVision vision;
    public final Led led;
    public final Shooter shooter;
    public final ExclusiveSubsystem[] lockedResources;
    public final SharedSubsystem[] locklessResources;

    public Subsystems(
            CommandSwerveDrivetrain drivetrain, LimeLightVision vision, Led led, Shooter shooter) {
        this.swerve = drivetrain;
        this.vision = vision;
        this.led = led;
        this.shooter = shooter;
        this.lockedResources = new ExclusiveSubsystem[] {this.swerve, led, shooter};
        this.locklessResources = new SharedSubsystem[] {vision};

        CommandScheduler.getInstance().registerSubsystem(this.lockedResources);
        this.shooter.setDefaultCommand(ShooterCommands.idle(shooter));
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
