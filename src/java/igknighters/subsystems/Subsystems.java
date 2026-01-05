package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.LimeLightVision.LimeLights;
import igknighters.subsystems.elevator.Elevator;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public class Subsystems {
    public final CommandSwerveDrivetrain swerve;
    public final LimeLights vision;
    public final Led led;
    public final Elevator elevator;
    public final ExclusiveSubsystem[] lockedResources;
    public final SharedSubsystem[] locklessResources;

    public Subsystems(
            CommandSwerveDrivetrain drivetrain, LimeLights vision, Led led, Elevator elevator) {
        this.swerve = drivetrain;
        this.vision = vision;
        this.led = led;
        this.elevator = elevator;
        this.lockedResources = new ExclusiveSubsystem[] {this.swerve, led, elevator};
        this.locklessResources = new SharedSubsystem[] {vision};

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
