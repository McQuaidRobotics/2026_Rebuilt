package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.swerve.Swerve;

public class Subsystems {
    public final Swerve swerve;
    public final LimeLightVision vision;
    public final Led led;
    public final Shooter shooter;
    public final Indexer indexer;
    public final Intake intake;
    public final Climber climber;
    public final SubsystemBase[] lockedResources;

    public Subsystems(
            Swerve swerve,
            LimeLightVision vision,
            Led led,
            Shooter shooter,
            Indexer indexer,
            Intake intake,
            Climber climber) {
        this.swerve = swerve;
        this.vision = vision;
        this.led = led;
        this.shooter = shooter;
        this.intake = intake;
        this.climber = climber;
        this.indexer = indexer;
        this.lockedResources =
                new SubsystemBase[] {swerve, led, shooter, vision, indexer, intake, climber};

        // SubsystemBase objects register themselves automatically in their constructor.
    }

    // public static interface SharedSubsystem {
    //     default void periodic() {}

    //     default void simulationPeriodic() {}

    //     default String getName() {
    //         return this.getClass().getSimpleName();
    //     }
    // }
}
