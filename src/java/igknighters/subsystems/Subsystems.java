package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.Luma.Luma;
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
    public final Luma luma;
    public final Climber climber;
    public final SubsystemBase[] lockedResources;

    public Subsystems(
            Swerve swerve,
            LimeLightVision vision,
            Led led,
            Shooter shooter,
            Indexer indexer,
            Intake intake,
            Climber climber,
            Luma luma) {
        this.swerve = swerve;
        this.vision = vision;
        this.led = led;
        this.shooter = shooter;
        this.luma = luma;
        this.intake = intake;
        this.climber = climber;
        this.indexer = indexer;
        this.lockedResources = new SubsystemBase[] {luma, swerve, led, shooter, vision, indexer, intake};

        CommandScheduler.getInstance().registerSubsystem(this.lockedResources);
        // this.shooter.setDefaultCommand(ShooterCommands.idle(shooter));
        // for (SharedSubsystem subsystem : this.locklessResources) {
        //     CommandScheduler.getInstance()
        //             .registerSubsystem(
        //                     new Subsystem() {
        //                         @Override
        //                         public void periodic() {
        //                             subsystem.periodic();
        //                         }

        //                         @Override
        //                         public String getName() {
        //                             return subsystem.getName();
        //                         }
        //                     });
        // }
    }

    // public static interface SharedSubsystem {
    //     default void periodic() {}

    //     default void simulationPeriodic() {}

    //     default String getName() {
    //         return this.getClass().getSimpleName();
    //     }
    // }
}
