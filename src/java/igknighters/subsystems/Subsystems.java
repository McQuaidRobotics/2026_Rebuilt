package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.climber.Climber;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.intake.Intake;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public class Subsystems {
    public final CommandSwerveDrivetrain swerve;
    public final LimeLightVision vision;
    public final Led led;
    public final Shooter shooter;
    public final Indexer indexer;
    public final Intake intake;
    public final Climber climber;
    public final SubsystemBase[] lockedResources;
    public final ExclusiveSubsystem[] notPublished;

    public Subsystems(
            CommandSwerveDrivetrain drivetrain,
            LimeLightVision vision,
            Led led,
            Shooter shooter,
            Indexer indexer,
            Intake intake,
            Climber climber) {
        this.swerve = drivetrain;
        this.vision = vision;
        this.led = led;
        this.shooter = shooter;
        this.intake = intake;
        this.climber = climber;
        this.indexer = indexer;
        this.lockedResources = new SubsystemBase[] {led, shooter, vision, indexer, intake, climber};
        this.notPublished = new ExclusiveSubsystem[] {swerve};

        CommandScheduler.getInstance().registerSubsystem(this.notPublished);
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

    public static interface ExclusiveSubsystem extends Subsystem {}

    // public static interface SharedSubsystem {
    //     default void periodic() {}

    //     default void simulationPeriodic() {}

    //     default String getName() {
    //         return this.getClass().getSimpleName();
    //     }
    // }
}
