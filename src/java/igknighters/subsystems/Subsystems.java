package igknighters.subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.commands.IntakeCommands;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.Luma.Luma;
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
        this.lockedResources =
                new SubsystemBase[] {
                    swerve,
                    shooter,
                    climber,
                    indexer.exitRollers,
                    indexer.spindexer,
                    intake,
                    luma,
                    vision,
                    led
                };

        this.indexer.spindexer.setDefaultCommand(indexer.spindexer.jorkRepeating());
        this.indexer.exitRollers.setDefaultCommand(indexer.exitRollers.holdSpeed(RPM.of(0.0)));
        this.intake.setDefaultCommand(IntakeCommands.goToStow(intake));
    }
}
