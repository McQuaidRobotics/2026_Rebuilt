package igknighters.subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.commands.IndexerCommands;
import igknighters.commands.IntakeCommands;
import igknighters.commands.Shooter.AimingCommands;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.Luma.Luma;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.indexer.launcherRollers.ExitRollersBase;
import igknighters.subsystems.indexer.spindexer.SpindexerBase;
import igknighters.subsystems.intake.AbstractIntake;
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
    public final ExitRollersBase exitRollers;
    public final SpindexerBase spindexer;
    public final AbstractIntake intake;
    public final Luma luma;
    public final SubsystemBase[] lockedResources;

    public Subsystems(
            Swerve swerve,
            LimeLightVision vision,
            Led led,
            Shooter shooter,
            Indexer indexer,
            Intake intake,
            Luma luma) {
        this.swerve = swerve;
        this.vision = vision;
        this.led = led;
        this.shooter = shooter;
        this.luma = luma;
        this.intake = intake;
        this.indexer = indexer;
        this.spindexer = indexer.spindexer;
        this.exitRollers = indexer.exitRollers;
        this.lockedResources =
                new SubsystemBase[] {
                    swerve, shooter, spindexer, exitRollers, intake, luma, vision, led
                };

        this.spindexer.setDefaultCommand(IndexerCommands.jorkIt(indexer).repeatedly());
        this.exitRollers.setDefaultCommand(this.exitRollers.setVelocity(RPM.of(0.0)));
        this.intake.setDefaultCommand(IntakeCommands.holdAtStow(intake));
        this.shooter.setDefaultCommand(AimingCommands.idleCommand(shooter));
    }
}
