package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.commands.IndexerCommands;
import igknighters.commands.Shooter.AimingCommands;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.Luma.Luma;
import igknighters.subsystems.YamsIntake.IntakePivot;
import igknighters.subsystems.YamsIntake.Rollers;
import igknighters.subsystems.YamsIntake.YamIntake;
import igknighters.subsystems.YamsIntake.YamIntakeState;
import igknighters.subsystems.indexer.Indexer;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.swerve.Swerve;

public class Subsystems {
    public final Swerve swerve;
    public final LimeLightVision vision;
    public final Led led;
    public final Shooter shooter;
    public final Indexer indexer;
    public final IntakePivot pivot;
    public final Rollers rollers;
    public final YamIntake intake;
    public final Luma luma;
    public final SubsystemBase[] lockedResources;

    public Subsystems(
            Swerve swerve,
            LimeLightVision vision,
            Led led,
            Shooter shooter,
            Indexer indexer,
            YamIntake intake,
            Luma luma) {
        this.swerve = swerve;
        this.vision = vision;
        this.led = led;
        this.shooter = shooter;
        this.luma = luma;
        this.intake = intake;
        this.indexer = indexer;
        this.pivot = intake.pivot;
        this.rollers = intake.rollers;
        this.lockedResources =
                new SubsystemBase[] {swerve, shooter, indexer, rollers, pivot, luma, vision, led};

        this.indexer.setDefaultCommand(IndexerCommands.jorkIt(indexer).repeatedly());

        this.pivot.setDefaultCommand(pivot.targetAngle(YamIntakeState.STOWED.pivotAngle));
        this.rollers.setDefaultCommand(
                rollers.targetVelocity(YamIntakeState.STOWED.rollerVelocity));

        this.shooter.setDefaultCommand(AimingCommands.idleCommand(shooter));
    }
}
