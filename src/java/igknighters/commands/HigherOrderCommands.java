package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.FieldConstants;
import igknighters.constants.ShootInformation;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.climber.ClimberState;
import java.util.Set;

public class HigherOrderCommands {
    public static Command shootTillEmpty(Subsystems subsystems, double timeout) {
        return rapidFireStream(subsystems)
                .withTimeout(timeout); // this is a placeholder for IndexerCommands.isBallPresent()
    }

    public static Command shootNoStop(Subsystems subsystems) {
        return Commands.parallel(
                        ShooterCommands.shoot(
                                        subsystems.shooter,
                                        () -> subsystems.swerve.getState().Pose,
                                        subsystems.swerve::getFieldRelativeSpeeds)
                                .repeatedly()
                                .withName("SHOOTING WHILE DOING OTHER STUFF"),
                        IndexerCommands.dispense(subsystems.indexer)
                                .onlyIf(ShootInformation.getInstance().atCommandedStateTrigger()))
                .withName("DISPENSING")
                .alongWith(Commands.print("DISPENSING"))
                .withName("SHOOT NO STOP");
    }

    public static Command rapidFireStream(Subsystems subsystems) {

        // 1. The Active Shooter (Tracks and spools continuously)
        Command shooterCommand =
                ShooterCommands.shoot(
                                subsystems.shooter,
                                () -> subsystems.swerve.getState().Pose,
                                subsystems.swerve::getFieldRelativeSpeeds)
                        .withName("Active Spool & Aim");

        // 2. The Smart Hopper/Indexer Feed
        Command smartFeed =
                Commands.either(
                        IndexerCommands.dispense(subsystems.indexer),
                        IndexerCommands.justStop(subsystems.indexer),
                        ShootInformation.getInstance()
                                .atCommandedStateTrigger()
                                .and(ShootInformation.getInstance().beingControlledTrigger())
                                .and(ShootInformation.getInstance().shotPosible()));

        return Commands.parallel(shooterCommand, smartFeed.repeatedly()).withName("SMART STREAM");
    }

    public static Command fireAtTarget(Subsystems subsystems, Pose3d targetPose) {
        Command shooterCommand =
                ShooterCommands.SHOOT_MAX_MIN_NO_AUTO_PICKED_TARGET(
                                subsystems.shooter,
                                targetPose,
                                () -> subsystems.swerve.getState().Pose,
                                subsystems.swerve::getFieldRelativeSpeeds,
                                5,
                                3)
                        .withName("Active Spool & Aim");

        // 2. The Smart Hopper/Indexer Feed
        Command smartFeed =
                Commands.either(
                        IndexerCommands.dispense(subsystems.indexer),
                        IndexerCommands.justStop(subsystems.indexer),
                        ShootInformation.getInstance()
                                .atCommandedStateTrigger()
                                .and(ShootInformation.getInstance().beingControlledTrigger())
                                .and(ShootInformation.getInstance().shotPosible()));

        return Commands.runOnce(() -> DrivingSharedState.getInstance().setDetune(.5))
                .andThen(
                        Commands.parallel(shooterCommand, smartFeed.repeatedly())
                                .withName("SMART STREAM"));
    }

    public static Command IdleShooter(Subsystems subsystems) {
        return ShooterCommands.idleCommand(
                        subsystems.shooter,
                        () -> subsystems.swerve.getState().Pose,
                        subsystems.swerve::getFieldRelativeSpeeds)
                .alongWith(IndexerCommands.jorkIt(subsystems.indexer).repeatedly())
                .alongWith(Commands.runOnce(() -> DrivingSharedState.getInstance().setDetune(1.0)))
                .withName("IDLING THE SHOOTER");
    }

    public static Command forceDispense(Subsystems subsystems) {
        // 1. The Active Shooter (Tracks and spools continuously)
        Command shooterCommand =
                ShooterCommands.shoot(
                                subsystems.shooter,
                                () -> subsystems.swerve.getState().Pose,
                                subsystems.swerve::getFieldRelativeSpeeds)
                        .withName("Active Spool & Aim");

        return Commands.parallel(
                        shooterCommand, IndexerCommands.dispense(subsystems.indexer).repeatedly())
                .withName("SMART STREAM");
    }

    public static Pose2d getClimbStartPose() {
        if (Robot.isBlue()) {
            return new Pose2d(
                    FieldConstants.CLIMB.POSITION_BLUE.getX() + 2.0,
                    FieldConstants.CLIMB.POSITION_BLUE.getY(),
                    FieldConstants.CLIMB.POSITION_BLUE.getRotation());
        } else {
            return new Pose2d(
                    FieldConstants.CLIMB.POSITION_RED.getX() - 2.0,
                    FieldConstants.CLIMB.POSITION_RED.getY(),
                    FieldConstants.CLIMB.POSITION_RED.getRotation());
        }
    }

    public static Pose2d getClimbEndPose() {
        if (Robot.isBlue()) {
            return FieldConstants.CLIMB.POSITION_BLUE;
        } else {
            return FieldConstants.CLIMB.POSITION_RED;
        }
    }

    public static Command hippoShoot(Subsystems subsystems) {
        return Commands.parallel(
                shootNoStop(subsystems),
                Commands.print("IM HIPPPOING TILL I HIPPO").repeatedly(),
                IntakeCommands.goToIntake(subsystems.intake));
    }

    public static Command unClimbCommand(Subsystems subsystems) {
        return Commands.sequence(
                ClimberCommands.goToState(subsystems.climber, ClimberState.LATCH_ON),
                Repulsor.moveWithRepulsor(subsystems.swerve, getClimbStartPose()),
                ClimberCommands.goToState(subsystems.climber, ClimberState.STOW));
    }

    public static Command prepToClimbFirstRung(Subsystems subsystems) {
        return Commands.defer(
                        () -> {
                            Pose2d startPose = getClimbStartPose();
                            Pose2d endPose = getClimbEndPose();
                            return Commands.parallel(
                                    IntakeCommands.goToStow(subsystems.intake),
                                    Commands.sequence(
                                            Commands.parallel(
                                                            ClimberCommands.holdAtState(
                                                                    subsystems.climber,
                                                                    ClimberState.LATCH_ON),
                                                            Repulsor.moveWithRepulsor(
                                                                    subsystems.swerve, startPose))
                                                    .until(
                                                            SwerveCommands.isAt(
                                                                    subsystems.swerve,
                                                                    startPose,
                                                                    3,
                                                                    2)),
                                            Commands.parallel(
                                                            ClimberCommands.holdAtState(
                                                                    subsystems.climber,
                                                                    ClimberState.LATCH_ON),
                                                            SwerveCommands.moveToSimple(
                                                                    subsystems.swerve, startPose))
                                                    .until(
                                                            SwerveCommands.isAt(
                                                                    subsystems.swerve,
                                                                    startPose,
                                                                    .1,
                                                                    .1)),
                                            Commands.parallel(
                                                            ClimberCommands.holdAtState(
                                                                    subsystems.climber,
                                                                    ClimberState.LATCH_ON),
                                                            SwerveCommands.moveToSimple(
                                                                    subsystems.swerve, endPose))
                                                    .until(
                                                            () ->
                                                                    SwerveCommands.isAt(
                                                                                            subsystems
                                                                                                    .swerve,
                                                                                            endPose,
                                                                                            .1,
                                                                                            .1)
                                                                                    .getAsBoolean()
                                                                            || SwerveCommands
                                                                                    .isAtVelocityAndNotAtStart(
                                                                                            subsystems
                                                                                                    .swerve,
                                                                                            new ChassisSpeeds(
                                                                                                    0,
                                                                                                    0,
                                                                                                    0),
                                                                                            new ChassisSpeeds(
                                                                                                    0.1,
                                                                                                    0.1,
                                                                                                    0.1),
                                                                                            endPose,
                                                                                            .2,
                                                                                            .1)
                                                                                    .getAsBoolean()),
                                            SwerveCommands.stopDriving(subsystems.swerve)));
                        },
                        Set.of(subsystems.swerve, subsystems.climber, subsystems.intake))
                .withName("Moving to Climber and raising to max height");
    }
}
