package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.commands.Shooter.AimingCommands;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.Subsystems;
import java.util.function.Supplier;

public class HigherOrderCommands {
    public static Command shootTillEmpty(Subsystems subsystems, double timeout) {
        return Commands.parallel(rapidFireStream(subsystems), subsystems.intake.jorkIntake())
                .withTimeout(timeout)
                .andThen(Commands.print("ALL BALLS SHOT CONTINUING")); // this is a placeholder for
        // IndexerCommands.isBallPresent()
    }

    public static Command rapidFireStream(Subsystems subsystems) {

        // 1. The Active Shooter (Tracks and spools continuously)
        Command shooterCommand =
                AimingCommands.shootWithProtection(subsystems.shooter)
                        .withName("Active Spool & Aim");

        return Commands.parallel(
                        shooterCommand.repeatedly(),
                        IndexerCommands.smartDispense(subsystems.indexer).repeatedly())
                .withName("SMART STREAM");
    }

    public static Command aggregiouslyHighRapidFireStream(Subsystems subsystems) {

        // 1. The Active Shooter (Tracks and spools continuously)
        Command shooterCommand =
                AimingCommands.shootWithProtectionAndAgregiousMaxHeight(subsystems.shooter)
                        .withName("Active Spool & Aim");

        return Commands.parallel(
                        shooterCommand.repeatedly(),
                        IndexerCommands.smartDispense(subsystems.indexer))
                .withName("SMART STREAM");
    }

    public static Command fireAtTarget(Subsystems subsystems, Pose3d targetPose) {
        Command shooterCommand =
                AimingCommands.SHOOT_AT_TARGET(subsystems.shooter, targetPose)
                        .withName("Active Spool & Aim");

        return Commands.parallel(shooterCommand, IndexerCommands.smartDispense(subsystems.indexer))
                .withName("SMART STREAM");
    }

    public static Command IdleShooter(Subsystems subsystems) {
        return AimingCommands.idleShooter(subsystems.shooter)
                .alongWith(IndexerCommands.jorkIt(subsystems.indexer).repeatedly())
                .alongWith(Commands.runOnce(() -> DrivingSharedState.getInstance().setDetune(1.0)))
                .withName("IDLING THE SHOOTER : HIGHER ORDER COMMAND");
    }

    public static Command forceDispense(Subsystems subsystems) {
        // 1. The Active Shooter (Tracks and spools continuously)
        Command shooterCommand =
                AimingCommands.shootWithProtection(subsystems.shooter)
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
        return Commands.parallel(rapidFireStream(subsystems), subsystems.intake.jorkIntake());
    }

    public Supplier<Pose2d> poseSupplier(Subsystems subsystems) {
        return () -> subsystems.swerve.getState().Pose;
    }
}
