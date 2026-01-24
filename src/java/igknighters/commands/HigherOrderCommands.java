package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.Subsystems;

public class HigherOrderCommands {
    public static Command shootTillEmpty(Subsystems subsystems, double timeout) {
        return Commands.parallel(
                        ShooterCommands.shootIChoseTargetWithLookAhead(
                                        subsystems.shooter,
                                        () -> subsystems.swerve.getState().Pose,
                                        () -> subsystems.swerve.getState().Speeds)
                                .withName("Aim At in Shoot till Empty"),
                        IndexerCommands.dispense(subsystems.indexer, 100.0))
                .onlyIf(() -> subsystems.shooter.atTarget(.5))
                .withTimeout(timeout); // this is a placeholder for IndexerCommands.isBallPresent()
    }

    public static Command shootNoStop(Subsystems subsystems) {
        return Commands.parallel(
                        ShooterCommands.shootIChoseTargetWithLookAhead(
                                subsystems.shooter,
                                () -> subsystems.swerve.getState().Pose,
                                () -> subsystems.swerve.getState().Speeds),
                        IndexerCommands.dispense(subsystems.indexer, 100.0))
                .onlyIf(() -> subsystems.shooter.atTarget(.5));
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

    public static Command prepToClimbFirstRung(Subsystems subsystems) {
        return Commands.runOnce(
                        () ->
                                SwerveCommands.moveToSimple(subsystems.swerve, getClimbStartPose())
                                        .until(
                                                SwerveCommands.isAt(
                                                        subsystems.swerve,
                                                        getClimbStartPose(),
                                                        .05,
                                                        0.1))
                                        .withName("Moving to start pose")
                                        .andThen(
                                                SwerveCommands.moveToSimpleWithVelocityControl(
                                                        subsystems.swerve,
                                                        getClimbEndPose(),
                                                        new Pose2d(.1, 1., new Rotation2d(0.2))))
                                        .withName("Moving to end pose")
                                        .until(ClimberCommands.isBumperPressed(subsystems.climber))
                                        .andThen(SwerveCommands.stopDriving(subsystems.swerve))
                                        .withName("stopping driving")
                                        .andThen(ClimberCommands.goToMax(subsystems.climber)))
                .withName("raising to max height")
                .withName("Moving to Climber and raising to max height");
    }
}
