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
                        ShooterCommands.shoot(
                                        subsystems.shooter,
                                        () -> subsystems.swerve.getState().Pose,
                                        subsystems.swerve::getFieldRelativeSpeeds)
                                .withName("Aim At in Shoot till Empty"),
                        IndexerCommands.dispense(subsystems.indexer)
                                .onlyIf(() -> subsystems.shooter.atTarget(300, 2, 2)))
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
                                .onlyIf(() -> subsystems.shooter.atTarget(500, 5, 5)))
                .withName("DISPENSING")
                .alongWith(Commands.print("DISPENSING"))
                .withName("SHOOT NO STOP");
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

    public static Command prepToClimbFirstRung(Subsystems subsystems) {
        return Commands.defer(
                        () -> {
                            Pose2d startPose = getClimbStartPose();
                            Pose2d endPose = getClimbEndPose();
                            return Commands.sequence(
                                    Commands.print("STARTING AUTO ALIGNMENT TO CLIMB"),
                                    SwerveCommands.moveToSimple(subsystems.swerve, startPose)
                                            .until(
                                                    SwerveCommands.isAt(
                                                            subsystems.swerve,
                                                            startPose,
                                                            0.03,
                                                            0.1)),
                                    Commands.print("REACHED STARTING POSE FOR CLIMB LINEUP"),
                                    SwerveCommands.moveToSimpleWithVelocityControl(
                                                    subsystems.swerve,
                                                    endPose,
                                                    new Pose2d(.5, .5, new Rotation2d(1)))
                                            .until(subsystems.climber::isSensorHit),
                                    Commands.print("REACHED CLIMBING POSITION"),
                                    ClimberCommands.goUp(subsystems.climber),
                                    Commands.print("CLIMBER IS PREPED TO RUN"));
                        },
                        java.util.Set.of(subsystems.swerve, subsystems.climber))
                .withName("Moving to Climber and raising to max height");
    }
}
