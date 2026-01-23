package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.Subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

public class HigherOrderCommands {
    public static Command shootTillEmpty(
            Subsystems subsystems, double timeout, Supplier<Pose3d> targetPoseSupplier) {
        return Commands.parallel(
                        ShooterCommands.aimAt(
                                        subsystems.shooter,
                                        () -> subsystems.swerve.getState().Pose,
                                        targetPoseSupplier)
                                .withName("Aim At in Shoot till Empty"),
                        IndexerCommands.dispense(subsystems.indexer, 100.0))
                .onlyIf(() -> subsystems.shooter.atTarget(.5))
                .withTimeout(timeout); // this is a placeholder for IndexerCommands.isBallPresent()
    }

    public static Command shootNoStop(Subsystems subsystems, Supplier<Pose3d> targetPoseSupplier) {
        return Commands.parallel(
                        ShooterCommands.aimAt(
                                subsystems.shooter,
                                () -> subsystems.swerve.getState().Pose,
                                targetPoseSupplier),
                        IndexerCommands.dispense(subsystems.indexer, 100.0))
                .onlyIf(() -> subsystems.shooter.atTarget(.5));
    }
    public static Pose2d getClimbStartPose() {
        if (Robot.isBlue()) {
            return new Pose2d(FieldConstants.CLIMB.POSITION_BLUE.getX() + 2.0, FieldConstants.CLIMB.POSITION_BLUE.getY(), FieldConstants.CLIMB.POSITION_BLUE.getRotation());
        } else {
            return new Pose2d(FieldConstants.CLIMB.POSITION_RED.getX() - 2.0, FieldConstants.CLIMB.POSITION_RED.getY(), FieldConstants.CLIMB.POSITION_RED.getRotation());
        }
    }
    public static Pose2d getClimbEndPose() {
        if (Robot.isBlue()) {
            return FieldConstants.CLIMB.POSITION_BLUE;
        } else {
            return FieldConstants.CLIMB.POSITION_RED;
        }
    }
    public static Command climbFirstRungWithLineup(Subsystems subsystems) {
        return Commands.runOnce(
                    () -> SwerveCommands.moveToSimple(subsystems.swerve, getClimbStartPose()).until(SwerveCommands.isAt(subsystems.swerve, getClimbStartPose(), .05, 0.1))
                        .andThen(SwerveCommands.moveToSimpleWithVelocityControl(subsystems.swerve, getClimbEndPose(), new Pose2d(.1, 1., new Rotation2d(0.2)))).until(ClimberCommands.isBumperPressed(subsystems.climber)).andThen(ClimberCommands.goToMax(subsystems.climber))
        );
}
     
}
