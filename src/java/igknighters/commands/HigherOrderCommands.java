package igknighters.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.Subsystems;
import java.util.function.Supplier;

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
                .withName("dispense balls into shooter")
                .withName("Shoot till empty")
                .withTimeout(timeout); // this is a placeholder for IndexerCommands.isBallPresent()
    }

    public static Command shootNoStop(Subsystems subsystems, Supplier<Pose3d> targetPoseSupplier) {
        return Commands.parallel(
                        ShooterCommands.aimAt(
                                subsystems.shooter,
                                () -> subsystems.swerve.getState().Pose,
                                targetPoseSupplier),
                        IndexerCommands.dispense(subsystems.indexer, 100.0))
                .onlyIf(() -> subsystems.shooter.atTarget(.5))
                .withName("Shoot no stop");
    }
}
