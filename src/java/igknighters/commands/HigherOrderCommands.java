package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.Subsystems;

public class HigherOrderCommands {
    public static Command shootTillEmpty(Subsystems subsystems, double timeout) {
        return Commands.parallel(
                        ShooterCommands.aimAtHub(
                                subsystems.shooter,
                                () -> subsystems.swerve.getState().Pose,
                                subsystems.shooter.getEstimatedRPM(
                                        Math.sqrt(
                                                FieldConstants.HUB.POSITION_BLUE.getX()
                                                                * FieldConstants.HUB.POSITION_BLUE
                                                                        .getX()
                                                        + FieldConstants.HUB.POSITION_BLUE.getY()
                                                                * FieldConstants.HUB.POSITION_BLUE
                                                                        .getY()))),
                        IndexerCommands.dispense(subsystems.indexer, 100.0))
                .withName("Shoot till empty")
                .withTimeout(timeout); // this is a placeholder for IndexerCommands.isBallPresent()
    }
}
