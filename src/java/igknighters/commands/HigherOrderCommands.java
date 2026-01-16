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
                                                FieldConstants.HUB.POSITION.getX()
                                                                * FieldConstants.HUB.POSITION.getX()
                                                        + FieldConstants.HUB.POSITION.getY()
                                                                * FieldConstants.HUB.POSITION
                                                                        .getY()))),
                        IndexerCommands.dispense())
                .withTimeout(timeout); // this is a placeholder for IndexerCommands.isBallPresent()
    }
}
