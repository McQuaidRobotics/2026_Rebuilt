package igknighters.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.shooter.Shooter;

public class ShooterCommands {
    public static Command homeHood(Shooter shooter) {
        // return Commands.run(() -> shooter.setHoodVoltage(-1)).until(()
        // ->shooter.isHoodSensorHit());
        if (shooter.isHoodSensorHit()) {
            return Commands.none();
        }
        return shooter.hood
                .run(() -> shooter.hood.setHoodVoltage(-1))
                .until(() -> shooter.isHoodSensorHit())
                .withTimeout(3.0)
                .withName("DRIVE DOWN HAS NOT HIT THE SENSOR YET HOME HOOD")
                .andThen(
                        Commands.runOnce(
                                () -> {
                                    shooter.hood.setHoodVoltage(0);
                                    shooter.hood.zeroAt(Degrees.of(kHood.MIN_ANGLE_DEGREES));
                                }))
                .withName("HOOD IS DOWN ON SENSOR");
    }
}
