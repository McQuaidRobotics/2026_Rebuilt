package igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.shooter.Shooter;

public class ShooterCommands {
    public static Command shootAtSpeed(Shooter shooter, double RPM) {
        return shooter.run(() -> shooter.targetState(RPM, 0, 0));
    }

    public static Command stopShooting(Shooter shooter) {
        return shooter.runOnce(() -> shooter.setRollerVoltage(0));
    }

    public static Command aimTurretAtAngle(
            Shooter shooter, double turretAngleDegrees, double hoodAngleDegrees) {
        return shooter.run(() -> shooter.targetState(0, turretAngleDegrees, hoodAngleDegrees));
    }
}
