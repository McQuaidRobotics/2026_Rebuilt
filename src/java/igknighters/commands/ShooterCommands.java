package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.constants.Conv;
import igknighters.constants.FieldConstants;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.shooter.AimSolver;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.shooter.ShooterState;
import java.util.function.Supplier;

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

    public static Command idle(Shooter shooter) {
        return shooter.run(() -> shooter.targetState(3000, 0, 0));
    }

    public static Command aimAtHub(
            Shooter shooter, Supplier<Pose2d> robotPoseSupplier, double RPM) {
        return shooter.run(
                () -> {
                    Pose2d robotPose = robotPoseSupplier.get();
                    ShooterState targetingData =
                            AimSolver.solve_simple_no_AR_or_FutureTiming(
                                    FieldConstants.HUB.POSE3D,
                                    new Pose3d(
                                            robotPose.getX(),
                                            robotPose.getY(),
                                            SubsystemConstants.kShooter
                                                    .kRollers
                                                    .ShooterHeightMeters,
                                            new Rotation3d(
                                                    0.0,
                                                    0.0,
                                                    robotPose.getRotation().getRadians())),
                                    shooter.getCurrentState().rpm);

                    if (targetingData.rpm != 0.0) {
                        shooter.targetState(
                                shooter.getCurrentState().rpm,
                                Math.toDegrees(targetingData.turretAngleRads),
                                Math.toDegrees(targetingData.hoodAngleRads));
                    } else {
                        shooter.targetState(
                                shooter.getCurrentState().rpm + 100.0,
                                targetingData.turretAngleRads * Conv.RADIANS_TO_DEGREES,
                                targetingData.hoodAngleRads
                                        * Conv.RADIANS_TO_DEGREES); // keep trying to increase RPM
                        // to reach shot
                    }
                });
    }
}
