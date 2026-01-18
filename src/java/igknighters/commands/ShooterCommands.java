package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.shooter.AimSolver;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.shooter.ShooterState;
import java.util.function.Supplier;

public class ShooterCommands {
    public static Command shootAtSpeed(Shooter shooter, double RPM) {
        return shooter.run(() -> shooter.targetState(RPM, 0, 0)).withName("shoot at speed: " + RPM);
    }

    public static Command stopShooting(Shooter shooter) {
        return shooter.runOnce(() -> shooter.setRollerVoltage(0)).withName("stop shooting");
    }

    public static Command aimTurretAtAngle(
            Shooter shooter, double turretAngleDegrees, double hoodAngleDegrees) {
        return shooter.run(() -> shooter.targetState(0, turretAngleDegrees, hoodAngleDegrees))
                .withName("aiming turret + hood");
    }

    public static Command idle(Shooter shooter) {
        return shooter.run(() -> shooter.targetState(3000, 0, 0)).withName("Idle Shooter");
    }

    public static Command aimAt(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Pose3d> targetPoseSupplier) {
        return shooter.run(
                        () -> {
                            Pose2d robotPose = robotPoseSupplier.get();
                            Pose3d targetPose = targetPoseSupplier.get();
                            double RPM =
                                    shooter.getEstimatedRPM(
                                            Math.sqrt(
                                                    Math.pow(
                                                                    targetPose.getX()
                                                                            - robotPose.getX(),
                                                                    2)
                                                            + Math.pow(
                                                                    targetPose.getY()
                                                                            - robotPose.getY(),
                                                                    2)));
                            ShooterState targetingData =
                                    AimSolver.solve_simple_no_AR_or_FutureTiming(
                                            targetPose,
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
                                        RPM,
                                        Math.toDegrees(targetingData.turretAngleRads),
                                        Math.toDegrees(targetingData.hoodAngleRads));
                            } else {
                                shooter.targetState(
                                        shooter.getCurrentState().rpm + 100.0,
                                        targetingData.turretAngleRads * Conv.RADIANS_TO_DEGREES,
                                        targetingData.hoodAngleRads
                                                * Conv.RADIANS_TO_DEGREES); // keep trying to
                                // increase RPM
                                // to reach shot
                            }
                        })
                .withName("Aiming at hub");
    }

    public static Command aimAt(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Pose3d> targetPoseSupplier,
            double RPM) {
        return shooter.run(
                        () -> {
                            Pose2d robotPose = robotPoseSupplier.get();
                            Pose3d targetPose = targetPoseSupplier.get();
                            ShooterState targetingData =
                                    AimSolver.solve_simple_no_AR_or_FutureTiming(
                                            targetPose,
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
                                        RPM,
                                        Math.toDegrees(targetingData.turretAngleRads),
                                        Math.toDegrees(targetingData.hoodAngleRads));
                            } else {
                                shooter.targetState(
                                        shooter.getCurrentState().rpm + 100.0,
                                        targetingData.turretAngleRads * Conv.RADIANS_TO_DEGREES,
                                        targetingData.hoodAngleRads
                                                * Conv.RADIANS_TO_DEGREES); // keep trying to
                                // increase RPM
                                // to reach shot
                            }
                        })
                .withName("Aiming at hub");
    }
}
