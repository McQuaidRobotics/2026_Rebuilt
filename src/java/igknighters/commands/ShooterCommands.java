package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.constants.Conv;
import igknighters.constants.FieldConstants;
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

    public static Command targetState(Shooter shooter, double RPM, double turretAngleDegrees, double hoodAngleDegrees) {
        return shooter.run(() -> shooter.targetState(RPM, turretAngleDegrees, hoodAngleDegrees))
                .withName("Target Shooter State");
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
                                            Math.pow(targetPose.getX() - robotPose.getX(), 2)
                                                    + Math.pow(
                                                            targetPose.getY() - robotPose.getY(),
                                                            2)));
                    ShooterState targetingData =
                            AimSolver.Solvers.solve_simple_no_AR_or_FutureTiming(
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
                });
    }

    public static Pose3d getHubTarget() {
        return Robot.isBlue() ? FieldConstants.HUB.POSE3D_BLUE : FieldConstants.HUB.POSE3D_RED;
    }

    public static Pose3d getPassTarget(Supplier<Pose2d> robotPoSupplier) {
        if (Robot.isBlue()) {
            Pose2d robotPose2d = robotPoSupplier.get();
            return robotPose2d.getY() > FieldConstants.WIDTH / 2
                    ? FieldConstants.PASS.POSITION_LEFT_BLUE
                    : FieldConstants.PASS.POSITION_RIGHT_BLUE;
        } else {
            Pose2d robotPose2d = robotPoSupplier.get();
            return robotPose2d.getY() > FieldConstants.WIDTH / 2
                    ? FieldConstants.PASS.POSITION_LEFT_RED
                    : FieldConstants.PASS.POSITION_RIGHT_RED;
        }
    }

    public static boolean shouldPass(Supplier<Pose2d> robotPoseSupplier) {
        if (Robot.isBlue()) {
            return robotPoseSupplier.get().getX() > FieldConstants.ALIANCE_ZONE_BLUE;
        } else {
            return robotPoseSupplier.get().getX() < FieldConstants.ALIANCE_ZONE_RED;
        }
    }

    public static Pose3d getTargetPose(Supplier<Pose2d> robotPoseSupplier) {
        if (shouldPass(robotPoseSupplier)) {
            return getPassTarget(robotPoseSupplier);
        } else {
            return getHubTarget();
        }
    }

    public static double getRPM(
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Pose3d> targetPoseSupplier,
            Shooter shooter) {
        Pose2d robotPose = robotPoseSupplier.get();
        Pose3d targetPose = targetPoseSupplier.get();
        return shooter.getEstimatedRPM(
                Math.sqrt(
                        Math.pow(targetPose.getX() - robotPose.getX(), 2)
                                + Math.pow(targetPose.getY() - robotPose.getY(), 2)));
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
                                    AimSolver.Solvers.solve_simple_no_AR_or_FutureTiming(
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

    public static Command shootIChoseTargetNoLookAhead(
            Shooter shooter, Supplier<Pose2d> robotPose) {
        return shooter.run(
                        () -> {
                            Pose2d robotPose2d = robotPose.get();
                            Pose3d targetPose = getTargetPose(robotPose);
                            double RPM = getRPM(robotPose, () -> targetPose, shooter);

                            ShooterState targetingData =
                                    AimSolver.Solvers.solve_simple_no_AR_or_FutureTiming(
                                            targetPose,
                                            new Pose3d(
                                                    robotPose2d.getX(),
                                                    robotPose2d.getY(),
                                                    SubsystemConstants.kShooter
                                                            .kRollers
                                                            .ShooterHeightMeters,
                                                    new Rotation3d(
                                                            0.0,
                                                            0.0,
                                                            robotPose2d
                                                                    .getRotation()
                                                                    .getRadians())),
                                            shooter.getCurrentState().rpm);
                            if (targetingData.rpm != 0.0) {
                                shooter.targetState(
                                        RPM,
                                        Math.toDegrees(targetingData.turretAngleRads),
                                        Math.toDegrees(targetingData.hoodAngleRads));
                            } else {
                                if (shooter.getCurrentState().rpm < RPM) {
                                    shooter.targetState(
                                            RPM,
                                            targetingData.turretAngleRads * Conv.RADIANS_TO_DEGREES,
                                            targetingData.hoodAngleRads
                                                    * Conv.RADIANS_TO_DEGREES); // keep trying to

                                } else {
                                    shooter.targetState(
                                            shooter.getCurrentState().rpm + 100.0,
                                            targetingData.turretAngleRads * Conv.RADIANS_TO_DEGREES,
                                            targetingData.hoodAngleRads * Conv.RADIANS_TO_DEGREES);
                                } // increase RPM to reach shot
                            }
                        })
                .withName("Aiming at auto chosen target");
    }



    public static Command shootIChoseTargetWithLookAhead(
            Shooter shooter, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
        return shooter.run(
                        () -> {
                            Pose2d robotPose2d = robotPose.get();
                            Pose3d targetPose = getTargetPose(robotPose);
                            double RPM = getRPM(robotPose, () -> targetPose, shooter);

                            ShooterState targetingData =
                                    AimSolver.Solvers.solve_moving(
                                            targetPose,
                                            new Pose3d(
                                                    robotPose2d.getX(),
                                                    robotPose2d.getY(),
                                                    SubsystemConstants.kShooter
                                                            .kRollers
                                                            .ShooterHeightMeters,
                                                    new Rotation3d(
                                                            0.0,
                                                            0.0,
                                                            robotPose2d
                                                                    .getRotation()
                                                                    .getRadians())),
                                            shooter.getCurrentState().rpm,
                                            robotVelocity.get(),
                                            0.1);

                            if (targetingData.rpm != 0.0) {
                                shooter.targetState(
                                        RPM,
                                        Math.toDegrees(targetingData.turretAngleRads),
                                        Math.toDegrees(targetingData.hoodAngleRads));
                            } else {
                                if (shooter.getCurrentState().rpm < (RPM - 500)) {
                                    shooter.targetState(
                                            RPM,
                                            targetingData.turretAngleRads * Conv.RADIANS_TO_DEGREES,
                                            targetingData.hoodAngleRads
                                                    * Conv.RADIANS_TO_DEGREES); // keep trying to

                                } else {
                                    shooter.targetState(
                                            shooter.getCurrentState().rpm + 100.0,
                                            targetingData.turretAngleRads * Conv.RADIANS_TO_DEGREES,
                                            targetingData.hoodAngleRads * Conv.RADIANS_TO_DEGREES);
                                } // increase RPM to reach shot
                            }
                        })
                .withName("Aiming at auto chosen target with look ahead");
    }
}
