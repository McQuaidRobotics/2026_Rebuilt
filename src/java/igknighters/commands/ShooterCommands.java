package igknighters.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.constants.Conv;
import igknighters.constants.FieldConstants;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kFlywheels;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.shooter.AimSolver;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.util.TunableValues;
import igknighters.util.log.Log;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ShooterCommands {

    public static TunableValues.TunableDouble shootRPM =
            TunableValues.getDouble("Shooter/ShootRPM", 3000);
    public static TunableValues.TunableDouble shootHoodAngle =
            TunableValues.getDouble("Shooter/ShootHoodAngle", kHood.MIN_ANGLE_DEGREES);

    public static Command shootAtSpeed(Shooter shooter, double speed) {
        return shooter.run(() -> shooter.targetState(RPM.of(speed), Degrees.of(0), Degrees.of(0)))
                .withName("shoot at speed: " + speed);
    }

    public static Command targetNetworkTablesValues(Shooter shooter) {
        return shooter.run(
                () ->
                        shooter.targetState(
                                RPM.of(shootRPM.value()),
                                Degrees.of(0),
                                Degrees.of(shootHoodAngle.value())));
    }

    public static Command targetState(Shooter shooter, ShooterState state) {
        return shooter.run(
                () -> shooter.targetState(state.flywheelSpeed, state.turretAngle, state.hoodAngle));
    }

    public static Command stopShooting(Shooter shooter) {
        return shooter.runOnce(() -> shooter.setRollerVoltage(0)).withName("stop shooting");
    }

    public static Command aimTurretAtAngle(
            Shooter shooter, double turretAngleDegrees, double hoodAngleDegrees) {
        return shooter.run(
                        () ->
                                shooter.targetState(
                                        RPM.of(0),
                                        Degrees.of(turretAngleDegrees),
                                        Degrees.of(hoodAngleDegrees)))
                .withName("aiming turret + hood");
    }

    public static Command idle(Shooter shooter) {
        return shooter.run(() -> shooter.targetState(RPM.of(3000), Degrees.of(0), Degrees.of(0)))
                .withName("Idle Shooter");
    }

    public static Command targetState(
            Shooter shooter, double speed, double turretAngleDegrees, double hoodAngleDegrees) {
        return shooter.run(
                        () ->
                                shooter.targetState(
                                        RPM.of(speed),
                                        Degrees.of(turretAngleDegrees),
                                        Degrees.of(hoodAngleDegrees)))
                .withName("Target Shooter State");
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
            Supplier<ChassisSpeeds> robotVelocitySupplier,
            double velocity) {
        return shooter.run(
                        () -> {
                            ShootInformation.getInstance().setBeingControlled(true);
                            Pose2d robotPose = robotPoseSupplier.get();
                            Pose3d targetPose = targetPoseSupplier.get();
                            ShooterState targetingData =
                                    AimSolver.Solvers.solve_max_and_min_iterative(
                                            new Pose3d(
                                                    robotPose.getX(),
                                                    robotPose.getY(),
                                                    SubsystemConstants.kShooter
                                                            .kFlywheels
                                                            .ShooterHeightMeters,
                                                    new Rotation3d(
                                                            0.0,
                                                            0.0,
                                                            robotPose.getRotation().getRadians())),
                                            targetPose,
                                            robotVelocitySupplier.get(),
                                            shooter.getCurrentState().flywheelSpeed.in(RPM),
                                            4,
                                            2,
                                            0.02);

                            if (targetingData.flywheelSpeed.in(RPM) != 0.0) {
                                shooter.targetState(
                                        targetingData.flywheelSpeed,
                                        targetingData.turretAngle,
                                        targetingData.hoodAngle);
                            } else {
                                shooter.targetState(
                                        shooter.getCurrentState().flywheelSpeed.plus(RPM.of(100.0)),
                                        targetingData.turretAngle,
                                        targetingData.hoodAngle); // keep trying to
                                // increase RPM
                                // to reach shot
                            }
                        })
                .withName(
                        "Aiming at: "
                                + targetPoseSupplier.get().getX()
                                + ", "
                                + targetPoseSupplier.get().getY());
    }

    public static Supplier<Pose2d> getShooterPoseWithOffset(Supplier<Pose2d> robotPose) {
        return () ->
                robotPose
                        .get()
                        .plus(
                                new Transform2d(
                                        -5 * Conv.INCHES_TO_METERS,
                                        -5 * Conv.INCHES_TO_METERS,
                                        new Rotation2d()));
    }

    public static Command shoot(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {
        Supplier<Pose2d> shooterPose = getShooterPoseWithOffset(robotPoseSupplier);
        return SHOOT_MAX_MIN(
                shooter,
                shooterPose,
                robotVelocitySupplier,
                4,
                FieldConstants.HUB.HEIGHT_METERS + .5);
    }

    public static boolean isBetween(Pose2d pose, double a, double b) {
        double x = pose.getX();
        return x >= a && x <= b;
    }

    public static BooleanSupplier isUnderTrench(Supplier<Pose2d> robotPoseSupplier) {
        return () -> {
            Pose2d pose = robotPoseSupplier.get();
            boolean under1 =
                    isBetween(
                            pose,
                            FieldConstants.BUMP.BUMP_1_X_METERS - .5,
                            FieldConstants.BUMP.BUMP_1_X_METERS + .5);
            boolean under2 =
                    isBetween(
                            pose,
                            FieldConstants.BUMP.BUMP_2_X_METERS - .5,
                            FieldConstants.BUMP.BUMP_2_X_METERS + .5);

            boolean isUnder = under1 || under2;
            if (!SubsystemConstants.disableAllLogs) {
                Log.log("Shooter/isUnderTrench", isUnder);
                Log.log("Shooter/RobotX", pose.getX());
            }

            return isUnder;
        };
    }

    public static Command shootWithProtection(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {
        ShootInformation info = ShootInformation.getInstance();
        BooleanSupplier underTrenchCheck = isUnderTrench(robotPoseSupplier);

        return shooter.run(
                () -> {
                    if (underTrenchCheck.getAsBoolean()) {
                        // --- Idle Logic (from idleCommand) ---
                        info.setBeingControlled(false);
                        Pose2d robotPose2d = robotPoseSupplier.get();
                        Pose3d targetPose = info.getShotLocation(robotPoseSupplier);
                        ChassisSpeeds robotVel = robotVelocitySupplier.get();

                        Pose3d shooterPose =
                                new Pose3d(
                                        robotPose2d.getX(),
                                        robotPose2d.getY(),
                                        SubsystemConstants.kShooter.kFlywheels.ShooterHeightMeters,
                                        new Rotation3d(
                                                0.0, 0.0, robotPose2d.getRotation().getRadians()));
                        ShooterState targetingData =
                                AimSolver.Solvers.solve_max_height_iterative(
                                        shooterPose,
                                        targetPose,
                                        robotVel,
                                        shooter.getCurrentState().flywheelSpeed.in(RPM),
                                        4,
                                        0.02);

                        shooter.targetState(
                                RPM.of(2000),
                                targetingData.turretAngle,
                                Degrees.of(kHood.MIN_ANGLE_DEGREES));
                    } else {
                        // --- Shoot Logic (from shoot/SHOOT_MAX_MIN) ---
                        Supplier<Pose2d> shooterPoseWithOffset =
                                getShooterPoseWithOffset(robotPoseSupplier);
                        Pose2d shooterPose2d = shooterPoseWithOffset.get();
                        Pose3d targetPose = info.getShotLocation(shooterPoseWithOffset);
                        info.setBeingControlled(true);
                        ChassisSpeeds robotVel = robotVelocitySupplier.get();

                        shooter.currentShotType = getShotType(shooterPoseWithOffset);

                        Pose3d shooterPose3d =
                                new Pose3d(
                                        shooterPose2d.getX(),
                                        shooterPose2d.getY(),
                                        SubsystemConstants.kShooter.kFlywheels.ShooterHeightMeters,
                                        new Rotation3d(
                                                0.0,
                                                0.0,
                                                shooterPose2d.getRotation().getRadians()));
                        ShooterState targetingData =
                                AimSolver.Solvers.solve_max_and_min_iterative(
                                        shooterPose3d,
                                        targetPose,
                                        robotVel,
                                        shooter.getCurrentState().flywheelSpeed.in(RPM),
                                        4,
                                        FieldConstants.HUB.HEIGHT_METERS + .5,
                                        0.02);

                        if (targetingData.flywheelSpeed.in(RPM) != 0) {
                            shooter.targetState(targetingData);
                        } else {
                            // shot is imposible so we should idle the shooter rpm at like 4000
                            shooter.targetState(
                                    RPM.of(4000),
                                    targetingData.turretAngle,
                                    Degrees.of(kHood.MIN_ANGLE_DEGREES));
                        }
                    }
                });
    }

    /**
     * aims without changing hood or rpm so that the shooter can go under bump
     *
     * @param shooter
     * @param robotPoseSupplier
     * @param robotVelocitySupplier
     * @return
     */
    public static Command idleCommand(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {
        ShootInformation info = ShootInformation.getInstance();
        return shooter.run(
                () -> {
                    info.setBeingControlled(false);
                    Pose2d robotPose2d = robotPoseSupplier.get();
                    Pose3d targetPose = info.getShotLocation(robotPoseSupplier);
                    ChassisSpeeds robotVel = robotVelocitySupplier.get();

                    Pose3d shooterPose =
                            new Pose3d(
                                    robotPose2d.getX(),
                                    robotPose2d.getY(),
                                    SubsystemConstants.kShooter.kFlywheels.ShooterHeightMeters,
                                    new Rotation3d(
                                            0.0, 0.0, robotPose2d.getRotation().getRadians()));
                    ShooterState targetingData =
                            AimSolver.Solvers.solve_max_height_iterative(
                                    shooterPose,
                                    targetPose,
                                    robotVel,
                                    shooter.getCurrentState().flywheelSpeed.in(RPM),
                                    5,
                                    0.02);

                    shooter.targetState(
                            RPM.of(2000),
                            targetingData.turretAngle,
                            Degrees.of(kHood.MIN_ANGLE_DEGREES));
                });
    }

    public static Command shootWithMaxHeightIterative(
            Shooter shooter,
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> robotVeloSupplier,
            double maxHeightMeters) {
        ShootInformation info = ShootInformation.getInstance();
        return shooter.run(
                () -> {
                    Pose2d robotPose2d = robotPose.get();
                    Pose3d targetPose = info.getShotLocation(robotPose);
                    info.setBeingControlled(true);
                    ChassisSpeeds robotVel = robotVeloSupplier.get();

                    Pose3d shooterPose =
                            new Pose3d(
                                    robotPose2d.getX(),
                                    robotPose2d.getY(),
                                    SubsystemConstants.kShooter.kFlywheels.ShooterHeightMeters,
                                    new Rotation3d(
                                            0.0, 0.0, robotPose2d.getRotation().getRadians()));
                    ShooterState targetingData =
                            AimSolver.Solvers.solve_max_height_iterative(
                                    shooterPose,
                                    targetPose,
                                    robotVel,
                                    shooter.getCurrentState().flywheelSpeed.in(RPM),
                                    maxHeightMeters,
                                    0.02);

                    if (targetingData.flywheelSpeed.in(RPM) != 0) {
                        shooter.targetState(targetingData);
                    } else {
                        // shot is imposible so we should idle the shooter rpm at like 4000 so it
                        // spins up faster
                        shooter.targetState(
                                RPM.of(4000),
                                targetingData.turretAngle,
                                Degrees.of(kHood.MIN_ANGLE_DEGREES));
                    }
                });
    }

    public static Command SHOOT_MAX_MIN(
            Shooter shooter,
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> robotVeloSupplier,
            double maxHeightMeters,
            double minHeightMeters) {
        ShootInformation info = ShootInformation.getInstance();
        return shooter.run(
                () -> {
                    Pose2d robotPose2d = robotPose.get();
                    Pose3d targetPose = info.getShotLocation(robotPose);
                    info.setBeingControlled(true);
                    ChassisSpeeds robotVel = robotVeloSupplier.get();

                    shooter.currentShotType = getShotType(robotPose);

                    Pose3d shooterPose =
                            new Pose3d(
                                    robotPose2d.getX(),
                                    robotPose2d.getY(),
                                    SubsystemConstants.kShooter.kFlywheels.ShooterHeightMeters,
                                    new Rotation3d(
                                            0.0, 0.0, robotPose2d.getRotation().getRadians()));
                    ShooterState targetingData =
                            AimSolver.Solvers.solve_max_and_min_iterative(
                                    shooterPose,
                                    targetPose,
                                    robotVel,
                                    shooter.getCurrentState().flywheelSpeed.in(RPM),
                                    maxHeightMeters,
                                    minHeightMeters,
                                    0.02);

                    if (targetingData.flywheelSpeed.in(RPM) != 0) {
                        shooter.targetState(targetingData);
                    } else {
                        // shot is imposible so we should idle the shooter rpm at like 4000 so it
                        // spins up faster
                        shooter.targetState(
                                RPM.of(4000),
                                targetingData.turretAngle,
                                Degrees.of(kHood.MIN_ANGLE_DEGREES));
                    }
                });
    }

    public static enum shotType {
        PASS,
        SHOT
    }

    public static shotType getShotType(Supplier<Pose2d> robotPoseSupplier) {
        ShootInformation info = ShootInformation.getInstance();
        if (info.shouldPass(robotPoseSupplier)) {
            return shotType.PASS;
        } else {
            return shotType.SHOT;
        }
    }

    public static Command SHOOT_MAX_MIN_NO_AUTO_PICKED_TARGET(
            Shooter shooter,
            Pose3d targetPose,
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> robotVeloSupplier,
            double maxHeightMeters,
            double minHeightMeters) {
        return shooter.run(
                () -> {
                    Pose2d robotPose2d = robotPose.get();
                    ChassisSpeeds robotVel = robotVeloSupplier.get();

                    Pose3d shooterPose =
                            new Pose3d(
                                    robotPose2d.getX(),
                                    robotPose2d.getY(),
                                    SubsystemConstants.kShooter.kFlywheels.ShooterHeightMeters,
                                    new Rotation3d(
                                            0.0, 0.0, robotPose2d.getRotation().getRadians()));
                    ShooterState targetingData =
                            AimSolver.Solvers.solve_max_and_min_iterative(
                                    shooterPose,
                                    targetPose,
                                    robotVel,
                                    shooter.getCurrentState().flywheelSpeed.in(RPM),
                                    maxHeightMeters,
                                    minHeightMeters,
                                    0.02);

                    if (targetingData.flywheelSpeed.in(RPM) != 0) {
                        shooter.targetState(targetingData);
                    } else {
                        // shot is imposible so we should idle the shooter rpm at like 4000 so it
                        // spins up faster
                        shooter.targetState(
                                RPM.of(4000),
                                targetingData.turretAngle,
                                Degrees.of(kHood.MIN_ANGLE_DEGREES));
                    }
                });
    }

    public static Command shootAt(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier,
            Supplier<Pose2d> targetPoseSupplier) {
        return shooter.run(
                        () -> {
                            Pose2d robotPose2d = robotPoseSupplier.get();
                            Pose2d targetPose = targetPoseSupplier.get();
                            Pose3d targetPose3d = new Pose3d(targetPose);

                            ShooterState targetingData =
                                    AimSolver.Solvers.solve_max_height_iterative(
                                            new Pose3d(robotPose2d)
                                                    .plus(
                                                            new Transform3d(
                                                                    0,
                                                                    0,
                                                                    kFlywheels.ShooterHeightMeters,
                                                                    new Rotation3d())),
                                            targetPose3d,
                                            robotVelocitySupplier.get(),
                                            shooter.getCurrentState().flywheelSpeed.in(RPM),
                                            5,
                                            0.02);

                            if (targetingData.flywheelSpeed.in(RPM) > 0.1) {
                                shooter.targetState(
                                        targetingData.flywheelSpeed,
                                        targetingData.turretAngle,
                                        targetingData.hoodAngle);
                            } else {
                                // Fallback: Spin up to a safe mid-range RPM and keep turret pointed
                                // at target
                                shooter.targetState(
                                        RPM.of(3000.0),
                                        targetingData.turretAngle,
                                        Degrees.of(kHood.MIN_ANGLE_DEGREES));
                            }
                        })
                .withName("Aiming at auto chosen target with look ahead");
    }
}
