package igknighters.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.constants.AbleToShootSharedState;
import igknighters.constants.FieldConstants;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kFlywheels;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.shooter.AimSolver;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.util.TunableValues;
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

    public static Command aimAt(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Pose3d> targetPoseSupplier) {
        return shooter.run(
                () -> {
                    Pose2d robotPose = robotPoseSupplier.get();
                    Pose3d targetPose = targetPoseSupplier.get();
                    double estimatedRPM =
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
                                                    .kFlywheels
                                                    .ShooterHeightMeters,
                                            new Rotation3d(
                                                    0.0,
                                                    0.0,
                                                    robotPose.getRotation().getRadians())),
                                    shooter.getCurrentState().flywheelSpeed.in(RPM));

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
                });
    }

    public static Pose3d getHubTarget() {
        return Robot.isBlue() ? FieldConstants.HUB.POSE3D_BLUE : FieldConstants.HUB.POSE3D_RED;
    }

    public static Pose3d getPassTarget(Supplier<Pose2d> robotPoSupplier) {
        if (Robot.isBlue()) {
            Pose2d robotPose2d = robotPoSupplier.get();
            return robotPose2d.getY() > FieldConstants.Y_FIELD / 2
                    ? FieldConstants.PASS.POSITION_LEFT_BLUE
                    : FieldConstants.PASS.POSITION_RIGHT_BLUE;
        } else {
            Pose2d robotPose2d = robotPoSupplier.get();
            return robotPose2d.getY() > FieldConstants.Y_FIELD / 2
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
            double velocity) {
        return shooter.run(
                        () -> {
                            AbleToShootSharedState.getInstance().setBeingControlled(true);
                            Pose2d robotPose = robotPoseSupplier.get();
                            Pose3d targetPose = targetPoseSupplier.get();
                            ShooterState targetingData =
                                    AimSolver.Solvers.solve_simple_no_AR_or_FutureTiming(
                                            targetPose,
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
                                            shooter.getCurrentState().flywheelSpeed.in(RPM));

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

    public static Command shootIChoseTargetNoLookAhead(
            Shooter shooter, Supplier<Pose2d> robotPose) {
        return shooter.run(
                        () -> {
                            Pose2d robotPose2d = robotPose.get();
                            Pose3d targetPose = getTargetPose(robotPose);
                            double velocity = getRPM(robotPose, () -> targetPose, shooter);
                            AbleToShootSharedState.getInstance().setBeingControlled(true);

                            ShooterState targetingData =
                                    AimSolver.Solvers.solve_simple_no_AR_or_FutureTiming(
                                            targetPose,
                                            new Pose3d(
                                                    robotPose2d.getX(),
                                                    robotPose2d.getY(),
                                                    SubsystemConstants.kShooter
                                                            .kFlywheels
                                                            .ShooterHeightMeters,
                                                    new Rotation3d(
                                                            0.0,
                                                            0.0,
                                                            robotPose2d
                                                                    .getRotation()
                                                                    .getRadians())),
                                            shooter.getCurrentState().flywheelSpeed.in(RPM));
                            if (targetingData.flywheelSpeed.in(RPM) != 0.0) {
                                shooter.targetState(
                                        RPM.of(velocity),
                                        targetingData.turretAngle,
                                        targetingData.hoodAngle);
                            } else {
                                if (shooter.getCurrentState().flywheelSpeed.in(RPM) < velocity) {
                                    shooter.targetState(
                                            RPM.of(velocity),
                                            targetingData.turretAngle,
                                            targetingData.hoodAngle); // keep trying to

                                } else {
                                    shooter.targetState(
                                            shooter.getCurrentState()
                                                    .flywheelSpeed
                                                    .plus(RPM.of(300.0)),
                                            targetingData.turretAngle,
                                            targetingData.hoodAngle);
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
                            AbleToShootSharedState.getInstance().setBeingControlled(true);
                            Pose3d targetPose = getTargetPose(robotPose);
                            double velocity = getRPM(robotPose, () -> targetPose, shooter);

                            ShooterState targetingData =
                                    AimSolver.Solvers.solve_with_project(
                                            targetPose,
                                            new Pose3d(
                                                    robotPose2d.getX(),
                                                    robotPose2d.getY(),
                                                    SubsystemConstants.kShooter
                                                            .kFlywheels
                                                            .ShooterHeightMeters,
                                                    new Rotation3d(
                                                            0.0,
                                                            0.0,
                                                            robotPose2d
                                                                    .getRotation()
                                                                    .getRadians())),
                                            shooter.getCurrentState().flywheelSpeed.in(RPM),
                                            robotVelocity.get(),
                                            0.02);

                            if (targetingData.flywheelSpeed.in(RPM) != 0.0) {
                                shooter.targetState(
                                        RPM.of(velocity),
                                        targetingData.turretAngle,
                                        targetingData.hoodAngle);
                            } else {
                                if (shooter.getCurrentState().flywheelSpeed.in(RPM)
                                        < (velocity - 500)) {
                                    shooter.targetState(
                                            RPM.of(velocity),
                                            targetingData.turretAngle,
                                            targetingData.hoodAngle); // keep trying to

                                } else {
                                    shooter.targetState(
                                            shooter.getCurrentState()
                                                    .flywheelSpeed
                                                    .plus(RPM.of(300.0)),
                                            targetingData.turretAngle,
                                            targetingData.hoodAngle);
                                } // increase RPM to reach shot
                            }
                        })
                .withName("Aiming at auto chosen target with look ahead");
    }

    public static Command shoot(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {
        return Commands.sequence(
                Commands.runOnce(
                        () -> AbleToShootSharedState.getInstance().setBeingControlled(true)),
                SHOOT_MAX_MIN(
                        shooter,
                        robotPoseSupplier,
                        robotVelocitySupplier,
                        4,
                        FieldConstants.HUB.HEIGHT_METERS + .5));
    }

    public static Command idleCommand(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {
        return shooter.run(
                () -> {
                    AbleToShootSharedState.getInstance().setBeingControlled(false);
                    Pose2d robotPose2d = robotPoseSupplier.get();
                    Pose3d targetPose = getTargetPose(robotPoseSupplier);
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
        return shooter.run(
                () -> {
                    Pose2d robotPose2d = robotPose.get();
                    Pose3d targetPose = getTargetPose(robotPose);
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
        return shooter.run(
                () -> {
                    Pose2d robotPose2d = robotPose.get();
                    Pose3d targetPose = getTargetPose(robotPose);
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
        if (shouldPass(robotPoseSupplier)) {
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

    public static Command shootWithMaxHeight(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier,
            double maxHeightMeters) {
        return shooter.run(
                        () -> {
                            Pose2d robotPose = robotPoseSupplier.get();
                            Pose3d targetPose = getTargetPose(robotPoseSupplier);
                            ChassisSpeeds robotVel = robotVelocitySupplier.get();

                            // Define where the shooter is physically located on the robot
                            Pose3d shooterPose =
                                    new Pose3d(
                                            robotPose.getX(),
                                            robotPose.getY(),
                                            SubsystemConstants.kShooter
                                                    .kFlywheels
                                                    .ShooterHeightMeters,
                                            new Rotation3d(
                                                    0.0,
                                                    0.0,
                                                    robotPose.getRotation().getRadians()));

                            // Solve for the state.
                            // Note: currentRPM is passed but effectively overridden by the solver
                            // logic
                            ShooterState targetingData =
                                    AimSolver.Solvers.solve_with_max_height(
                                            targetPose,
                                            shooterPose,
                                            shooter.getCurrentState().flywheelSpeed.in(RPM),
                                            robotVel,
                                            0.02, // 20ms lookahead for robot movement
                                            maxHeightMeters);

                            // if targetingData.rpm is 0, the solver couldn't find a solution
                            // (physically impossible)
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
                .withName("Shoot With Max Height: " + maxHeightMeters + "m");
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
