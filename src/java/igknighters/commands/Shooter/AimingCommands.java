package igknighters.commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.commands.Shooter.ShooterCommands.shotType;
import igknighters.constants.Conv;
import igknighters.constants.FieldConstants;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.shooter.AimSolver;
import igknighters.subsystems.shooter.Shooter;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.subsystems.shooter.ShootingData;
import igknighters.subsystems.shooter.solvers.Math.LerpSolveShot;
import igknighters.util.log.Log;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AimingCommands {

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

    public static boolean isBetween(Pose2d pose, double a, double b) {
        double x = pose.getX();
        return x >= a && x <= b;
    }

    public static BooleanSupplier isUnderTrench(Supplier<Pose2d> robotPoseSupplier) {
        Supplier<Pose2d> turretPose = getShooterPoseWithOffset(robotPoseSupplier);
        return () -> {
            boolean under1 =
                    isBetween(
                            turretPose.get(),
                            FieldConstants.BUMP.BUMP_1_X_METERS - .08,
                            FieldConstants.BUMP.BUMP_1_X_METERS + .08);
            boolean under2 =
                    isBetween(
                            turretPose.get(),
                            FieldConstants.BUMP.BUMP_2_X_METERS - 0.08,
                            FieldConstants.BUMP.BUMP_2_X_METERS + 0.08);

            boolean isUnder = under1 || under2;
            if (!SubsystemConstants.disableAllLogs) {
                Log.log("ROBOT/Commands/Shooter/Trench Protection/isUnderTrench", isUnder);
                Log.log("ROBOT/Commands/Shooter/Trench Protection/RobotX", turretPose.get().getX());
            }

            return isUnder;
        };
    }

    public static shotType getShotType(Supplier<Pose2d> robotPoseSupplier) {
        ShootInformation info = ShootInformation.getInstance();
        if (info.shouldPass(robotPoseSupplier)) {
            return shotType.PASS;
        } else {
            return shotType.SHOT;
        }
    }

    /**
     * Idles the shooter once. Intended for trench protection
     *
     * @param shooter
     * @param robotPoseSupplier
     * @param robotVelocitySupplier
     */
    public static void idleOnce(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {
        ShootInformation info = ShootInformation.getInstance();

        Pose2d robotPose2d = robotPoseSupplier.get();
        ShootingData shootingData = info.getData(robotPoseSupplier);
        ChassisSpeeds robotVel = robotVelocitySupplier.get();

        Pose3d shooterPose =
                new Pose3d(
                        robotPose2d.getX(),
                        robotPose2d.getY(),
                        SubsystemConstants.kShooter.kFlywheels.ShooterHeightMeters,
                        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
        ShooterState targetingData =
                AimSolver.Solvers.solve_max_and_min_iterative_with_vectors(
                        shooterPose,
                        shootingData.TARGET_POSE,
                        robotVel,
                        shooter.getCurrentState().flywheelSpeed.in(RPM),
                        shootingData.MAX_HEIGHT_METERS,
                        shootingData.MIN_HEIGHT_METERS,
                        0.02);

        shooter.targetState(
                RPM.of(3000), targetingData.turretAngle, Degrees.of(kHood.MIN_ANGLE_DEGREES));
    }

    public static void shootOnce(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {

        ShootInformation info = ShootInformation.getInstance();
        Supplier<Pose2d> shooterPoseWithOffset = getShooterPoseWithOffset(robotPoseSupplier);
        Pose2d shooterPose2d = shooterPoseWithOffset.get();
        ShootingData shootingData = info.getData(shooterPoseWithOffset);
        ChassisSpeeds robotVel = robotVelocitySupplier.get();

        shooter.currentShotType = getShotType(shooterPoseWithOffset);

        Pose3d shooterPose3d =
                new Pose3d(
                        shooterPose2d.getX(),
                        shooterPose2d.getY(),
                        SubsystemConstants.kShooter.kFlywheels.ShooterHeightMeters,
                        new Rotation3d(0.0, 0.0, shooterPose2d.getRotation().getRadians()));

        ShooterState targetingData =
                LerpSolveShot.solve(shooterPose3d, shootingData.TARGET_POSE, 0.1, 0.0);

        if (targetingData.flywheelSpeed.in(RPM) != 0) {
            // possible shot so follow its instructions
            shooter.targetState(targetingData);
        } else {
            // shot is impossible so we should idle the shooter rpm at like 4000
            shooter.targetState(
                    RPM.of(4000), targetingData.turretAngle, Degrees.of(kHood.MIN_ANGLE_DEGREES));
        }
    }

    public static double maxHeightMeters = 4.8;

    public static Command shootWithProtectionAndAgregiousMaxHeight(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {
        ShootInformation info = ShootInformation.getInstance();
        BooleanSupplier underTrenchCheck = isUnderTrench(robotPoseSupplier);
        return shooter.run(
                () -> {
                    info.setBeingControlled(true);
                    if (underTrenchCheck.getAsBoolean()) {
                        idleOnce(shooter, robotPoseSupplier, robotVelocitySupplier);
                    } else {
                        shootOnce(shooter, robotPoseSupplier, robotVelocitySupplier);
                    }
                });
    }

    public static Command shootWithProtection(
            Shooter shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> robotVelocitySupplier) {
        ShootInformation info = ShootInformation.getInstance();
        BooleanSupplier underTrenchCheck = isUnderTrench(robotPoseSupplier);

        return shooter.run(
                () -> {
                    info.setBeingControlled(true);
                    if (underTrenchCheck.getAsBoolean()) {
                        idleOnce(shooter, robotPoseSupplier, robotVelocitySupplier);
                    } else {
                        shootOnce(shooter, robotPoseSupplier, robotVelocitySupplier);
                    }
                });
    }

    public static Command SHOOT_AT_TARGET(
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
                            LerpSolveShot.solve(
                                    shooterPose,
                                    targetPose,
                                    shooter.getCurrentState().flywheelSpeed.in(RPM),
                                    0.02);

                    if (targetingData.flywheelSpeed.in(RPM) != 0) {
                        shooter.targetState(targetingData);
                    } else {
                        // shot is imposible so we should idle the shooter rpm at like 4000 so it
                        // spins up faster
                        shooter.targetState(
                                RPM.of(3000),
                                targetingData.turretAngle,
                                Degrees.of(kHood.MIN_ANGLE_DEGREES));
                    }
                });
    }
}
