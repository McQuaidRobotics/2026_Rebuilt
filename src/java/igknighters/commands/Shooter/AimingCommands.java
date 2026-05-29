package igknighters.commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.ShootInformation;
import igknighters.subsystems.YamShooter.Shooter;
import igknighters.subsystems.YamShooter.Shooter.shotType;
import igknighters.subsystems.YamShooter.ShooterState;
import igknighters.subsystems.YamShooter.ShootingData;
import igknighters.subsystems.YamShooter.solvers.Math.LerpSolveShot;
import java.util.function.Supplier;

public class AimingCommands {

    public static Supplier<Pose2d> getShooterPoseWithOffset(Supplier<Pose2d> robotPose) {
        return () -> robotPose.get();
    }

    public static boolean isBetween(Pose2d pose, double a, double b) {
        double x = pose.getX();
        return x >= a && x <= b;
    }

    public static Pose2d getTurretPose() {
        return getShooterPoseWithOffset(() -> Robot.pose_pred.getDynamicPredictedPose()).get();
    }

    public static Command idleShooter(Shooter shooter) {

        ShootInformation info = ShootInformation.getInstance();
        return Commands.run(
                        () -> {
                            info.setBeingControlled(false);
                            Pose3d targetPose = info.getShotLocation();

                            ShooterState targetingData =
                                    LerpSolveShot.solve(
                                            targetPose,
                                            shooter.getCurrentState().flywheelSpeed.in(RPM),
                                            0.0);

                            shooter.targetState(
                                    RPM.of(3000),
                                    targetingData.turretAngle,
                                    Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()));
                        },
                        shooter.hood,
                        shooter.flywheels,
                        shooter.turret)
                .withName("IDLE ENTIRE SHOOTER : NOT DEFAULT COMMAND");
    }

    public static shotType getShotType() {
        ShootInformation info = ShootInformation.getInstance();
        if (info.shouldPass()) {
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
    public static void idleOnce(Shooter shooter) {
        ShootInformation info = ShootInformation.getInstance();
        ShootingData shootingData = info.getData();
        info.setBeingControlled(false);

        ShooterState targetingData =
                LerpSolveShot.solve(
                        shootingData.TARGET_POSE,
                        shooter.getCurrentState().flywheelSpeed.in(RPM),
                        0.02);

        shooter.targetState(
                RPM.of(3000),
                targetingData.turretAngle,
                Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()));
    }

    public static void shootOnce(Shooter shooter) {

        ShootInformation info = ShootInformation.getInstance();
        info.setBeingControlled(true);
        ShootingData shootingData = info.getData();

        shooter.currentShotType = getShotType();

        ShooterState targetingData = LerpSolveShot.solve(shootingData.TARGET_POSE, 0.1, 0.0);

        if (targetingData.flywheelSpeed.in(RPM) != 0) {
            // possible shot so follow its instructions
            shooter.targetState(targetingData);
        } else {
            // shot is impossible so we should idle the shooter rpm at like 4000
            shooter.targetState(
                    RPM.of(3200),
                    targetingData.turretAngle,
                    Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()));
        }
    }

    public static double maxHeightMeters = 4.8;

    public static Command shootWithProtectionAndAgregiousMaxHeight(Shooter shooter) {
        return Commands.run(
                        () -> {
                            Boolean underTrenchCheck = DrivingSharedState.getInstance().underTrench;
                            if (underTrenchCheck) {
                                idleOnce(shooter);
                            } else {
                                shootOnce(shooter);
                            }
                        },
                        shooter.turret,
                        shooter.hood,
                        shooter.flywheels)
                .withName("SHOOT WITH PROTECTION AND AGRETIOUS MAX HEIGHT");
    }

    public static Command shootWithProtection(Shooter shooter) {

        return Commands.run(
                        () -> {
                            Boolean underTrenchCheck = DrivingSharedState.getInstance().underTrench;
                            if (underTrenchCheck) {
                                idleOnce(shooter);
                            } else {
                                shootOnce(shooter);
                            }
                        },
                        shooter.turret,
                        shooter.hood,
                        shooter.flywheels)
                .withName("SHOOT WITH PROTECTION");
    }

    public static Command SHOOT_AT_TARGET(Shooter shooter, Pose3d targetPose) {
        return Commands.run(
                        () -> {
                            ShooterState targetingData =
                                    LerpSolveShot.solve(
                                            targetPose,
                                            shooter.getCurrentState().flywheelSpeed.in(RPM),
                                            0.02);

                            if (targetingData.flywheelSpeed.in(RPM) != 0) {
                                shooter.targetState(targetingData);
                            } else {
                                // shot is imposible so we should idle the shooter rpm at like 4000
                                // so it
                                // spins up faster
                                shooter.targetState(
                                        RPM.of(3000),
                                        targetingData.turretAngle,
                                        Degrees.of(
                                                Robot.consts
                                                        .shooter()
                                                        .kHood()
                                                        .MIN_ANGLE_DEGREES()));
                            }
                        },
                        shooter.turret,
                        shooter.hood,
                        shooter.flywheels)
                .withName("SHOOT AT TARGET");
    }
}
