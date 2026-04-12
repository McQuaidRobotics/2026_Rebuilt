package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.FieldVisualizer;
import igknighters.Robot;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.util.LerpTable;
import igknighters.util.LerpTable.LerpTableEntry;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;
import igknighters.util.log.Log;
import org.littletonrobotics.junction.Logger;

public class AimSolver {

    public static class Solvers {

        public static TunableDouble efficiencyConst =
                TunableValues.getDouble("Shooter/EfficiencyConst", 2.1);

        static LerpTable maxHeightDistanceLerp =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1, 2.5),
                            new LerpTableEntry(2, 3.0),
                            new LerpTableEntry(3, 3.2),
                            new LerpTableEntry(4, 3.4),
                            new LerpTableEntry(5, 3.6),
                            new LerpTableEntry(6, 3.9),
                            new LerpTableEntry(10, 5.0)
                        });

        static LerpTable airResistanceDTodistancedivider =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1.0, 2.4),
                            new LerpTableEntry(3.0, 2.0),
                            new LerpTableEntry(3.6, 1.8),
                            new LerpTableEntry(4.0, 1.9),
                            new LerpTableEntry(5.0, 2.2),
                            new LerpTableEntry(6, 2.25),
                            new LerpTableEntry(7, 2.25),
                            new LerpTableEntry(10.0, 2.2),
                            new LerpTableEntry(15.0, 1.5),
                            new LerpTableEntry(20.0, 1.485),
                        });

        static Mechanism2d canSHOOTMECH = new Mechanism2d(20, 20);
        static boolean hasBeenAdded = false;

        public static void publishOnce() {
            if (!hasBeenAdded) {
                SmartDashboard.putData("Visualizers/Shooter/CAN SHOOT", canSHOOTMECH);
            }
            hasBeenAdded = true;
        }

        public static void canShoot(boolean canShoot) {
            publishOnce();
            if (canShoot) {
                canSHOOTMECH.setBackgroundColor(new Color8Bit(Color.kGreen));
            } else {
                canSHOOTMECH.setBackgroundColor(new Color8Bit(Color.kRed));
            }
        }

        public static ShooterState solve_max_height_iterative(
                Pose3d shooterPose,
                Pose3d targetPose,
                ChassisSpeeds speeds,
                double currentRPM,
                double maxHeightMeters,
                double periodTime) {

            Pose3d predictedShooterPose = Robot.turret_pred.getPredictedPose().get();
            double sx = predictedShooterPose.getX();
            double sy = predictedShooterPose.getY();
            double sz = predictedShooterPose.getZ();

            ChassisSpeeds predSpeeds = Robot.turret_pred.getPredictedVelos().get();

            double estimatedToF = 1.5;

            Translation2d airResistanceAdder =
                    addDToTargetWithAirResistance(targetPose, shooterPose);
            double tx =
                    targetPose.getX()
                            + airResistanceAdder.getX()
                            - (predSpeeds.vxMetersPerSecond * (estimatedToF));
            double ty =
                    targetPose.getY()
                            + airResistanceAdder.getY()
                            - (predSpeeds.vyMetersPerSecond * (estimatedToF));
            double tz = targetPose.getZ();

            FieldVisualizer.getInstance()
                    .updateShootingTarget(new Pose2d(tx, ty, new Rotation2d()));

            double dx = tx - sx;
            double dy = ty - sy;
            double floorDistance = Math.hypot(dx, dy);

            double bestRPM = 0.0;
            double bestThetaHoodDegrees = 0.0;
            double bestV = 0.0;
            double minRPMDiff = Double.MAX_VALUE;

            for (int i = 0; i < 3; i++) {
                double currentCeilingHeight = tz + 2.0 + (i * (maxHeightMeters - (tz + 2.0)) / 3.0);

                double hRise = currentCeilingHeight - sz;
                double hFall = currentCeilingHeight - tz;

                if (hRise < 0 || hFall < 0) continue;

                double tRise = Math.sqrt(2.0 * hRise / 9.81);
                double tFall = Math.sqrt(2.0 * hFall / 9.81);
                double totalTime = tRise + tFall;

                double vx_planar = floorDistance / totalTime;
                double vz_initial = 9.81 * tRise;
                double v_total = Math.hypot(vx_planar, vz_initial);

                double RPM =
                        (v_total
                                        / (2
                                                * Math.PI
                                                * Robot.consts
                                                        .shooter()
                                                        .kFlywheels()
                                                        .WHEEL_RADIUS_METERS()))
                                * 60
                                * efficiencyConst.value();

                double launchAngleDegrees = Math.toDegrees(Math.atan2(vz_initial, vx_planar));
                double hoodAngleDegrees = 90.0 - launchAngleDegrees;

                if (hoodAngleDegrees < kHood.MIN_ANGLE_DEGREES
                        || hoodAngleDegrees > kHood.MAX_ANGLE_DEGREES) continue;

                double dRPM = Math.abs(currentRPM - RPM);
                if (dRPM < minRPMDiff) {
                    minRPMDiff = dRPM;
                    bestRPM = RPM;
                    bestThetaHoodDegrees = hoodAngleDegrees;
                    bestV = v_total;
                }
            }

            double absoluteFieldAngle = Math.atan2(dy, dx);
            double robotYawFuture = predictedShooterPose.getRotation().getZ();
            double turretAngle =
                    Math.atan2(
                            Math.sin(absoluteFieldAngle - robotYawFuture),
                            Math.cos(absoluteFieldAngle - robotYawFuture));

            if (bestRPM != 0 && bestThetaHoodDegrees != 0) {
                canShoot(true);
                ShootInformation.getInstance().setPossibleShot(true);
                publishShotTrajectory(
                        bestV,
                        Math.toRadians(90 - bestThetaHoodDegrees),
                        absoluteFieldAngle,
                        shooterPose,
                        targetPose);
            } else {
                canShoot(false);
                ShootInformation.getInstance().setPossibleShot(false);
            }

            return new ShooterState(
                    RPM.of(bestRPM), Radians.of(-turretAngle), Degrees.of(bestThetaHoodDegrees));
        }

        public static ShooterState solve_max_and_min_iterative(
                Pose3d shooterPose,
                Pose3d targetPose,
                ChassisSpeeds speeds,
                double currentRPM,
                double maxHeightMeters,
                double minHeightMeters,
                double periodTime) {
            Pose3d predictedShooterPose = Robot.turret_pred.getPredictedPose().get();
            FieldVisualizer.getInstance().updatePredictedPose(predictedShooterPose.toPose2d());
            double sx = predictedShooterPose.getX();
            double sy = predictedShooterPose.getY();
            double sz = predictedShooterPose.getZ();
            ChassisSpeeds predSpeeds = Robot.turret_pred.getPredictedVelos().get();

            double initialDist =
                    predictedShooterPose.getTranslation().getDistance(targetPose.getTranslation());
            Log.log("ROBOT/Commands/AimSolver/Distance", initialDist);
            double estimatedToF = 1.5;

            Translation2d airResistanceAdder =
                    addDToTargetWithAirResistance(targetPose, shooterPose);
            double tx =
                    targetPose.getX()
                            + airResistanceAdder.getX()
                            - (predSpeeds.vxMetersPerSecond * (estimatedToF));
            double ty =
                    targetPose.getY()
                            + airResistanceAdder.getY()
                            - (predSpeeds.vyMetersPerSecond * (estimatedToF));
            double tz = targetPose.getZ();

            FieldVisualizer.getInstance()
                    .updateShootingTarget(new Pose2d(tx, ty, new Rotation2d()));

            double dx = tx - sx;
            double dy = ty - sy;
            double floorDistance = Math.hypot(dx, dy);

            double bestRPM = 0.0;
            double bestThetaHoodDegrees = kHood.MIN_ANGLE_DEGREES;
            double bestV = 0.0;
            double highestArc = 0.0;

            for (int i = 0; i < 5; i++) {
                double currentCeilingHeight =
                        minHeightMeters + (i * (maxHeightMeters - minHeightMeters) / 5.0);

                double hRise = currentCeilingHeight - sz;
                double hFall = currentCeilingHeight - tz;

                if (hRise < 0 || hFall < 0) continue;

                double tRise = Math.sqrt(2.0 * hRise / 9.81);
                double tFall = Math.sqrt(2.0 * hFall / 9.81);
                double totalTime = tRise + tFall;

                double vx_planar = floorDistance / totalTime;
                double vz_initial = 9.81 * tRise;
                double v_total = Math.hypot(vx_planar, vz_initial);

                double RPM =
                        (v_total
                                        / (2
                                                * Math.PI
                                                * Robot.consts
                                                        .shooter()
                                                        .kFlywheels()
                                                        .WHEEL_RADIUS_METERS()))
                                * 60
                                * efficiencyConst.value();

                double launchAngleDegrees = Math.toDegrees(Math.atan2(vz_initial, vx_planar));
                double hoodAngleDegrees = 90.0 - launchAngleDegrees;

                if (hoodAngleDegrees < kHood.MIN_ANGLE_DEGREES
                        || hoodAngleDegrees > kHood.MAX_ANGLE_DEGREES) continue;

                double arc = launchAngleDegrees;
                if (arc > highestArc) {
                    highestArc = arc;
                    bestRPM = RPM;
                    bestThetaHoodDegrees = hoodAngleDegrees;
                    bestV = v_total;
                }
            }

            double absoluteFieldAngle = Math.atan2(dy, dx);
            double robotYawFuture = predictedShooterPose.getRotation().getZ();
            double turretAngle =
                    Math.atan2(
                            Math.sin(absoluteFieldAngle - robotYawFuture),
                            Math.cos(absoluteFieldAngle - robotYawFuture));

            if (bestRPM != 0 && bestThetaHoodDegrees != 0) {
                canShoot(true);
                ShootInformation.getInstance().setPossibleShot(true);
                publishShotTrajectory(
                        bestV,
                        Math.toRadians(90 - bestThetaHoodDegrees),
                        absoluteFieldAngle,
                        shooterPose,
                        targetPose);
            } else {
                canShoot(false);
                ShootInformation.getInstance().setPossibleShot(false);
            }

            return new ShooterState(
                    RPM.of(bestRPM), Radians.of(-turretAngle), Degrees.of(bestThetaHoodDegrees));
        }

        public static ShooterState solve_max_and_min_iterative_with_vectors(
                Pose3d shooterPose,
                Pose3d targetPose,
                ChassisSpeeds speeds,
                double currentRPM,
                double maxHeightMeters,
                double minHeightMeters,
                double periodTime) {

            Pose3d predictedShooterPose = Robot.turret_pred.getPredictedPose().get();
            FieldVisualizer.getInstance().updatePredictedPose(predictedShooterPose.toPose2d());
            ChassisSpeeds predSpeeds = Robot.turret_pred.getPredictedVelos().get();

            double sx = predictedShooterPose.getX();
            double sy = predictedShooterPose.getY();
            double sz = predictedShooterPose.getZ();

            Translation2d airResistanceAdder =
                    addDToTargetWithAirResistance(targetPose, shooterPose);
            double tx = targetPose.getX() + airResistanceAdder.getX();
            double ty = targetPose.getY() + airResistanceAdder.getY();
            double tz = targetPose.getZ();

            FieldVisualizer.getInstance()
                    .updateShootingTarget(new Pose2d(tx, ty, new Rotation2d()));

            double dx = tx - sx;
            double dy = ty - sy;
            double floorDistance = Math.hypot(dx, dy);

            double bestV_shooter = 0.0;
            double bestThetaHoodDegrees = 0.0;
            double bestTurretFieldAngle = 0.0;
            double highestArc = -1.0;
            boolean foundValidShot = false;

            for (int i = 0; i < 5; i++) {
                double currentCeilingHeight =
                        minHeightMeters + (i * (maxHeightMeters - minHeightMeters) / 4.0);

                double hRise = currentCeilingHeight - sz;
                double hFall = currentCeilingHeight - tz;

                if (hRise <= 0 || hFall <= 0) continue;

                double tRise = Math.sqrt(2.0 * hRise / 9.81);
                double tFall = Math.sqrt(2.0 * hFall / 9.81);
                double totalTime = tRise + tFall;

                double vx_field = dx / totalTime;
                double vy_field = dy / totalTime;
                double vz_field = 9.81 * tRise;

                double vx_shooter = vx_field - speeds.vxMetersPerSecond;
                double vy_shooter = vy_field - speeds.vyMetersPerSecond;
                double vz_shooter = vz_field;

                double v_planar_shooter = Math.hypot(vx_shooter, vy_shooter);
                double total_v_shooter = Math.hypot(v_planar_shooter, vz_shooter);

                double launchAngleDegrees =
                        Math.toDegrees(Math.atan2(vz_shooter, v_planar_shooter));
                double hoodAngleDegrees = 90.0 - launchAngleDegrees;

                if (hoodAngleDegrees < kHood.MIN_ANGLE_DEGREES
                        || hoodAngleDegrees > kHood.MAX_ANGLE_DEGREES) {
                    continue;
                }

                if (launchAngleDegrees > highestArc) {
                    highestArc = launchAngleDegrees;
                    bestV_shooter = total_v_shooter;
                    bestThetaHoodDegrees = hoodAngleDegrees;
                    bestTurretFieldAngle = Math.atan2(vy_shooter, vx_shooter);
                    foundValidShot = true;
                }
            }

            double shooterRPM = 0.0;
            double turretAngle = 0.0;

            if (foundValidShot) {
                shooterRPM =
                        (bestV_shooter
                                        / (2
                                                * Math.PI
                                                * Robot.consts
                                                        .shooter()
                                                        .kFlywheels()
                                                        .WHEEL_RADIUS_METERS()))
                                * 60
                                * efficiencyConst.value();

                double robotYaw = predictedShooterPose.getRotation().getZ();
                turretAngle =
                        Math.atan2(
                                Math.sin(bestTurretFieldAngle - robotYaw),
                                Math.cos(bestTurretFieldAngle - robotYaw));

                canShoot(true);
                ShootInformation.getInstance().setPossibleShot(true);
                publishShotTrajectory(
                        bestV_shooter,
                        Math.toRadians(90 - bestThetaHoodDegrees),
                        bestTurretFieldAngle,
                        shooterPose,
                        targetPose);
            } else {
                canShoot(false);
                ShootInformation.getInstance().setPossibleShot(false);
            }

            return new ShooterState(
                    RPM.of(shooterRPM), Radians.of(-turretAngle), Degrees.of(bestThetaHoodDegrees));
        }

        public static ShooterState solve_max_and_min_iterative_with_vectors_VARIED_MAX_HEIGHT(
                Pose3d shooterPose,
                Pose3d targetPose,
                ChassisSpeeds speeds,
                double currentRPM,
                double maxHeightMeters,
                double minHeightMeters,
                double periodTime) {

            Pose3d predictedShooterPose = Robot.turret_pred.getPredictedPose().get();
            FieldVisualizer.getInstance().updatePredictedPose(predictedShooterPose.toPose2d());
            ChassisSpeeds predSpeeds = Robot.turret_pred.getPredictedVelos().get();

            double sx = predictedShooterPose.getX();
            double sy = predictedShooterPose.getY();
            double sz = predictedShooterPose.getZ();

            Translation2d airResistanceAdder =
                    addDToTargetWithAirResistance(targetPose, shooterPose);
            double tx = targetPose.getX() + airResistanceAdder.getX();
            double ty = targetPose.getY() + airResistanceAdder.getY();
            double tz = targetPose.getZ();

            FieldVisualizer.getInstance()
                    .updateShootingTarget(new Pose2d(tx, ty, new Rotation2d()));

            double dx = tx - sx;
            double dy = ty - sy;
            double floorDistance = Math.hypot(dx, dy);

            double bestV_shooter = 0.0;
            double bestThetaHoodDegrees = 0.0;
            double bestTurretFieldAngle = 0.0;
            double highestArc = -1.0;
            boolean foundValidShot = false;

            double maxHeight = maxHeightDistanceLerp.lerp(floorDistance);
            for (int i = 0; i < 5; i++) {
                double currentCeilingHeight = 1.0 + (i * (maxHeight - 1.0) / 4.0);

                double hRise = currentCeilingHeight - sz;
                double hFall = currentCeilingHeight - tz;

                if (hRise <= 0 || hFall <= 0) continue;

                double tRise = Math.sqrt(2.0 * hRise / 9.81);
                double tFall = Math.sqrt(2.0 * hFall / 9.81);
                double totalTime = tRise + tFall;

                double vx_field = dx / totalTime;
                double vy_field = dy / totalTime;
                double vz_field = 9.81 * tRise;

                double vx_shooter = vx_field - predSpeeds.vxMetersPerSecond;
                double vy_shooter = vy_field - predSpeeds.vyMetersPerSecond;
                double vz_shooter = vz_field;

                double v_planar_shooter = Math.hypot(vx_shooter, vy_shooter);
                double total_v_shooter = Math.hypot(v_planar_shooter, vz_shooter);

                double launchAngleDegrees =
                        Math.toDegrees(Math.atan2(vz_shooter, v_planar_shooter));
                double hoodAngleDegrees = 90.0 - launchAngleDegrees;

                if (hoodAngleDegrees < kHood.MIN_ANGLE_DEGREES
                        || hoodAngleDegrees > kHood.MAX_ANGLE_DEGREES) {
                    continue;
                }

                if (launchAngleDegrees > highestArc) {
                    highestArc = launchAngleDegrees;
                    bestV_shooter = total_v_shooter;
                    bestThetaHoodDegrees = hoodAngleDegrees;
                    bestTurretFieldAngle = Math.atan2(vy_shooter, vx_shooter);
                    foundValidShot = true;
                }
            }

            double shooterRPM = 0.0;
            double turretAngle = 0.0;

            if (foundValidShot) {
                shooterRPM =
                        (bestV_shooter
                                        / (2
                                                * Math.PI
                                                * Robot.consts
                                                        .shooter()
                                                        .kFlywheels()
                                                        .WHEEL_RADIUS_METERS()))
                                * 60
                                * efficiencyConst.value();

                double robotYaw = predictedShooterPose.getRotation().getZ();
                turretAngle =
                        Math.atan2(
                                Math.sin(bestTurretFieldAngle - robotYaw),
                                Math.cos(bestTurretFieldAngle - robotYaw));

                canShoot(true);
                ShootInformation.getInstance().setPossibleShot(true);
                publishShotTrajectory(
                        bestV_shooter,
                        Math.toRadians(90 - bestThetaHoodDegrees),
                        bestTurretFieldAngle,
                        shooterPose,
                        targetPose);
            } else {
                canShoot(false);
                ShootInformation.getInstance().setPossibleShot(false);
            }

            return new ShooterState(
                    RPM.of(shooterRPM), Radians.of(-turretAngle), Degrees.of(bestThetaHoodDegrees));
        }

        public static Translation2d addDToTargetWithAirResistance(
                Pose3d targetPose, Pose3d shooterPose, double launchAngleRads) {

            double d = shooterPose.getTranslation().getDistance(targetPose.getTranslation());

            double cosAngle = Math.cos(launchAngleRads);
            double arcLengthApproximation = d / Math.max(cosAngle, 0.5);

            double angleToTarget =
                    Math.atan2(
                            targetPose.getY() - shooterPose.getY(),
                            targetPose.getX() - shooterPose.getX());

            double dragComp = arcLengthApproximation * arcLengthApproximation * 0.015;

            double px = Math.cos(angleToTarget) * dragComp;
            double py = Math.sin(angleToTarget) * dragComp;

            return new Translation2d(px, py);
        }

        public static Translation2d addDToTargetWithAirResistance(
                Pose3d targetPose, Pose3d shooterPose) {
            double d = shooterPose.getTranslation().getDistance(targetPose.getTranslation());

            Log.log("ROBOT/Commands/AimSolver/Distance", d);

            double angleToTarget =
                    Math.atan2(
                            targetPose.getY() - shooterPose.getY(),
                            targetPose.getX() - shooterPose.getX());
            double p = d / airResistanceDTodistancedivider.lerp(d);

            double px = Math.cos(angleToTarget) * p;
            double py = Math.sin(angleToTarget) * p;

            return new Translation2d(px, py);
        }

        public static double getShotTime(
                double ballLaunchVelocity,
                double hoodAngleRadians,
                double shooterHeight,
                double targetHeight) {
            double vY = ballLaunchVelocity * Math.sin(hoodAngleRadians);

            double a = 0.5 * 9.81;
            double b = -vY;
            double c = targetHeight - shooterHeight;

            double discriminant = b * b - 4 * a * c;

            if (discriminant < 0) {
                return Double.POSITIVE_INFINITY;
            }

            double time = (-b + Math.sqrt(discriminant)) / (2 * a);
            return time;
        }

        public static void clearShotTrajectory() {
            Logger.recordOutput("Shooter/ShotTrajectory", new Translation2d[] {});
        }

        public static void publishShotTrajectory(
                double ballLaunchVelocity,
                double launchAngleRads,
                double fieldShotAngle,
                Pose3d shooterPose3d,
                Pose3d targetPose3d) {

            double sx = shooterPose3d.getX();
            double sy = shooterPose3d.getY();
            double sz = shooterPose3d.getZ();

            double vZ = ballLaunchVelocity * Math.sin(launchAngleRads);
            double vH = ballLaunchVelocity * Math.cos(launchAngleRads);

            double vx = vH * Math.cos(fieldShotAngle);
            double vy = vH * Math.sin(fieldShotAngle);

            double time = getShotTime(ballLaunchVelocity, launchAngleRads, sz, targetPose3d.getZ());
            if (Double.isInfinite(time) || time <= 0) time = 1.5;

            int nPoints = 25;
            Pose3d[] trajectoryPoints = new Pose3d[nPoints];

            for (int i = 0; i < nPoints; i++) {
                double t = (time / (nPoints - 1)) * i;

                double x = sx + vx * t;
                double y = sy + vy * t;
                double z = sz + (vZ * t) - (0.5 * 9.81 * t * t);

                trajectoryPoints[i] = new Pose3d(x, y, Math.max(0, z), new Rotation3d());
            }

            Logger.recordOutput("Shooter/ShotTrajectory", trajectoryPoints);
        }
    }
}
