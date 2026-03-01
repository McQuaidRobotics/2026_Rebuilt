package igknighters.subsystems.shooter.solvers.Math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.subsystems.shooter.solvers.*;
import java.util.function.Supplier;

public class MaxHeightSolver extends Solver {

    @Override
    public ShooterState solve(
            Supplier<Pose3d> shooterPose,
            Supplier<Pose3d> targetPose,
            Supplier<ChassisSpeeds> chassisSpeeds,
            double currentRPM,
            double maxHeightMeters) {
        // 1. Position and Target setup
        double sx = shooterPose.get().getX() + chassisSpeeds.get().vxMetersPerSecond * 0.02;
        double sy = shooterPose.get().getY() + chassisSpeeds.get().vyMetersPerSecond * 0.02;
        double sz = shooterPose.get().getZ();

        double initialDist =
                shooterPose.get().getTranslation().getDistance(targetPose.get().getTranslation());
        double estimatedToF = initialDist / 5.0; // Assume 5m/s avg horizontal velocity

        // 3. TARGET PROJECTION: Scale the target lead by (ToF + Latency)
        // We subtract the robot's velocity because from the ball's perspective,
        // the target is moving toward/away at the robot's speed.
        Translation2d airResistanceAdder =
                super.addDToTargetWithAirResistance(targetPose.get(), shooterPose.get());
        double tx =
                targetPose.get().getX()
                        + airResistanceAdder.getX()
                        - (chassisSpeeds.get().vxMetersPerSecond * (estimatedToF + 0.02));
        double ty =
                targetPose.get().getY()
                        + airResistanceAdder.getY()
                        - (chassisSpeeds.get().vyMetersPerSecond * (estimatedToF + 0.02));
        double tz = targetPose.get().getZ();

        double dx = tx - sx;
        double dy = ty - sy;
        double floorDistance = Math.hypot(dx, dy); // Total horizontal distance

        double bestRPM = 0.0;
        double bestThetaHoodDegrees = SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES;
        double bestV = 0.0;
        double minRPMDiff = Double.MAX_VALUE;

        // 2. Iterative Arc Search
        for (int i = 0; i < 10; i++) {
            // Search heights between target + 1m and max ceiling
            double currentCeilingHeight = tz + 2.0 + (i * (maxHeightMeters - (tz + 2.0)) / 10.0);

            double hRise = currentCeilingHeight - sz;
            double hFall = currentCeilingHeight - tz;

            if (hRise < 0 || hFall < 0) continue;

            double tRise = Math.sqrt(2.0 * hRise / 9.81);
            double tFall = Math.sqrt(2.0 * hFall / 9.81);
            double totalTime = tRise + tFall;

            double vx_planar = floorDistance / totalTime;
            double vz_initial = 9.81 * tRise; // Velocity needed to reach peak
            double v_total = Math.hypot(vx_planar, vz_initial);

            // Single-sided flywheel: Wheel surface speed = 2x Ball speed
            double RPM =
                    (v_total
                                    / (2
                                            * Math.PI
                                            * SubsystemConstants.kShooter
                                                    .kFlywheels
                                                    .WHEEL_RADIUS_METERS))
                            * 60
                            * 2.0;

            double launchAngleDegrees = Math.toDegrees(Math.atan2(vz_initial, vx_planar));
            double hoodAngleDegrees = 90.0 - launchAngleDegrees;

            if (hoodAngleDegrees < SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES
                    || hoodAngleDegrees > SubsystemConstants.kShooter.kHood.MAX_ANGLE_DEGREES)
                continue;

            // Choose the shot closest to our current flywheel speed for faster spin-up
            double dRPM = Math.abs(currentRPM - RPM);
            if (dRPM < minRPMDiff) {
                minRPMDiff = dRPM;
                bestRPM = RPM;
                bestThetaHoodDegrees = hoodAngleDegrees;
                bestV = v_total;
            }
        }

        // 3. Final Angles
        double absoluteFieldAngle = Math.atan2(dy, dx);
        double robotYawFuture =
                shooterPose.get().getRotation().getZ()
                        + chassisSpeeds.get().omegaRadiansPerSecond * 0.02;
        double turretAngle =
                Math.atan2(
                        Math.sin(absoluteFieldAngle - robotYawFuture),
                        Math.cos(absoluteFieldAngle - robotYawFuture));

        if (bestRPM != 0) {
            canShoot(true);
            // We pass absoluteFieldAngle so the trajectory line points at the target
            publishShotTrajectory(
                    bestV,
                    Math.toRadians(90 - bestThetaHoodDegrees),
                    absoluteFieldAngle,
                    shooterPose.get(),
                    targetPose.get());
        } else {
            canShoot(false);
        }

        return new ShooterState(
                RPM.of(bestRPM), Radians.of(turretAngle), Degrees.of(bestThetaHoodDegrees));
    }
}
