package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
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
import igknighters.constants.AbleToShootSharedState;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kFlywheels;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import org.littletonrobotics.junction.Logger;

public class AimSolver {

    public static class Solvers {
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
            // 1. Position and Target setup
            double sx = shooterPose.getX() + speeds.vxMetersPerSecond * periodTime;
            double sy = shooterPose.getY() + speeds.vyMetersPerSecond * periodTime;
            double sz = shooterPose.getZ();

            double initialDist =
                    shooterPose.getTranslation().getDistance(targetPose.getTranslation());
            double estimatedToF = initialDist / 5.0; // Assume 5m/s avg horizontal velocity

            // 3. TARGET PROJECTION: Scale the target lead by (ToF + Latency)
            // We subtract the robot's velocity because from the ball's perspective,
            // the target is moving toward/away at the robot's speed.
            Translation2d airResistanceAdder =
                    addDToTargetWithAirResistance(targetPose, shooterPose);
            double tx =
                    targetPose.getX()
                            + airResistanceAdder.getX()
                            - (speeds.vxMetersPerSecond * (estimatedToF + periodTime));
            double ty =
                    targetPose.getY()
                            + airResistanceAdder.getY()
                            - (speeds.vyMetersPerSecond * (estimatedToF + periodTime));
            double tz = targetPose.getZ();

            FieldVisualizer.getInstance()
                    .updateShootingTarget(new Pose2d(tx, ty, new Rotation2d()));

            double dx = tx - sx;
            double dy = ty - sy;
            double floorDistance = Math.hypot(dx, dy); // Total horizontal distance

            double bestRPM = 0.0;
            double bestThetaHoodDegrees = 0.0;
            double bestV = 0.0;
            double minRPMDiff = Double.MAX_VALUE;

            // 2. Iterative Arc Search
            for (int i = 0; i < 10; i++) {
                // Search heights between target + 1m and max ceiling
                double currentCeilingHeight =
                        tz + 2.0 + (i * (maxHeightMeters - (tz + 2.0)) / 10.0);

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
                double RPM = (v_total / (2 * Math.PI * kFlywheels.WHEEL_RADIUS_METERS)) * 60 * 2.0;

                double launchAngleDegrees = Math.toDegrees(Math.atan2(vz_initial, vx_planar));
                double hoodAngleDegrees = 90.0 - launchAngleDegrees;

                if (hoodAngleDegrees < kHood.MIN_ANGLE_DEGREES
                        || hoodAngleDegrees > kHood.MAX_ANGLE_DEGREES) continue;

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
                    shooterPose.getRotation().getZ() + speeds.omegaRadiansPerSecond * periodTime;
            double turretAngle =
                    Math.atan2(
                            Math.sin(absoluteFieldAngle - robotYawFuture),
                            Math.cos(absoluteFieldAngle - robotYawFuture));

            if (bestRPM != 0 && bestThetaHoodDegrees != 0) {
                canShoot(true);
                // We pass absoluteFieldAngle so the trajectory line points at the target
                AbleToShootSharedState.getInstance().setPossibleShot(true);
                publishShotTrajectory(
                        bestV,
                        Math.toRadians(90 - bestThetaHoodDegrees),
                        absoluteFieldAngle,
                        shooterPose,
                        targetPose);
            } else {
                canShoot(false);
                AbleToShootSharedState.getInstance().setPossibleShot(false);
            }

            if (Robot.isBlue()) {
                return new ShooterState(
                        RPM.of(bestRPM),
                        Radians.of(-turretAngle),
                        Degrees.of(bestThetaHoodDegrees));
            } else {
                return new ShooterState(
                        RPM.of(bestRPM), Radians.of(turretAngle), Degrees.of(bestThetaHoodDegrees));
            }
        }

        private static final double FLYWHEEL_RADIUS =
                SubsystemConstants.kShooter.kFlywheels.WHEEL_RADIUS_METERS;

        // Gravity
        private static final double G = 9.81;

        /**
         * @return { turretAngleRadians, hoodAngleRadians, rpm } or null if the shot is impossible
         *     at this RPM
         */
        public static ShooterState solve_simple_no_AR_or_FutureTiming(
                Pose3d targetPose, Pose3d shooterPose, double currentRPM) {
            // Update Targeting Visualizer
            FieldVisualizer.getInstance().updateShootingTarget(targetPose.toPose2d());
            // --- Extract positions ---
            double sx = shooterPose.getX();
            double sy = shooterPose.getY();
            double sz = shooterPose.getZ();

            double tx = targetPose.getX();
            double ty = targetPose.getY();
            double tz = targetPose.getZ();

            // --- Differences ---
            double dx = tx - sx;
            double dy = ty - sy;
            double dz = tz - sz;

            // --- Turret angle (robot-relative) ---
            double absoluteAngle =
                    Math.atan2(dy, dx); // angle to target from robot to field in Field plane
            double robotYaw = shooterPose.getRotation().getZ(); // Rotation3d yaw field relative
            double turretAngleRADS =
                    absoluteAngle - robotYaw; // the angle the turret must turn to face target

            // Normalize to [-π, π]
            turretAngleRADS = Math.atan2(Math.sin(turretAngleRADS), Math.cos(turretAngleRADS));

            // --- Ballistic geometry ---
            double d = Math.sqrt(dx * dx + dy * dy); // horizontal distance
            double h = dz; // height difference

            // --- Convert RPM → launch velocity ---
            double omega = currentRPM * 2.0 * Math.PI / 60.0; // rad/s
            double v = (omega * FLYWHEEL_RADIUS) / 2.0; // m/s

            // --- Solve for hood angle (high arc) ---
            double inside = v * v * v * v - G * (G * d * d + 2 * h * v * v);

            if (inside < 0) {
                // Shot is physically impossible at this RPM
                DogLog.log(
                        "Subsystems/Shooter/Aiming/SHOT IS NOT POSSIBLE AT THIS RPM", currentRPM);
                canShoot(false);
                Logger.recordOutput(
                        "Shooter/ShotTrajectory",
                        new Pose3d[] {}); // Clear trajectory visualization
                return new ShooterState(
                        RPM.of(0.0),
                        Radians.of(turretAngleRADS),
                        Degrees.of(kHood.MIN_ANGLE_DEGREES));
            }
            canShoot(true);
            DogLog.log("Subsystems/Shooter/Aiming/SHOT IS POSSIBLE AT THIS RPM", currentRPM);

            double root = Math.sqrt(inside);

            double thetaLow =
                    Math.atan(
                            (v * v - root)
                                    / (G * d)); // this is the ball launch angle turretTheta is 90 -
            // theta if theta in degs
            double thetaHigh = Math.atan((v * v + root) / (G * d));
            DogLog.log("Subsystems/Shooter/Aiming/Theta Low (deg)", Math.toDegrees(thetaLow));
            DogLog.log("Subsystems/Shooter/Aiming/Theta High (deg)", Math.toDegrees(thetaHigh));
            DogLog.log("Subsystems/Shooter/Aiming/Distance to Target (m)", d);
            DogLog.log("Subsystems/Shooter/Aiming/Height to Target (m)", h);
            DogLog.log("Subsystems/Shooter/Aiming/Launch Velocity", v);

            // You want the HIGH arc
            double hoodAngle = Math.max(thetaLow, thetaHigh);
            double hoodSetpoint = Math.PI / 2 - hoodAngle;

            publishShotTrajectory(v, hoodAngle, turretAngleRADS, shooterPose, targetPose);

            return new ShooterState(
                    RPM.of(currentRPM), Radians.of(turretAngleRADS), Radians.of(hoodSetpoint));
        }

        public static double getSwerveVelocityProjection(double vx, double vy, double turretAngle) {
            return vx * Math.cos(turretAngle) + vy * Math.sin(turretAngle);
        }

        public static ShooterState solve_moving(
                Pose3d targetPose,
                Pose3d shooterPose,
                double currentRPM,
                ChassisSpeeds shooterVel, // vx, vy, omega
                double delaySeconds) {

            // update Targeting Visualizer
            FieldVisualizer.getInstance().updateShootingTarget(targetPose.toPose2d());

            // -----------------------------
            // 1. Predict shooter future pose
            // -----------------------------
            double vx = shooterVel.vxMetersPerSecond;
            double vy = shooterVel.vyMetersPerSecond;
            double omega = shooterVel.omegaRadiansPerSecond; // rad/s

            double sx = shooterPose.getX() + vx * delaySeconds;
            double sy = shooterPose.getY() + vy * delaySeconds;
            double sz = shooterPose.getZ();

            double tx = targetPose.getX();
            double ty = targetPose.getY();
            double tz = targetPose.getZ();

            // -----------------------------
            // 2. Compute horizontal vector to target
            // -----------------------------
            double dx = tx - sx;
            double dy = ty - sy;
            double dz = tz - sz;

            double absoluteAngle = Math.atan2(dy, dx); // field-relative

            // -----------------------------
            // 3. Predict robot future yaw
            // -----------------------------
            double robotYawNow = shooterPose.getRotation().getZ();
            double robotYawFuture = robotYawNow + omega * delaySeconds;

            // -----------------------------
            // 4. Compute turret angle (robot-relative)
            // -----------------------------
            double turretAngle = absoluteAngle - robotYawFuture;

            // Normalize
            turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle));

            // -----------------------------
            // 5. Horizontal distance + height
            // -----------------------------
            double d = Math.sqrt(dx * dx + dy * dy);
            double h = dz;

            // -----------------------------
            // 6. Compute launch velocity including swerve projection
            // -----------------------------
            double flywheelOmega = currentRPM * 2.0 * Math.PI / 60.0; // rad/s
            double vFlywheel =
                    flywheelOmega
                            * FLYWHEEL_RADIUS
                            / 2.0; // only one half of the flywheel is being moved by flywheel

            // Project robot velocity onto shot direction
            double shotDirX = Math.cos(turretAngle);
            double shotDirY = Math.sin(turretAngle);

            double vRobotProj = vx * shotDirX + vy * shotDirY;

            double v = vFlywheel + vRobotProj;

            // -----------------------------
            // 7. Ballistic discriminant
            // -----------------------------
            double inside = v * v * v * v - G * (G * d * d + 2 * h * v * v);

            DogLog.log("Subsystems/Shooter/Aiming/Distance", d);
            DogLog.log("Subsystems/Shooter/Aiming/Height", h);
            DogLog.log("Subsystems/Shooter/Aiming/Robot Velocity Projection", vRobotProj);
            DogLog.log("Subsystems/Shooter/Aiming/Flywheel Velocity", vFlywheel);
            DogLog.log("Subsystems/Shooter/Aiming/Launch Velocity", v);
            DogLog.log("Subsystems/Shooter/Aiming/RPM", currentRPM);
            DogLog.log("Subsystems/Shooter/Aiming/Ballistic Discriminant", inside);

            if (inside < 0) {
                DogLog.log(
                        "Subsystems/Shooter/Aiming/SHOT IS NOT POSSIBLE AT THIS RPM", currentRPM);
                canShoot(false);
                Logger.recordOutput(
                        "Shooter/ShotTrajectory",
                        new Pose3d[] {}); // Clear trajectory visualization
                return new ShooterState(
                        RPM.of(0.0), Radians.of(turretAngle), Degrees.of(kHood.MIN_ANGLE_DEGREES));
            }
            canShoot(true);

            DogLog.log("Subsystems/Shooter/Aiming/SHOT IS POSSIBLE AT THIS RPM", currentRPM);

            double root = Math.sqrt(inside);

            double thetaLow = Math.atan((v * v - root) / (G * d));
            double thetaHigh = Math.atan((v * v + root) / (G * d));

            DogLog.log("Subsystems/Shooter/Aiming/Theta Low", thetaLow);
            DogLog.log("Subsystems/Shooter/Aiming/Theta High", thetaHigh);
            DogLog.log("Subsystems/Shooter/Aiming/Predicted Turret Angle", turretAngle);
            // High arc
            double hoodAngle = Math.max(thetaLow, thetaHigh);

            // Convert to your mechanical hood reference
            double hoodSetpoint = Math.PI / 2 - hoodAngle;
            DogLog.log("Subsystems/Shooter/Aiming/Predicted Hood Angle", hoodSetpoint);

            // Pass the predicted pose so the trajectory starts from where the robot WILL be
            publishShotTrajectory(
                    v,
                    hoodAngle,
                    turretAngle,
                    new Pose3d(sx, sy, sz, new Rotation3d(0, 0, robotYawFuture)),
                    targetPose);
            return new ShooterState(
                    RPM.of(currentRPM), Radians.of(turretAngle), Radians.of(hoodSetpoint));
        }

        /**
         * Solves for shooter state using vector addition with field-relative robot velocity.
         * * @param robotVel Expected to be field-relative (vx, vy).
         */
        public static ShooterState solve_moving_vector_perfect(
                Pose3d targetPose,
                Pose3d shooterPose,
                double currentRPM,
                ChassisSpeeds robotVel,
                double delaySeconds) {

            // 1. Position Prediction
            double sx = shooterPose.getX() + (robotVel.vxMetersPerSecond * delaySeconds);
            double sy = shooterPose.getY() + (robotVel.vyMetersPerSecond * delaySeconds);
            double sz = shooterPose.getZ();

            double dx = targetPose.getX() - sx;
            double dy = targetPose.getY() - sy;
            double dz = targetPose.getZ() - sz;
            double d = Math.sqrt(dx * dx + dy * dy);
            double h = dz;

            // 2. Launch Velocity Calculation
            double omega = currentRPM * 2.0 * Math.PI / 60.0;

            // ADJUST THIS CONSTANT:
            // 1.0 if you have top and bottom wheels.
            // 0.5 if you have a hooded shooter (one wheel).
            double velocityMultiplier = 0.5;
            double vFlywheel = omega * FLYWHEEL_RADIUS * velocityMultiplier;

            // 3. Lateral Compensation
            double angleToTarget = Math.atan2(dy, dx);
            double vRobotLateral =
                    robotVel.vyMetersPerSecond * Math.cos(angleToTarget)
                            - robotVel.vxMetersPerSecond * Math.sin(angleToTarget);

            // Use a realistic horizontal velocity estimate for the turret lead
            double vFlywheelHorizGuess = vFlywheel * Math.cos(Math.toRadians(30));
            double turretOffset =
                    Math.asin(
                            Math.max(
                                    -1,
                                    Math.min(
                                            1,
                                            -vRobotLateral / Math.max(vFlywheelHorizGuess, 0.1))));
            double compensatedAbsoluteAngle = angleToTarget + turretOffset;

            double robotYawFuture =
                    shooterPose.getRotation().getZ()
                            + (robotVel.omegaRadiansPerSecond * delaySeconds);
            double turretAngle =
                    Math.atan2(
                            Math.sin(compensatedAbsoluteAngle - robotYawFuture),
                            Math.cos(compensatedAbsoluteAngle - robotYawFuture));

            // 4. Radial Robot Velocity
            double vRobotRadial =
                    robotVel.vxMetersPerSecond * Math.cos(compensatedAbsoluteAngle)
                            + robotVel.vyMetersPerSecond * Math.sin(compensatedAbsoluteAngle);

            // 5. Iterative Solver
            double currentGuessTheta = Math.toRadians(30.0);
            double finalTheta = currentGuessTheta;
            boolean possible = false;
            double v_eff = 0.0;

            DogLog.log("Subsystems/Shooter/Aiming/Distance", d);
            DogLog.log("Subsystems/Shooter/Aiming/Height", h);
            DogLog.log(
                    "Subsystems/Shooter/Aiming/TurretAngle", turretAngle * Conv.RADIANS_TO_DEGREES);
            DogLog.log("Subsystems/Shooter/Aiming/Robot Velocity Lateral", vRobotLateral);
            DogLog.log("Subsystems/Shooter/Aiming/Robot Velocity Radial", vRobotRadial);
            DogLog.log("Subsystems/Shooter/Aiming/Flywheel Velocity", vFlywheel);

            for (int i = 0; i < 8; i++) {
                double v_h = vFlywheel * Math.cos(currentGuessTheta) + vRobotRadial;
                double v_z = vFlywheel * Math.sin(currentGuessTheta);
                v_eff = Math.sqrt(v_h * v_h + v_z * v_z);

                // Ballistic trajectory formula discriminant
                double inside = Math.pow(v_eff, 4) - 9.81 * (9.81 * d * d + 2 * h * v_eff * v_eff);

                if (inside >= 0) {
                    double root = Math.sqrt(inside);
                    currentGuessTheta = Math.atan((v_eff * v_eff + root) / (9.81 * d));
                    finalTheta = currentGuessTheta;
                    possible = true;
                } else {
                    // If the loop finds it's impossible, it stops updating finalTheta
                    break;
                }
            }

            if (!possible) {
                canShoot(false);
                return new ShooterState(
                        RPM.of(0.0), Radians.of(turretAngle), Degrees.of(kHood.MIN_ANGLE_DEGREES));
            }

            double hoodSetpoint = Math.PI / 2 - finalTheta;

            if (hoodSetpoint < kHood.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_RADIANS
                    || hoodSetpoint > kHood.MAX_ANGLE_DEGREES * Conv.DEGREES_TO_RADIANS) {
                DogLog.log(
                        "Subsystems/Shooter/Aiming/Calculated hood angle out of bounds",
                        Math.toDegrees(hoodSetpoint));
                return new ShooterState(
                        RPM.of(0.0), Radians.of(turretAngle), Degrees.of(kHood.MIN_ANGLE_DEGREES));
            }

            canShoot(true);
            hoodSetpoint =
                    MathUtil.clamp(
                            hoodSetpoint,
                            SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES
                                    * Conv.DEGREES_TO_RADIANS,
                            SubsystemConstants.kShooter.kHood.MAX_ANGLE_DEGREES
                                    * Conv.DEGREES_TO_RADIANS);

            publishShotTrajectory(
                    v_eff, Math.PI / 2 - hoodSetpoint, turretAngle, shooterPose, targetPose);

            return new ShooterState(
                    RPM.of(currentRPM), Radians.of(turretAngle), Radians.of(hoodSetpoint));
        }

        public static Translation2d addDToTargetWithAirResistance(
                Pose3d targetPose, Pose3d shooterPose, double launchAngleRads) {

            double d = shooterPose.getTranslation().getDistance(targetPose.getTranslation());

            // Approximate Arc Length: If the shot is flat, length = d.
            // If it's a high arc, the path is significantly longer.
            // A simple trig approximation for path length: L = d / cos(launchAngle)
            double cosAngle = Math.cos(launchAngleRads);
            double arcLengthApproximation = d / Math.max(cosAngle, 0.5);

            double angleToTarget =
                    Math.atan2(
                            targetPose.getY() - shooterPose.getY(),
                            targetPose.getX() - shooterPose.getX());

            // Drag Constant: Adjust this '0.015' based on testing.
            // It adds extra distance to the 'target' to make the robot over-shoot
            // and compensate for drag.
            double dragComp = arcLengthApproximation * arcLengthApproximation * 0.015;

            double px = Math.cos(angleToTarget) * dragComp;
            double py = Math.sin(angleToTarget) * dragComp;

            return new Translation2d(px, py);
        }

        public static Translation2d addDToTargetWithAirResistance(
                Pose3d targetPose, Pose3d shooterPose) {
            double d = shooterPose.getTranslation().getDistance(targetPose.getTranslation());

            // this is complete bs
            double angleToTarget =
                    Math.atan2(
                            targetPose.getY() - shooterPose.getY(),
                            targetPose.getX() - shooterPose.getX());
            double p = d / 2.5;

            double px = Math.cos(angleToTarget) * p;
            double py = Math.sin(angleToTarget) * p;

            return new Translation2d(px, py);
        }

        public static ShooterState solve_with_project(
                Pose3d targetPose,
                Pose3d shooterPose,
                double currentRPM,
                ChassisSpeeds robotVel,
                double delaySeconds) {

            double distance = shooterPose.getTranslation().getDistance(targetPose.getTranslation());
            double vx = robotVel.vxMetersPerSecond;
            double vy = robotVel.vyMetersPerSecond;

            Translation2d AIR_RESISTANCE_ADDER =
                    addDToTargetWithAirResistance(targetPose, shooterPose);

            Pose3d projectedTargetPose =
                    new Pose3d(
                            targetPose.getX() + AIR_RESISTANCE_ADDER.getX() - vx,
                            targetPose.getY() + AIR_RESISTANCE_ADDER.getY() - vy,
                            targetPose.getZ(),
                            targetPose.getRotation());

            return solve_simple_no_AR_or_FutureTiming(projectedTargetPose, shooterPose, currentRPM);
        }

        public static ShooterState solve_with_max_height(
                Pose3d targetPose,
                Pose3d shooterPose,
                double currentRPM, // Now used for the "Ready" check
                ChassisSpeeds robotVel,
                double delaySeconds,
                double maxHeightMeters) {

            // --- 1. Position Prediction (Predict where robot will be when it shoots) ---
            double vx = robotVel.vxMetersPerSecond;
            double vy = robotVel.vyMetersPerSecond;
            double omega = robotVel.omegaRadiansPerSecond;

            double sx = shooterPose.getX() + vx * delaySeconds;
            double sy = shooterPose.getY() + vy * delaySeconds;
            double sz = shooterPose.getZ();

            double tx = targetPose.getX();
            double ty = targetPose.getY();
            double tz = targetPose.getZ();

            // --- 2. Geometry relative to predicted launch point ---
            double dx = tx - sx;
            double dy = ty - sy;
            double dz = tz - sz;
            double d_horizontal = Math.sqrt(dx * dx + dy * dy);

            // --- 3. Air Resistance (Arc Distance) ---
            double peakHeight = maxHeightMeters - sz;
            double arcLengthFactor =
                    1.0 + (2.0 / 3.0) * Math.pow(peakHeight / Math.max(d_horizontal, 0.1), 2);
            double estimatedArcDistance = d_horizontal * arcLengthFactor;

            // Drag compensation adds "virtual distance" to the target
            double dragAdjustment = Math.pow(estimatedArcDistance, 2) * 0.012;
            double angleToTarget = Math.atan2(dy, dx);

            // Required horizontal distance including drag compensation
            double d_comp = d_horizontal + dragAdjustment;

            // --- 4. Vertical Velocity (Vz) for Max Height ---
            // Ensure relativeMaxHeight is at least slightly above the target
            double relativeMaxHeight = Math.max(maxHeightMeters - sz, dz + 0.1);
            double vz = Math.sqrt(2 * G * relativeMaxHeight);

            // --- 5. Solve for Time and Required Horizontal Velocity (Field Frame) ---
            double a = 0.5 * G;
            double b = -vz;
            double c = dz;
            double discriminant = b * b - 4 * a * c;

            if (discriminant < 0) {
                canShoot(false);
                return new ShooterState(
                        RPM.of(0), Radians.of(0), Degrees.of(kHood.MIN_ANGLE_DEGREES));
            }

            double t = (-b + Math.sqrt(discriminant)) / (2 * a);
            double vh_required = d_comp / t;

            // --- 6. Vector Compensation for Robot Velocity ---
            // Desired ball velocity in field horizontal plane:
            double v_ball_x = vh_required * Math.cos(angleToTarget);
            double v_ball_y = vh_required * Math.sin(angleToTarget);

            // Required velocity from flywheel (relative to robot):
            double v_flywheel_x = v_ball_x - vx;
            double v_flywheel_y = v_ball_y - vy;

            double vh_flywheel =
                    Math.sqrt(v_flywheel_x * v_flywheel_x + v_flywheel_y * v_flywheel_y);
            double fieldShotAngle = Math.atan2(v_flywheel_y, v_flywheel_x);

            // --- 7. Hardware Constraint Validation ---
            double launchAngle = Math.atan2(vz, vh_flywheel);
            double hoodSetpointRads = Math.PI / 2 - launchAngle;

            // Retrieve constants from your SubsystemConstants
            double minHood = Math.toRadians(SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES);
            double maxHood = Math.toRadians(SubsystemConstants.kShooter.kHood.MAX_ANGLE_DEGREES);

            // Check if the angle is physically possible
            boolean anglePossible = (hoodSetpointRads >= minHood && hoodSetpointRads <= maxHood);

            // Clamp the setpoint so we don't break the hood if anglePossible is false
            double clampedHoodSetpoint = MathUtil.clamp(hoodSetpointRads, minHood, maxHood);

            // --- 8. Calculate Required RPM ---
            double vRequiredTotal = Math.sqrt(vh_flywheel * vh_flywheel + vz * vz);
            double velocityMultiplier = 0.5; // Typical for a tangential shooter
            double requiredRPM =
                    (vRequiredTotal / (FLYWHEEL_RADIUS * velocityMultiplier))
                            * 60.0
                            / (2.0 * Math.PI);

            // --- 9. Final Status and Results ---
            double robotYawFuture = shooterPose.getRotation().getZ() + omega * delaySeconds;
            double turretAngle = MathUtil.angleModulus(fieldShotAngle - robotYawFuture);

            double rpmError = Math.abs(currentRPM - requiredRPM);
            boolean rpmPossible =
                    requiredRPM < SubsystemConstants.kShooter.kFlywheels.MAX_SPEED_RPM;
            boolean rpmReady = rpmError < 150.0;

            // The shot is only "Green" if physics work, hardware can reach it, and RPM is spun up
            canShoot(anglePossible && rpmPossible && rpmReady);

            // If the angle wasn't possible, we return 0 RPM to prevent shooting a "bad" ball
            if (!anglePossible || !rpmPossible) {
                return new ShooterState(
                        RPM.of(0), Radians.of(turretAngle), Radians.of(clampedHoodSetpoint));
            }

            // For visualization, use the predicted future pose and field-relative results
            double v_field_h = vh_required;
            double v_field_total = Math.sqrt(v_field_h * v_field_h + vz * vz);
            double launchAngleField = Math.atan2(vz, v_field_h);
            double turretAngleForVis = angleToTarget - robotYawFuture;

            Pose3d futureShooterPose = new Pose3d(sx, sy, sz, new Rotation3d(0, 0, robotYawFuture));
            publishShotTrajectory(
                    v_field_total,
                    launchAngleField,
                    turretAngleForVis,
                    futureShooterPose,
                    targetPose);

            return new ShooterState(
                    RPM.of(requiredRPM), Radians.of(turretAngle), Radians.of(clampedHoodSetpoint));
        }
    }

    public static double getShotTime(
            double ballLaunchVelocity,
            double hoodAngleRadians,
            double shooterHeight,
            double targetHeight) {
        // Vertical component of the velocity
        double vY = ballLaunchVelocity * Math.sin(hoodAngleRadians);

        // Time to reach the target height using the formula: h = vY * t - 0.5 * g * t^2
        // Rearranging gives: 0.5 * g * t^2 - vY * t + (targetHeight - shooterHeight) = 0
        double a = 0.5 * 9.81;
        double b = -vY;
        double c = targetHeight - shooterHeight;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            // No real solution, the shot cannot reach the target height
            return Double.POSITIVE_INFINITY;
        }

        // We take the positive root since time cannot be negative
        double time = (-b + Math.sqrt(discriminant)) / (2 * a);
        return time;
    }

    public static void clearShotTrajectory() {
        Logger.recordOutput("Shooter/ShotTrajectory", new Translation2d[] {});
    }

    public static void publishShotTrajectory(
            double ballLaunchVelocity,
            double launchAngleRads, // Angle relative to the floor
            double fieldShotAngle, // Absolute angle toward the target
            Pose3d shooterPose3d,
            Pose3d targetPose3d) {

        double sx = shooterPose3d.getX();
        double sy = shooterPose3d.getY();
        double sz = shooterPose3d.getZ();

        // vZ is vertical, vH is horizontal across the floor
        double vZ = ballLaunchVelocity * Math.sin(launchAngleRads);
        double vH = ballLaunchVelocity * Math.cos(launchAngleRads);

        // Break horizontal velocity into field X and Y
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
