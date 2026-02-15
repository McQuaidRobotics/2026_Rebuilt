package igknighters.subsystems.shooter;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.FieldVisualizer;
import igknighters.constants.Conv;
import igknighters.constants.SubsystemConstants;
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

        private static final double FLYWHEEL_RADIUS =
                SubsystemConstants.kShooter.kRollers.WHEEL_RADIUS_METERS;

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
            double turretAngle =
                    absoluteAngle - robotYaw; // the angle the turret must turn to face target

            // Normalize to [-π, π]
            turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle));

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
                return new ShooterState(0.0, turretAngle, 0.0);
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

            publishShotTrajectory(v, hoodAngle, turretAngle, shooterPose, targetPose);

            return new ShooterState(currentRPM, turretAngle, hoodSetpoint);
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
                return new ShooterState(0.0, turretAngle, 0.0);
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
            return new ShooterState(currentRPM, turretAngle, hoodSetpoint);
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
                        0.0,
                        turretAngle,
                        SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES
                                * Conv.DEGREES_TO_RADIANS);
            }

            double hoodSetpoint = Math.PI / 2 - finalTheta;

            if (hoodSetpoint < kHood.MIN_ANGLE_DEGREES * Conv.DEGREES_TO_RADIANS
                    || hoodSetpoint > kHood.MAX_ANGLE_DEGREES * Conv.DEGREES_TO_RADIANS) {
                DogLog.log(
                        "Subsystems/Shooter/Aiming/Calculated hood angle out of bounds",
                        Math.toDegrees(hoodSetpoint));
                return new ShooterState(
                        0.0,
                        turretAngle,
                        SubsystemConstants.kShooter.kHood.MIN_ANGLE_DEGREES
                                * Conv.DEGREES_TO_RADIANS);
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

            return new ShooterState(currentRPM, turretAngle, hoodSetpoint);
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

            double projectedX = targetPose.getX() - (vx * distance / 2.0); // This is a very rough
            // estimate
            // of where the
            // target will be
            double projectedY = targetPose.getY() - (vy * distance / 2.0); // This is a very rough
            // estimate
            // of where the
            // target will be

            Pose3d projectedTargetPose =
                    new Pose3d(projectedX, projectedY, targetPose.getZ(), targetPose.getRotation());

            return solve_simple_no_AR_or_FutureTiming(projectedTargetPose, shooterPose, currentRPM);
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
            double a = 0.5 * G;
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

        public static void publishShotTrajectory(
                double ballLaunchVelocity,
                double launchAngleRads,
                double turretAngleRads,
                Pose3d shooterPose3d,
                Pose3d targetPose3d) {
            double sx = shooterPose3d.getX();
            double sy = shooterPose3d.getY();
            double sz = shooterPose3d.getZ();

            // The turret angle is relative to the robot's yaw.
            // We need the absolute field angle for the trajectory visualization.
            double fieldShotAngle = shooterPose3d.getRotation().getZ() + turretAngleRads;

            double time = getShotTime(ballLaunchVelocity, launchAngleRads, sz, targetPose3d.getZ());
            if (Double.isInfinite(time) || time <= 0) time = 1.5; // Fallback for visualization

            int nPoints = 20;
            Pose3d[] trajectoryPoints = new Pose3d[nPoints];

            for (int i = 0; i < nPoints; i++) {
                double t = (time / (nPoints - 1)) * i;
                double x =
                        sx
                                + ballLaunchVelocity
                                        * Math.cos(launchAngleRads)
                                        * Math.cos(fieldShotAngle)
                                        * t;
                double y =
                        sy
                                + ballLaunchVelocity
                                        * Math.cos(launchAngleRads)
                                        * Math.sin(fieldShotAngle)
                                        * t;
                double z =
                        sz + ballLaunchVelocity * Math.sin(launchAngleRads) * t - (0.5 * G * t * t);

                trajectoryPoints[i] = new Pose3d(x, y, Math.max(0, z), new Rotation3d());
            }

            Logger.recordOutput("Shooter/ShotTrajectory", trajectoryPoints);
        }
    }
}
