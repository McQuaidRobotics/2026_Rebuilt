package igknighters.subsystems.shooter;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.constants.SubsystemConstants;

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
            double v = omega * FLYWHEEL_RADIUS; // m/s

            // --- Solve for hood angle (high arc) ---
            double inside = v * v * v * v - G * (G * d * d + 2 * h * v * v);

            if (inside < 0) {
                // Shot is physically impossible at this RPM
                DogLog.log(
                        "Subsystems/Shooter/Aiming/SHOT IS NOT POSSIBLE AT THIS RPM", currentRPM);
                canShoot(false);
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

            return new ShooterState(currentRPM, turretAngle, Math.PI / 2 - hoodAngle);
        }

        public static double getSwerveVelocityProjection(double vx, double vy, double turretAngle) {
            return vx * Math.cos(turretAngle) + vy * Math.sin(turretAngle);
        }

        public static ShooterState solve_moving(
                Pose3d targetPose,
                Pose3d shooterPose,
                double currentRPM,
                Pose2d shooterVel, // vx, vy, omega
                double delaySeconds) {

            // -----------------------------
            // 1. Predict shooter future pose
            // -----------------------------
            double vx = shooterVel.getX();
            double vy = shooterVel.getY();
            double omega = shooterVel.getRotation().getRadians(); // rad/s

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
            double vFlywheel = flywheelOmega * FLYWHEEL_RADIUS;

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
            DogLog.log("Subsystems/Shooter/Aiming/Launch Velocity", v);
            DogLog.log("Subsystems/Shooter/Aiming/Ballistic Discriminant", inside);

            if (inside < 0) {
                DogLog.log(
                        "Subsystems/Shooter/Aiming/SHOT IS NOT POSSIBLE AT THIS RPM", currentRPM);
                canShoot(false);

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

            return new ShooterState(currentRPM, turretAngle, hoodSetpoint);
        }
    }
}
