package igknighters.subsystems.shooter;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;

public class AimSolver {

    // flywheel radius (4 in diameter → 0.0508 m radius)
    private static final double FLYWHEEL_RADIUS = 0.0508;

    // Gravity
    private static final double G = 9.81;

    /**
     * @return { turretAngleRadians, hoodAngleRadians, rpm } or null if the shot is impossible at
     *     this RPM
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
            DogLog.log("Subsystems/Shooter/Aiming/SHOT IS NOT POSSIBLE AT THIS RPM", currentRPM);
            return new ShooterState(0.0, turretAngle, 0.0);
        }
        DogLog.log("Subsystems/Shooter/Aiming/SHOT IS POSSIBLE AT THIS RPM", currentRPM);

        double root = Math.sqrt(inside);

        double thetaLow = Math.atan((v * v - root) / (G * d));
        double thetaHigh = Math.atan((v * v + root) / (G * d));

        // You want the HIGH arc
        double hoodAngle = Math.max(thetaLow, thetaHigh);

        return new ShooterState(currentRPM, turretAngle, hoodAngle);
    }
}
