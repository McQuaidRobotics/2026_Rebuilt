package igknighters.subsystems.shooter.solvers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.subsystems.shooter.ShooterState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public abstract class Solver {

    public abstract ShooterState solve(
            Supplier<Pose3d> shooterPose,
            Supplier<Pose3d> targetPose,
            Supplier<ChassisSpeeds> chassisSpeeds,
            double currentRPM,
            double maxHeightMeters);

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

    public static Translation2d addDToTargetWithAirResistance(
            Pose3d targetPose, Pose3d shooterPose) {
        double d = shooterPose.getTranslation().getDistance(targetPose.getTranslation());

        // this is complete bs
        double angleToTarget =
                Math.atan2(
                        targetPose.getY() - shooterPose.getY(),
                        targetPose.getX() - shooterPose.getX());
        double p = d / 1.5;

        double px = Math.cos(angleToTarget) * p;
        double py = Math.sin(angleToTarget) * p;

        return new Translation2d(px, py);
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
