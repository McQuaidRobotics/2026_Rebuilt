package igknighters.util.Prediction;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Arrays;

/**
 * Represents a snapshot of vision measurements from multiple cameras. Holds the poses, timestamps
 * and per-camera XY standard deviations from each camera.
 */
public class VisionSnapshot {
    /** WPILib's default vision std dev, used when a source can't estimate its own. */
    public static final double DEFAULT_XY_STD_DEV = 0.9;

    /** Heading std dev for a pose whose heading is just our own gyro echoed back (MT2). */
    public static final double UNTRUSTED_THETA_STD_DEV = 9999999;

    private final Pose2d[] pose;
    private final Double[] timestamp;
    private final double[] xyStdDev;
    private final double[] thetaStdDev;

    public VisionSnapshot(
            Pose2d[] poses, Double[] timestamp, double[] xyStdDevs, double[] thetaStdDevs) {
        this.pose = poses;
        this.timestamp = timestamp;
        this.xyStdDev = xyStdDevs;
        this.thetaStdDev = thetaStdDevs;
    }

    /** Sim: default XY trust, heading not trusted (sim poses come from our own estimate). */
    public VisionSnapshot(Pose2d[] poses, Double[] timestamp) {
        this(
                poses,
                timestamp,
                filled(poses.length, DEFAULT_XY_STD_DEV),
                filled(poses.length, UNTRUSTED_THETA_STD_DEV));
    }

    private static double[] filled(int n, double value) {
        double[] a = new double[n];
        Arrays.fill(a, value);
        return a;
    }

    public Pose2d[] getPose() {
        return pose;
    }

    public Double[] getTimestamp() {
        return timestamp;
    }

    public double[] getXyStdDev() {
        return xyStdDev;
    }

    public double[] getThetaStdDev() {
        return thetaStdDev;
    }
}
