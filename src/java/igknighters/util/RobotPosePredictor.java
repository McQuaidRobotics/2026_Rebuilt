package igknighters.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * Estimates the robot's pose on the next loop iteration using an alpha-beta
 * filter applied to a rolling history of timestamped poses.
 *
 * <p>Alpha corrects the position estimate toward each new measurement.
 * Beta corrects the velocity estimate to reduce lag over time.
 */
public class RobotPosePredictor {

    private static final int HISTORY_SIZE = 10;

    /** Smoothing gain for the position estimate (0 < alpha ≤ 1). */
    private final double alpha;

    /** Smoothing gain for the velocity estimate (0 < beta ≤ 1). */
    private final double beta;

    private final Pose3d[] poseHistory = new Pose3d[HISTORY_SIZE];
    private final double[] timestampHistory = new double[HISTORY_SIZE];

    /** Index of the next write slot in the circular buffers. */
    private int writeIndex = 0;

    /** Number of poses stored so far, capped at HISTORY_SIZE. */
    private int storedCount = 0;

    // Alpha-beta filter state: smoothed pose and per-axis velocity (m/s or rad/s)
    private Pose3d smoothedPose = new Pose3d();
    private final double[] smoothedVelocity = new double[6]; // vx, vy, vz, vroll, vpitch, vyaw

    /**
     * @param alpha position smoothing gain, typically 0.5–0.9
     * @param beta  velocity smoothing gain, typically 0.1–0.5
     */
    public RobotPosePredictor(double alpha, double beta) {
        this.alpha = alpha;
        this.beta = beta;
    }

    /**
     * Records a new measured pose and updates the internal filter state.
     * Should be called once per loop iteration whenever a fresh pose is available.
     *
     * @param pose the latest measured robot pose
     */
    public void setNewPose(Pose3d pose) {
        double now = Timer.getFPGATimestamp();

        // Write into circular buffer
        poseHistory[writeIndex] = pose;
        timestampHistory[writeIndex] = now;
        writeIndex = (writeIndex + 1) % HISTORY_SIZE;
        storedCount++;

        // Need at least one prior sample to compute dt and run the filter
        if (storedCount == 1) {
            smoothedPose = pose;
            return;
        }

        int latestIdx = (writeIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
        int prevIdx   = (writeIndex - 2 + HISTORY_SIZE) % HISTORY_SIZE;
        double dt = timestampHistory[latestIdx] - timestampHistory[prevIdx];

        if (dt <= 0.0) return;

        double[] measured  = poseToComponents(pose);
        double[] estimated = poseToComponents(smoothedPose);

        // Predict where we thought we'd be based on last velocity
        double[] predicted = new double[6];
        for (int i = 0; i < 6; i++) {
            predicted[i] = estimated[i] + smoothedVelocity[i] * dt;
        }

        // Residual: difference between measurement and prediction
        double[] residual = new double[6];
        for (int i = 0; i < 6; i++) {
            residual[i] = measured[i] - predicted[i];
        }
        // Normalize angular residuals to [-π, π]
        for (int i = 3; i < 6; i++) {
            residual[i] = Math.atan2(Math.sin(residual[i]), Math.cos(residual[i]));
        }

        // Alpha-beta update
        double[] updated = new double[6];
        for (int i = 0; i < 6; i++) {
            updated[i]           = predicted[i] + alpha * residual[i];
            smoothedVelocity[i] += (beta / dt)  * residual[i];
        }

        smoothedPose = componentsToPose(updated);
    }

    /**
     * Returns the estimated robot pose at the next loop iteration, extrapolated
     * from the current smoothed state using the most recent dt.
     *
     * @return predicted {@link Pose3d} one loop period into the future
     */
    public Pose3d getPredictedPose() {
        if (storedCount < 2) return smoothedPose;

        int latestIdx = (writeIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
        int prevIdx   = (writeIndex - 2 + HISTORY_SIZE) % HISTORY_SIZE;
        double dt = timestampHistory[latestIdx] - timestampHistory[prevIdx];

        if (dt <= 0.0) return smoothedPose;

        double[] current   = poseToComponents(smoothedPose);
        double[] predicted = new double[6];
        for (int i = 0; i < 6; i++) {
            predicted[i] = current[i] + smoothedVelocity[i] * dt;
        }

        return componentsToPose(predicted);
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /** Decomposes a Pose3d into [x, y, z, roll, pitch, yaw]. */
    private static double[] poseToComponents(Pose3d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            pose.getRotation().getX(),
            pose.getRotation().getY(),
            pose.getRotation().getZ()
        };
    }

    /** Reconstructs a Pose3d from [x, y, z, roll, pitch, yaw]. */
    private static Pose3d componentsToPose(double[] c) {
        return new Pose3d(
                new Translation3d(c[0], c[1], c[2]),
                new Rotation3d(c[3], c[4], c[5]));
    }
}
