package igknighters.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * Estimates the robot's pose on the next loop iteration using an alpha-beta filter applied to a
 * rolling history of timestamped poses.
 *
 * <p>Alpha corrects the position estimate toward each new measurement. Beta corrects the velocity
 * estimate to reduce lag over time.
 */
public class RobotPosePredictor {

    private static final int HISTORY_SIZE = 10;

    /** Smoothing gain for the position estimate (0 < alpha ≤ 1). */
    private final double alpha;

    /** Smoothing gain for the velocity estimate (0 < beta ≤ 1). */
    private final double beta;

    private final double[][] veloHistory = new double[HISTORY_SIZE][3];
    private final double[] timestampHistory = new double[HISTORY_SIZE];

    // time in seconds to look-ahead
    private final double predTime = .02;

    /** Index of the next write slot in the circular buffers. */
    private int writeIndex = 0;

    /** Number of poses stored so far, capped at HISTORY_SIZE. */
    private int storedCount = 0;

    // Alpha-beta filter state: smoothed pose and per-axis velocity (m/s or rad/s)
    private double[] smoothedVelocities = new double[3];

    /**
     * @param alpha position smoothing gain, typically 0.5–0.9
     * @param beta velocity smoothing gain, typically 0.1–0.5
     */
    public RobotPosePredictor(double alpha, double beta) {
        this.alpha = alpha;
        this.beta = beta;
    }

    /**
     * Records a new measured pose and updates the internal filter state. Should be called once per
     * loop iteration whenever a fresh pose is available.
     *
     * @param pose the latest measured robot pose
     */
    public void setNewPose(Pose2d pose, ChassisSpeeds chassisSpeeds) {
        double now = Timer.getFPGATimestamp();

        // Write into circular buffer
        veloHistory[writeIndex][0] = chassisSpeeds.vxMetersPerSecond;
        veloHistory[writeIndex][1] = chassisSpeeds.vyMetersPerSecond;
        veloHistory[writeIndex][2] = chassisSpeeds.omegaRadiansPerSecond;
        timestampHistory[writeIndex] = now;
        writeIndex = (writeIndex + 1) % HISTORY_SIZE;
        storedCount++;

        final double[] updatedVelos = new double[3];

        final double[] currentVelos = {
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond
        };

        // Need at least one prior sample to compute dt and run the filter
        if (storedCount == 1) {
            smoothedVelocities = currentVelos;
            return;
        }

        int latestIdx = (writeIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
        int prevIdx = (writeIndex - 2 + HISTORY_SIZE) % HISTORY_SIZE;
        double dt = timestampHistory[latestIdx] - timestampHistory[prevIdx];

        if (dt <= 0.0) return;

        // find acceleration based on last velocity and current
        double[] predictedVelo = new double[3];
        for (int i = 0; i < 3; i++) {
            predictedVelo[i] = currentVelos[i] + (currentVelos[i] - veloHistory[prevIdx][i]) * dt;
        }

        // Residual: difference between measurement and prediction
        double[] residual = new double[3];
        for (int i = 0; i < 3; i++) {
            residual[i] = currentVelos[i] - predictedVelo[i];
        }
        // Normalize angular residuals to [-π, π]
        for (int i = 3; i < 3; i++) {
            residual[i] = Math.atan2(Math.sin(residual[i]), Math.cos(residual[i]));
        }

        // Alpha-beta update
        for (int i = 0; i < 3; i++) {
            updatedVelos[i] = predictedVelo[i] + currentVelos[i];
        }

        smoothedVelocities = updatedVelos;
    }


        // public double getPredictedVelos(Pose2d pose, ChassisSpeeds chassisSpeeds) {

        // }
    /**
     * Returns the estimated robot pose at the next loop iteration, extrapolated from the current
     * smoothed state using the most recent dt.
     *
     * @return predicted {@link Pose2d} one loop period into the future
     */
    public Pose2d getPredictedPose(Pose2d pose) {
        double[] currentPose = poseToComponents(pose);
        double[] prediction = new double[3];
        if (storedCount < 2) {
            for (int i = 0; i < 3; i++) {
                prediction[i] = currentPose[i] + smoothedVelocities[i] * predTime;
            }
            return componentsToPose(prediction);
        }

        int latestIdx = (writeIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
        int prevIdx = (writeIndex - 2 + HISTORY_SIZE) % HISTORY_SIZE;
        double dt = timestampHistory[latestIdx] - timestampHistory[prevIdx];

        if (dt <= 0.0) return pose;

        double[] current = poseToComponents(pose);
        double[] predicted = new double[3];
        for (int i = 0; i < 3; i++) {
            predicted[i] = currentPose[i] + smoothedVelocities[i] * predTime;
        }

        return componentsToPose(predicted);
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /** Decomposes a Pose2d into [x, y, roll, pitch]. */
    private static double[] poseToComponents(Pose2d pose) {
        return new double[] {
            pose.getX(), pose.getY(), pose.getRotation().getCos(), pose.getRotation().getSin()
        };
    }

    /** Reconstructs a Pose2d from [x, y, roll, pitch]. */
    private static Pose2d componentsToPose(double[] c) {
        return new Pose2d(new Translation2d(c[0], c[1]), new Rotation2d(c[2]));
    }
}
