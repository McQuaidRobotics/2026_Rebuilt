package igknighters.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import igknighters.util.log.Log;
import java.util.Arrays;
import java.util.Collections;

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

    public ChassisSpeeds[] predVeloHistory = new ChassisSpeeds[HISTORY_SIZE];
    public ChassisSpeeds[] veloHistory = new ChassisSpeeds[HISTORY_SIZE];
    public double[] timestampHistory = new double[HISTORY_SIZE];

    // time in seconds to look-ahead
    public static final double predTime = .02;

    /** Index of the next write slot in the circular buffers. */
    public int writeIndex = 0;

    /** Number of poses stored so far, capped at HISTORY_SIZE. */
    public int storedCount = 0;

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
    public void setNewPose(ChassisSpeeds chassisSpeeds) {
        double now = Timer.getFPGATimestamp();

        // Write into circular buffer
        veloHistory[writeIndex] = chassisSpeeds;
        predVeloHistory[writeIndex] = chassisSpeeds;
        timestampHistory[writeIndex] = now;
        double mostRecentTimestamp =
                Collections.max(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx =
                Arrays.stream(timestampHistory).boxed().toList().indexOf(mostRecentTimestamp);
        double dt = 0;
        if (latestIdx == 0 && predVeloHistory[HISTORY_SIZE - 1] == null) {
            dt = 0;
        } else if (latestIdx == 0 && predVeloHistory[HISTORY_SIZE - 1] != null) {
            dt = timestampHistory[latestIdx] - timestampHistory[HISTORY_SIZE - 1];
        }
        if (latestIdx != 0) {
            dt = timestampHistory[latestIdx] - timestampHistory[latestIdx - 1];
        }
        writeIndex = (writeIndex + 1) % HISTORY_SIZE;
        storedCount++;

        final ChassisSpeeds updatedVelos = new ChassisSpeeds();

        final ChassisSpeeds currentVelos =
                new ChassisSpeeds(
                        chassisSpeeds.vxMetersPerSecond,
                        chassisSpeeds.vyMetersPerSecond,
                        chassisSpeeds.omegaRadiansPerSecond);

        if (dt <= 0.0) return;
        // find acceleration based on last velocity and current
        ChassisSpeeds predictedVelo = new ChassisSpeeds();

        if (latestIdx == 0 && predVeloHistory[HISTORY_SIZE - 1] == null) {
            predictedVelo.vxMetersPerSecond =
                    currentVelos.vxMetersPerSecond + (currentVelos.vxMetersPerSecond) * dt;
            predictedVelo.vyMetersPerSecond =
                    currentVelos.vyMetersPerSecond + (currentVelos.vyMetersPerSecond) * dt;
            predictedVelo.omegaRadiansPerSecond =
                    currentVelos.omegaRadiansPerSecond + (currentVelos.omegaRadiansPerSecond) * dt;
        } else if (latestIdx == 0) {
            predictedVelo.vxMetersPerSecond =
                    currentVelos.vxMetersPerSecond
                            + (currentVelos.vxMetersPerSecond
                                            - predVeloHistory[HISTORY_SIZE - 1].vxMetersPerSecond)
                                    * dt;
            predictedVelo.vyMetersPerSecond =
                    currentVelos.vyMetersPerSecond
                            + (currentVelos.vyMetersPerSecond
                                            - predVeloHistory[HISTORY_SIZE - 1].vyMetersPerSecond)
                                    * dt;
            predictedVelo.omegaRadiansPerSecond =
                    currentVelos.omegaRadiansPerSecond
                            + (currentVelos.omegaRadiansPerSecond
                                            - predVeloHistory[HISTORY_SIZE - 1]
                                                    .omegaRadiansPerSecond)
                                    * dt;
        } else {
            predictedVelo.vxMetersPerSecond =
                    currentVelos.vxMetersPerSecond
                            + (currentVelos.vxMetersPerSecond
                                            - predVeloHistory[latestIdx - 1].vxMetersPerSecond)
                                    * dt;
            predictedVelo.vyMetersPerSecond =
                    currentVelos.vyMetersPerSecond
                            + (currentVelos.vyMetersPerSecond
                                            - predVeloHistory[latestIdx - 1].vyMetersPerSecond)
                                    * dt;
            predictedVelo.omegaRadiansPerSecond =
                    currentVelos.omegaRadiansPerSecond
                            + (currentVelos.omegaRadiansPerSecond
                                            - predVeloHistory[latestIdx - 1].omegaRadiansPerSecond)
                                    * dt;
        }

        // Residual: difference between measurement and prediction
        double[] residual = new double[3];
        residual[0] = currentVelos.vxMetersPerSecond - predictedVelo.vxMetersPerSecond;
        residual[1] = currentVelos.vyMetersPerSecond - predictedVelo.vyMetersPerSecond;
        residual[2] = currentVelos.omegaRadiansPerSecond - predictedVelo.omegaRadiansPerSecond;
        // Normalize angular residuals to [-π, π]
        for (int i = 3; i < 3; i++) {
            residual[i] = Math.atan2(Math.sin(residual[i]), Math.cos(residual[i]));
        }

        // Alpha-beta update
        updatedVelos.vxMetersPerSecond =
                predictedVelo.vxMetersPerSecond + currentVelos.vxMetersPerSecond;
        updatedVelos.vyMetersPerSecond =
                predictedVelo.vyMetersPerSecond + currentVelos.vyMetersPerSecond;
        updatedVelos.omegaRadiansPerSecond =
                predictedVelo.omegaRadiansPerSecond + currentVelos.omegaRadiansPerSecond;

        predVeloHistory[latestIdx] = updatedVelos;
    }

    public ChassisSpeeds getVelos(ChassisSpeeds chassisSpeeds) {
        double mostRecentTimestamp =
                Collections.max(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx =
                Arrays.stream(timestampHistory).boxed().toList().indexOf(mostRecentTimestamp);
        return predVeloHistory[latestIdx];
    }

    /**
     * Returns the estimated robot pose at the next loop iteration, extrapolated from the current
     * smoothed state using the most recent dt.
     *
     * @return predicted {@link Pose2d} one loop period into the future
     */
    public Pose2d getPredictedPose(Pose2d pose) {
        double[] currentPose = poseToComponents(pose);
        ChassisSpeeds prediction = new ChassisSpeeds();
        double mostRecentTimestamp =
                Collections.max(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx =
                Arrays.stream(timestampHistory).boxed().toList().indexOf(mostRecentTimestamp);

        double[] current = poseToComponents(pose);
        double[] predicted = new double[3];
        predicted[0] = current[0] + veloHistory[latestIdx].vxMetersPerSecond * predTime;
        predicted[1] = current[1] + veloHistory[latestIdx].vyMetersPerSecond * predTime;
        predicted[2] = current[2] + veloHistory[latestIdx].omegaRadiansPerSecond * predTime;
        if (veloHistory != null) {
            Log.log("ROBOT/veloHistory", veloHistory);
        }

        return componentsToPose(predicted);
    }

    public ChassisSpeeds getPredictedVelos() {
        double mostRecentTimestamp =
                Collections.max(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx =
                Arrays.stream(timestampHistory).boxed().toList().indexOf(mostRecentTimestamp);
        return predVeloHistory[latestIdx];
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /** Decomposes a Pose2d into [x, y, rotation]. */
    private static double[] poseToComponents(Pose2d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

    /** Reconstructs a Pose2d from [x, y, roll, pitch]. */
    private static Pose2d componentsToPose(double[] c) {
        return new Pose2d(new Translation2d(c[0], c[1]), new Rotation2d(c[2]));
    }
}
