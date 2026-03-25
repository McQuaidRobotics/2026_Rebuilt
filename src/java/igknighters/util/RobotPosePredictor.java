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
        for (int i = 0; i < HISTORY_SIZE; i++) {
            veloHistory[i] = new ChassisSpeeds(0, 0, 0);
        }
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
        Object xCopy = PipedDeepCopy.copy(chassisSpeeds.vxMetersPerSecond);
        Object yCopy = PipedDeepCopy.copy(chassisSpeeds.vyMetersPerSecond);
        Object oCopy = PipedDeepCopy.copy(chassisSpeeds.omegaRadiansPerSecond);
        veloHistory[writeIndex].vxMetersPerSecond = (double) xCopy;
        veloHistory[writeIndex].vyMetersPerSecond = (double) yCopy;
        veloHistory[writeIndex].omegaRadiansPerSecond = (double) oCopy;
        timestampHistory[writeIndex] = now;
        double mostRecentTimestamp =
                Collections.max(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx =
                Arrays.stream(timestampHistory).boxed().toList().indexOf(mostRecentTimestamp);
        int prevIdx = 0;
        if (latestIdx == 0) {
            prevIdx = HISTORY_SIZE - 1;
        } else {
            prevIdx = latestIdx - 1;
        }
        writeIndex = (writeIndex + 1) % HISTORY_SIZE;
        storedCount++;
        if (veloHistory[HISTORY_SIZE - 1] != null) {
            Log.log("ROBOT/veloHistory", veloHistory);
        }
    }

    public Pose2d getPredictedPose(Pose2d pose) {

        double mostRecentTimestamp =
                Collections.max(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx =
                Arrays.stream(timestampHistory).boxed().toList().indexOf(mostRecentTimestamp);
        int prevIdx = 0;
        if (latestIdx == 0) {
            prevIdx = HISTORY_SIZE - 1;
        } else {
            prevIdx = latestIdx - 1;
        }
        double dt = timestampHistory[latestIdx] - timestampHistory[prevIdx];
        final ChassisSpeeds currentVelos =
                new ChassisSpeeds(
                        (double) PipedDeepCopy.copy(veloHistory[latestIdx].vxMetersPerSecond),
                        (double) PipedDeepCopy.copy(veloHistory[latestIdx].vyMetersPerSecond),
                        (double) PipedDeepCopy.copy(veloHistory[latestIdx].omegaRadiansPerSecond));

        if (dt <= 0.0) return pose;
        // find acceleration based on last velocity and current
        ChassisSpeeds predictedVelo = new ChassisSpeeds();

        if (veloHistory[HISTORY_SIZE - 1] == null) {
            return pose;
        }

        double[] currentPose = poseToComponents(pose);
        double[] predicted = new double[3];

        // Predict next pose using predicted velocity
        predicted[0] = currentPose[0] + currentVelos.vxMetersPerSecond * predTime;
        predicted[1] = currentPose[1] + currentVelos.vyMetersPerSecond * predTime;
        predicted[2] = currentPose[2] + currentVelos.omegaRadiansPerSecond * predTime;

        // predicted[0] = currentPose[0] +
        // (currentVelos.vxMetersPerSecond-veloHistory[prevIdx].vxMetersPerSecond*dt);
        // predicted[1] = currentPose[1] +
        // (currentVelos.vyMetersPerSecond-veloHistory[prevIdx].vyMetersPerSecond*dt);
        // predicted[2] = currentPose[2] +
        // (currentVelos.omegaRadiansPerSecond-veloHistory[prevIdx].omegaRadiansPerSecond*dt);

        return componentsToPose(predicted);
    }

    public ChassisSpeeds getPredictedVelos() {
        ChassisSpeeds predictedAcc = new ChassisSpeeds();
        ChassisSpeeds predictedVelo = new ChassisSpeeds();
        double mostRecentTimestamp =
                Collections.max(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx =
                Arrays.stream(timestampHistory).boxed().toList().indexOf(mostRecentTimestamp);
        int prevIdx = 0;
        if (latestIdx == 0) {
            prevIdx = HISTORY_SIZE - 1;
        } else {
            prevIdx = latestIdx - 1;
        }
        double dt = timestampHistory[latestIdx] - timestampHistory[prevIdx];
        predictedAcc.vxMetersPerSecond =
                veloHistory[latestIdx].vxMetersPerSecond
                        + (veloHistory[latestIdx].vxMetersPerSecond
                                        - veloHistory[prevIdx].vxMetersPerSecond)
                                / dt;
        predictedAcc.vyMetersPerSecond =
                veloHistory[latestIdx].vyMetersPerSecond
                        + (veloHistory[latestIdx].vyMetersPerSecond
                                        - veloHistory[prevIdx].vyMetersPerSecond)
                                / dt;
        predictedAcc.omegaRadiansPerSecond =
                veloHistory[latestIdx].omegaRadiansPerSecond
                        + (veloHistory[latestIdx].omegaRadiansPerSecond
                                        - veloHistory[prevIdx].omegaRadiansPerSecond)
                                / dt;
        predictedVelo.vxMetersPerSecond = predictedAcc.vxMetersPerSecond * predTime;
        predictedVelo.vyMetersPerSecond = predictedAcc.vyMetersPerSecond * predTime;
        predictedVelo.omegaRadiansPerSecond = predictedAcc.omegaRadiansPerSecond * predTime;

        return predictedVelo;
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
