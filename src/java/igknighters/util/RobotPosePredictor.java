package igknighters.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import igknighters.Robot;
import igknighters.subsystems.swerve.Swerve;
import java.util.Arrays;
import java.util.Collections;
import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

/**
 * Estimates the robot's pose on the next loop iteration using an alpha-beta filter applied to a
 * rolling history of timestamped poses.
 *
 * <p>Alpha corrects the position estimate toward each new measurement. Beta corrects the velocity
 * estimate to reduce lag over time.
 */
public class RobotPosePredictor {

    private static final int HISTORY_SIZE = 10;

    private static boolean usingAuto = false;

    private static SwerveSample swerveSample;

    private static AutoTrajectory autoTrajectory;

    public ChassisSpeeds[] veloHistory = new ChassisSpeeds[HISTORY_SIZE];
    public double[] timestampHistory = new double[HISTORY_SIZE];

    // time in seconds to look-ahead
    public static final double predTime = .02;

    Pose2d poseNow = new Pose2d();

    double[] accelerationsNow = new double[2];
    double[] rotVelos = new double[HISTORY_SIZE];

    /** Index of the next write slot in the circular buffers. */
    public int writeIndex = 0;

    /** Number of poses stored so far, capped at HISTORY_SIZE. */
    public int storedCount = 0;

    public RobotPosePredictor() {
        for (int i = 0; i < HISTORY_SIZE; i++) {
            veloHistory[i] = new ChassisSpeeds(0, 0, 0);
        }
    }

    public void updateAutoState(SwerveSample sample, boolean isSwerveMoving) {
        swerveSample = sample;
        usingAuto = isSwerveMoving;
    }

    /**
     * Records a new measured pose and updates the internal filter state. Should be called once per
     * loop iteration whenever a fresh pose is available.
     *
     * @param pose the latest measured robot pose
     */
    public void setVelocitiesAndPose(Swerve swerve) {

        double now = Timer.getFPGATimestamp();
        poseNow = swerve.getState().Pose;
        ChassisSpeeds chassisSpeeds = swerve.getFieldRelativeSpeeds();
        accelerationsNow[0] = swerve.getXAcceleration();
        accelerationsNow[1] = swerve.getYAcceleration();
        veloHistory[writeIndex].omegaRadiansPerSecond = swerve.getRotationalVelocity();
        // Write into circular buffer
        veloHistory[writeIndex].vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond;
        veloHistory[writeIndex].vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond;
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
        // if (veloHistory[HISTORY_SIZE - 1] != null) {
        //     Log.log("ROBOT/veloHistory", veloHistory);
        // }
    }

    public Pose2d getPredictedPose() {

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

        if (dt <= 0.0) return poseNow;

        if (veloHistory[HISTORY_SIZE - 1] == null) {
            return poseNow;
        }

        double[] currentPose = poseToComponents(poseNow);
        double[] predicted = new double[3];

        // Predict next pose using predicted velocity (consider changing predicted velo to current
        // velo)
        double predictedRotAcc = getCalculatedRotAcceleration();
        predicted[0] =
                currentPose[0]
                        + veloHistory[latestIdx].vxMetersPerSecond * predTime
                        + 1 / 2 * accelerationsNow[0] * Math.pow(predTime, 2);
        predicted[1] =
                currentPose[1]
                        + veloHistory[latestIdx].vyMetersPerSecond * predTime
                        + 1 / 2 * accelerationsNow[1] * Math.pow(predTime, 2);
        // handle wrapping
        double predOmega =
                currentPose[2]
                        + veloHistory[latestIdx].omegaRadiansPerSecond * 0.1
                        + 1 / 2 * predictedRotAcc * Math.pow(0.1, 2);

        if (predOmega > Math.PI) {
            predicted[2] = predOmega - 2 * Math.PI;
        } else if (predOmega < -Math.PI) {
            predicted[2] = predOmega - 2 * Math.PI;
        } else {
            predicted[2] = predOmega;
        }

        Robot.pose_pred_error.findError(poseNow);

        return componentsToPose(predicted);
    }
    /**
     * Gets the predicted velocities from a Choreo trajectory at a specific time.
     * @param trajectory pass straight from choreo
     * @param initialTime simply pass RobotController.getFPGATime()
     * @return predictedSpeeds
     */

    public ChassisSpeeds getPredictedVelosFromChoreo(AutoTrajectory trajectory, double initialTime) {
        // Implementation for getting predicted velocities from Choreo trajectory

        if(!usingAuto) {
            return new ChassisSpeeds();
        }

        autoTrajectory = trajectory;

        usingAuto = true;

        Trajectory<SwerveSample> rawTrajectory = trajectory.getRawTrajectory();

        Optional<SwerveSample> sample = rawTrajectory.sampleAt((1.0 / 1000000.0) * (RobotController.getFPGATime()-initialTime), true); // mili to seconds
        // this will return the predicted velocities at the specified time if it exists if not will use standard
        if (sample.isPresent()) {
            return sample.get().getChassisSpeeds();
        } else {
            return getPredictedVelos();
        }
    }


    public Pose2d getPredictedPoseFromChoreo(AutoTrajectory trajectory, double initialTime) {
        // Implementation for getting predicted pose from Choreo trajectory

        if(!usingAuto) {
            return new Pose2d();
        }

        Pose2d predictedNoChoreo = getPredictedPose();
        Trajectory<SwerveSample> rawTrajectory = trajectory.getRawTrajectory();

        Optional<SwerveSample> sample = rawTrajectory.sampleAt((1.0 / 1000000.0) * (RobotController.getFPGATime()-initialTime), true); // mili to seconds
        // this will return the predicted pose at the specified time if it exists if not will use standard
        if (sample.isPresent()) {
            return PoseMerger.trustedMerge(sample.get().getPose(), predictedNoChoreo);
        } else {
            return predictedNoChoreo;
        }
    }

    public ChassisSpeeds getPredictedVelos() {
        ChassisSpeeds predictedVelo = new ChassisSpeeds();
        double mostRecentTimestamp =
                Collections.max(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx =
                Arrays.stream(timestampHistory).boxed().toList().indexOf(mostRecentTimestamp);
        double predictedRotAcc = getCalculatedRotAcceleration();
        predictedVelo.vxMetersPerSecond =
                veloHistory[latestIdx].vxMetersPerSecond + accelerationsNow[0] * predTime;
        predictedVelo.vyMetersPerSecond =
                veloHistory[latestIdx].vyMetersPerSecond + accelerationsNow[1] * predTime;
        predictedVelo.omegaRadiansPerSecond =
                veloHistory[latestIdx].omegaRadiansPerSecond + predictedRotAcc * predTime;

        return predictedVelo;
    }

    public double getCalculatedRotAcceleration() {
        ChassisSpeeds predictedAcc = new ChassisSpeeds();
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
        predictedAcc.omegaRadiansPerSecond =
                (veloHistory[latestIdx].omegaRadiansPerSecond
                                - veloHistory[prevIdx].omegaRadiansPerSecond)
                        / dt;
        return predictedAcc.omegaRadiansPerSecond;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /** Decomposes a Pose2d into [x, y, rotation]. */
    private static double[] poseToComponents(Pose2d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

    private static double[] poseToComponents(Pose3d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getZ(), pose.getRotation().getZ()};
    }

    /** Reconstructs a Pose2d from [x, y, roll, pitch]. */
    private static Pose2d componentsToPose(double[] c) {
        return new Pose2d(new Translation2d(c[0], c[1]), new Rotation2d(c[2]));
    }
}
