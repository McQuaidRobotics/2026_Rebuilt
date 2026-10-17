package igknighters.util.Prediction;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class VisionFuser {

    // --- TUNABLE TOLERANCES ---
    private static final double TRANSLATION_TOLERANCE_METERS = 0.5;
    private static final double ROTATION_TOLERANCE_RADIANS = 0.2;

    // --- STANDARD DEVIATIONS ---
    // Never trust vision XY more than this (WPILib's default is 0.9; odometry is 0.1).
    private static final double MIN_XY_STD_DEV = 0.3;
    // Same idea for heading from multi-tag MT1 (radians); the Pigeon stays the main source.
    private static final double MIN_THETA_STD_DEV = 0.3;

    /** Container for the fused result to feed into your PoseEstimator. */
    public static class FusedVisionData {
        public final Pose2d fusedPose;
        public final double[] stdDevs; // [x, y, theta]
        public final double averageTimeStamp;

        public FusedVisionData(Pose2d fusedPose, double[] stdDevs, double averageTimeStamp) {
            this.fusedPose = fusedPose;
            this.stdDevs = stdDevs;
            this.averageTimeStamp = averageTimeStamp;
        }
    }

    /**
     * Processes a VisionSnapshot and outputs a single high-confidence Pose2d, dynamic StdDevs, and
     * averaged timestamp.
     */
    public static FusedVisionData getFusedVision(
            VisionSnapshot snapshot, SwerveDriveState currentState) {
        Pose2d[] poses = snapshot.getPose();
        Double[] timestamps = snapshot.getTimestamp();

        if (poses == null || poses.length == 0 || timestamps == null || timestamps.length == 0) {
            return null;
        }

        // 1. Cluster Translations (X, Y) by tracking their array indices
        List<List<Integer>> transClusters = new ArrayList<>();
        for (int i = 0; i < poses.length; i++) {
            boolean added = false;
            for (List<Integer> cluster : transClusters) {
                Translation2d clusterFirstTrans = poses[cluster.get(0)].getTranslation();
                if (clusterFirstTrans.getDistance(poses[i].getTranslation())
                        < TRANSLATION_TOLERANCE_METERS) {
                    cluster.add(i);
                    added = true;
                    break;
                }
            }
            if (!added) {
                List<Integer> newCluster = new ArrayList<>();
                newCluster.add(i);
                transClusters.add(newCluster);
            }
        }

        // 2. Cluster Rotations (Theta) by tracking their array indices. Only cameras that actually
        // measured heading vote; MT2 headings are our own gyro echoed back and would outvote them.
        double[] cameraThetaStdDevs = snapshot.getThetaStdDev();
        List<List<Integer>> rotClusters = new ArrayList<>();
        for (int i = 0; i < poses.length; i++) {
            if (cameraThetaStdDevs[i] >= VisionSnapshot.UNTRUSTED_THETA_STD_DEV) {
                continue;
            }
            boolean added = false;
            for (List<Integer> cluster : rotClusters) {
                Rotation2d clusterFirstRot = poses[cluster.get(0)].getRotation();
                if (Math.abs(clusterFirstRot.minus(poses[i].getRotation()).getRadians())
                        < ROTATION_TOLERANCE_RADIANS) {
                    cluster.add(i);
                    added = true;
                    break;
                }
            }
            if (!added) {
                List<Integer> newCluster = new ArrayList<>();
                newCluster.add(i);
                rotClusters.add(newCluster);
            }
        }

        // 3. Find Best Clusters (Size is priority. Tie-breaker: closeness to current Odometry)
        List<Integer> bestTransCluster =
                getBestTranslationCluster(transClusters, currentState.Pose.getTranslation(), poses);
        List<Integer> bestRotCluster =
                rotClusters.isEmpty()
                        ? List.of()
                        : getBestRotationCluster(
                                rotClusters, currentState.Pose.getRotation(), poses);

        // 4. Calculate Averages of Winning Clusters
        Translation2d averagedTranslation = averageTranslations(bestTransCluster, poses);
        // No heading measurement: echo odometry (its std dev below means it's ignored anyway)
        Rotation2d averagedRotation =
                bestRotCluster.isEmpty()
                        ? currentState.Pose.getRotation()
                        : averageRotations(bestRotCluster, poses);

        // 5. Calculate Timestamps
        // We use the timestamps from the winning translation cluster
        double timestampSum = 0;
        for (int index : bestTransCluster) {
            timestampSum += timestamps[index];
        }
        double averageTimestamp = timestampSum / bestTransCluster.size();

        // 6. Calculate Trust (Standard Deviations)
        double xyStdDev =
                Math.max(MIN_XY_STD_DEV, combinedStdDev(bestTransCluster, snapshot.getXyStdDev()));
        double thetaStdDev =
                bestRotCluster.isEmpty()
                        ? VisionSnapshot.UNTRUSTED_THETA_STD_DEV
                        : Math.max(
                                MIN_THETA_STD_DEV,
                                combinedStdDev(bestRotCluster, cameraThetaStdDevs));

        Pose2d finalPose = new Pose2d(averagedTranslation, averagedRotation);
        double[] finalStdDevs = new double[] {xyStdDev, xyStdDev, thetaStdDev};

        // Matches your new 3-argument constructor perfectly
        return new FusedVisionData(finalPose, finalStdDevs, averageTimestamp);
    }

    // --- HELPER METHODS ---

    /** Averaging n independent cameras shrinks error by sqrt(n), not n^2. */
    private static double combinedStdDev(List<Integer> indices, double[] stdDevs) {
        double sum = 0;
        for (int index : indices) {
            sum += stdDevs[index];
        }
        int n = indices.size();
        return sum / n / Math.sqrt(n);
    }

    private static List<Integer> getBestTranslationCluster(
            List<List<Integer>> clusters, Translation2d currentTranslation, Pose2d[] poses) {
        List<Integer> best = clusters.get(0);
        for (List<Integer> cluster : clusters) {
            if (cluster.size() > best.size()) {
                best = cluster;
            } else if (cluster.size() == best.size()) {
                double distCurrentBest =
                        averageTranslations(best, poses).getDistance(currentTranslation);
                double distNewCluster =
                        averageTranslations(cluster, poses).getDistance(currentTranslation);
                if (distNewCluster < distCurrentBest) {
                    best = cluster;
                }
            }
        }
        return best;
    }

    private static List<Integer> getBestRotationCluster(
            List<List<Integer>> clusters, Rotation2d currentRotation, Pose2d[] poses) {
        List<Integer> best = clusters.get(0);
        for (List<Integer> cluster : clusters) {
            if (cluster.size() > best.size()) {
                best = cluster;
            } else if (cluster.size() == best.size()) {
                double diffCurrentBest =
                        Math.abs(averageRotations(best, poses).minus(currentRotation).getRadians());
                double diffNewCluster =
                        Math.abs(
                                averageRotations(cluster, poses)
                                        .minus(currentRotation)
                                        .getRadians());
                if (diffNewCluster < diffCurrentBest) {
                    best = cluster;
                }
            }
        }
        return best;
    }

    private static Translation2d averageTranslations(List<Integer> indices, Pose2d[] poses) {
        double xSum = 0;
        double ySum = 0;
        for (int index : indices) {
            Translation2d t = poses[index].getTranslation();
            xSum += t.getX();
            ySum += t.getY();
        }
        return new Translation2d(xSum / indices.size(), ySum / indices.size());
    }

    private static Rotation2d averageRotations(List<Integer> indices, Pose2d[] poses) {
        double xSum = 0;
        double ySum = 0;
        for (int index : indices) {
            Rotation2d r = poses[index].getRotation();
            xSum += r.getCos();
            ySum += r.getSin();
        }
        return new Rotation2d(Math.atan2(ySum, xSum));
    }
}
