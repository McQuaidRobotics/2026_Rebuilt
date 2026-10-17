package igknighters.util.Prediction;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

public class VisionFuserTest {
    private static final double UNTRUSTED = VisionSnapshot.UNTRUSTED_THETA_STD_DEV;
    private static final SwerveDriveState STATE = new SwerveDriveState();

    private static VisionFuser.FusedVisionData fuse(
            Pose2d[] poses, double[] xyStdDevs, double[] thetaStdDevs) {
        Double[] ts = new Double[poses.length];
        java.util.Arrays.fill(ts, 1.0);
        return VisionFuser.getFusedVision(
                new VisionSnapshot(poses, ts, xyStdDevs, thetaStdDevs), STATE);
    }

    @Test
    void oneCameraKeepsItsOwnStdDevs() {
        var fused =
                fuse(
                        new Pose2d[] {new Pose2d(3, 3, Rotation2d.kZero)},
                        new double[] {0.9},
                        new double[] {0.9});
        assertEquals(0.9, fused.stdDevs[0], 1e-9);
        assertEquals(0.9, fused.stdDevs[2], 1e-9);
    }

    @Test
    void agreeingCamerasShrinkBySqrtN() {
        var fused =
                fuse(
                        new Pose2d[] {
                            new Pose2d(3, 3, Rotation2d.kZero), new Pose2d(3.1, 3, Rotation2d.kZero)
                        },
                        new double[] {0.9, 0.9},
                        new double[] {0.9, 0.9});
        assertEquals(0.9 / Math.sqrt(2), fused.stdDevs[0], 1e-9);
        assertEquals(0.9 / Math.sqrt(2), fused.stdDevs[2], 1e-9);
    }

    @Test
    void neverTrustedMoreThanFloor() {
        var fused =
                fuse(
                        new Pose2d[] {
                            new Pose2d(3, 3, Rotation2d.kZero), new Pose2d(3, 3, Rotation2d.kZero)
                        },
                        new double[] {0.1, 0.1},
                        new double[] {0.1, 0.1});
        assertEquals(0.3, fused.stdDevs[0], 1e-9);
        assertEquals(0.3, fused.stdDevs[2], 1e-9);
    }

    @Test
    void echoedMt2HeadingsDoNotOutvoteMeasuredHeading() {
        // Two MT2 cameras echo a 180-deg-wrong gyro; one MT1 multi-tag camera measures the truth.
        var fused =
                fuse(
                        new Pose2d[] {
                            new Pose2d(3, 3, Rotation2d.kPi),
                            new Pose2d(3, 3, Rotation2d.kPi),
                            new Pose2d(3, 3, Rotation2d.kZero)
                        },
                        new double[] {0.9, 0.9, 0.9},
                        new double[] {UNTRUSTED, UNTRUSTED, 0.9});
        assertEquals(0, fused.fusedPose.getRotation().getRadians(), 1e-9);
        assertEquals(0.9, fused.stdDevs[2], 1e-9);
    }

    @Test
    void noMeasuredHeadingMeansHeadingIgnored() {
        var fused =
                fuse(
                        new Pose2d[] {new Pose2d(3, 3, Rotation2d.kPi)},
                        new double[] {0.9},
                        new double[] {UNTRUSTED});
        assertEquals(UNTRUSTED, fused.stdDevs[2], 1e-9);
    }
}
