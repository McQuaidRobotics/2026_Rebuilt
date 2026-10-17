package igknighters.util.Merging;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

public class PoseMergerTest {
    @Test
    void trustedMergePullsTowardSecondPose() {
        Pose2d merged =
                PoseMerger.trustedMerge(
                        new Pose2d(5.0, 0, new Rotation2d()), new Pose2d(5.2, 0, new Rotation2d()));
        assertTrue(merged.getX() > 5.0 && merged.getX() < 5.2, "x=" + merged.getX());
    }

    @Test
    void trustedMergeWrapsHeading() {
        // 3.1 and -3.1 rad are only ~0.08 rad apart across the +/-pi seam.
        Pose2d merged =
                PoseMerger.trustedMerge(
                        new Pose2d(0, 0, new Rotation2d(3.1)),
                        new Pose2d(0, 0, new Rotation2d(-3.1)));
        assertEquals(0.0, merged.getRotation().minus(new Rotation2d(Math.PI)).getRadians(), 0.1);
    }
}
