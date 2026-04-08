package igknighters.util.Merging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PoseMerger {
    public static Pose2d simpleMerge(Pose2d pose1, Pose2d pose2, double weight1, double weight2) {
        return new Pose2d(
                (pose1.getX() * weight1 + pose2.getX() * weight2) / (weight1 + weight2),
                (pose1.getY() * weight1 + pose2.getY() * weight2) / (weight1 + weight2),
                new Rotation2d(
                        (pose1.getRotation().getRadians() * weight1
                                        + pose2.getRotation().getRadians() * weight2)
                                / (weight1 + weight2)));
    }

    /**
     * Merges two poses with a trusted approach, giving more weight to the first pose.
     *
     * @param pose1 The first pose (assumed to be more trusted).
     * @param pose2 The second pose.
     * @return The merged pose.
     */
    public static Pose2d trustedMerge(Pose2d pose1, Pose2d pose2) {
        // pose 1 trusted
        // we should use the pose 1 values and use an exponentially declined trust of pose2

        double x1 = pose1.getX();
        double y1 = pose1.getY();
        double theta1 = pose1.getRotation().getRadians();

        double x2 = pose2.getX();
        double y2 = pose2.getY();
        double theta2 = pose2.getRotation().getRadians();

        double dx = x1 - x2;
        double dy = y1 - y2;
        double dtheta = theta1 - theta2;

        double modifierX = dx * Math.exp(-Math.abs(dx));
        double modifierY = dy * Math.exp(-Math.abs(dy));
        double modifierTheta = dtheta * Math.exp(-Math.abs(dtheta));

        double newX = x1 + modifierX;
        double newY = y1 + modifierY;
        double newTheta = theta1 + modifierTheta;
        return new Pose2d(newX, newY, new Rotation2d(newTheta));
    }
}
