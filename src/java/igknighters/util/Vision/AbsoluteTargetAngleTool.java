package igknighters.util.Vision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import java.util.Optional;

public class AbsoluteTargetAngleTool {
    private final String cameraName;

    public AbsoluteTargetAngleTool(String cameraName) {
        this.cameraName = cameraName;
    }

    /**
     * Calculates the absolute angle from the front of the robot to a specific AprilTag. * @param
     * currentTurretAngle The turret's current rotation relative to robot front.
     *
     * @param targetID The AprilTag ID to filter for.
     * @return Optional Rotation2d of the target's position. Empty if ID is not seen.
     */
    public Optional<Rotation2d> getAbsoluteAngleToTag(Angle currentTurretAngle, int targetID) {
        // 1. Check if we see any target
        if (!LimelightHelpers.getTV(cameraName)) {
            return Optional.empty();
        }

        // 2. Check if the target we see is the one we want
        // LimelightHelpers returns the primary target's ID
        int seenID = (int) LimelightHelpers.getFiducialID(cameraName);

        if (seenID != targetID) {
            return Optional.empty();
        }

        /*
         * Limelight tx: Positive = Right (Clockwise), Negative = Left (Counter-Clockwise)
         * WPILib: Positive = Left (CCW), Negative = Right (CW)
         * We negate tx so that "Target to the left" becomes a positive addition.
         */
        double txStandard = -LimelightHelpers.getTX(cameraName);

        double robotRelativeAngle = currentTurretAngle.in(Degrees) + txStandard;

        return Optional.of(Rotation2d.fromDegrees(robotRelativeAngle));
    }
}
