package igknighters.util.Prediction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A class for predicting the pose and velocities of the apparatus (e.g., shooter) based on the
 * robot's predicted pose and velocity state.
 */
public class AparatusPosePredictor {
    private final Translation2d offset;

    /**
     * Creates a new apparatus pose predictor with the specified offset.
     *
     * @param offset The offset of the apparatus relative to the robot's physical center.
     */
    public AparatusPosePredictor(Translation2d offset) {
        this.offset = offset;
    }

    /** Predicts the global field position (Pose2d) of the apparatus. */
    public Pose2d getPredictedPose(double lookaheadTimeSeconds) {
        Pose2d robotPredictedPose = Localizer.getInstance().getPredictedPose(lookaheadTimeSeconds);

        // Transform2d handles rotating the translation offset relative to the robot's predicted
        // heading
        return robotPredictedPose.plus(new Transform2d(offset, new Rotation2d()));
    }

    /**
     * Predicts the field-centric velocities (ChassisSpeeds) of the apparatus. This compensates for
     * tangential linear velocities induced by chassis rotation.
     */
    public ChassisSpeeds getPredictedSpeeds(double lookaheadTimeSeconds) {
        // Assuming this returns FIELD-CENTRIC speeds
        ChassisSpeeds robotPredictedSpeeds =
                Localizer.getInstance().getPredictedVelocity(lookaheadTimeSeconds);

        // We need the predicted robot heading to align the robot-centric tangential
        // velocity with the field-centric base velocity.
        Rotation2d predictedHeading =
                Localizer.getInstance().getPredictedPose(lookaheadTimeSeconds).getRotation();

        double omega = robotPredictedSpeeds.omegaRadiansPerSecond;

        // Field-centric linear velocity contributions from chassis translation
        double baseVxFc = robotPredictedSpeeds.vxMetersPerSecond;
        double baseVyFc = robotPredictedSpeeds.vyMetersPerSecond;

        // Robot-centric linear velocity contributions from chassis rotation (w x r)
        double tangentialVxRc = -omega * offset.getY();
        double tangentialVyRc = omega * offset.getX();

        // Rotate the robot-centric tangential velocity into the field-centric frame
        Translation2d tangentialVelocityRc = new Translation2d(tangentialVxRc, tangentialVyRc);
        Translation2d tangentialVelocityFc = tangentialVelocityRc.rotateBy(predictedHeading);

        // Fuse field-centric translation and field-centric rotation velocities together
        double apparatusSpeedX = baseVxFc + tangentialVelocityFc.getX();
        double apparatusSpeedY = baseVyFc + tangentialVelocityFc.getY();

        // Return the final field-centric ChassisSpeeds
        return new ChassisSpeeds(apparatusSpeedX, apparatusSpeedY, omega);
    }
}
