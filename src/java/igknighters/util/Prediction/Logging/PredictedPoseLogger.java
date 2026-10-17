package igknighters.util.Prediction.Logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import igknighters.util.Prediction.Localizer;
import igknighters.util.log.Log;
import org.littletonrobotics.junction.Logger;

public class PredictedPoseLogger {
    private static final double LOOKAHEAD_SECONDS = .05;

    // Predictions keyed by the time they predict, so we can compare them once that time arrives.
    private static final TimeInterpolatableBuffer<Pose2d> predictions =
            TimeInterpolatableBuffer.createBuffer(0.5);

    /** Call once per loop after Localizer.update(). */
    public static void log(Pose2d actualPose) {
        double now = Timer.getFPGATimestamp();
        Pose2d predicted = Localizer.getInstance().getPredictedPose(LOOKAHEAD_SECONDS);
        Logger.recordOutput("PREDICTED ROBOT POSE", predicted);

        // Compare against what we predicted LOOKAHEAD_SECONDS ago for right now.
        var pastPrediction = predictions.getSample(now);
        predictions.addSample(now + LOOKAHEAD_SECONDS, predicted);
        if (pastPrediction.isEmpty()) {
            return;
        }

        double errorX = pastPrediction.get().getX() - actualPose.getX();
        double errorY = pastPrediction.get().getY() - actualPose.getY();
        double errorTheta =
                pastPrediction.get().getRotation().minus(actualPose.getRotation()).getRadians();
        Log.log("ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/DETAILS/PREDICTED POSE ERROR X", errorX);
        Log.log("ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/DETAILS/PREDICTED POSE ERROR Y", errorY);
        Log.log(
                "ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/DETAILS/PREDICTED POSE ERROR THETA",
                errorTheta);

        Log.log(
                "ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/SUMMARY/IS X ERROR AGRETIOUS",
                Math.abs(errorX) > .3);
        Log.log(
                "ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/SUMMARY/IS Y ERROR AGRETIOUS",
                Math.abs(errorY) > .3);
        Log.log(
                "ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/SUMMARY/IS THETA ERROR AGRETIOUS",
                Math.abs(errorTheta) > 1);
    }
}
