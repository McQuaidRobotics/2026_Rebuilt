package igknighters.util.Prediction.Logging;

import igknighters.util.Prediction.Localizer;
import igknighters.util.log.Log;
import org.littletonrobotics.junction.Logger;

public class PredictedPoseLogger {

    public static void logPredictedPose() {
        Logger.recordOutput("PREDICTED ROBOT POSE", Localizer.getInstance().getPredictedPose(.05));
    }

    public static void logErrors() {
        double errorX =
                Localizer.getInstance().getPredictedPose(.05).getX()
                        - Localizer.getInstance().getSwerveState().Pose.getX();
        double errorY =
                Localizer.getInstance().getPredictedPose(.05).getY()
                        - Localizer.getInstance().getSwerveState().Pose.getY();
        double errorTheta =
                Localizer.getInstance().getPredictedPose(.05).getRotation().getRadians()
                        - Localizer.getInstance().getSwerveState().Pose.getRotation().getRadians();
        Log.log("ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/DETAILS/PREDICTED POSE ERROR X", errorX);
        Log.log("ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/DETAILS/PREDICTED POSE ERROR Y", errorY);
        Log.log(
                "ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/DETAILS/PREDICTED POSE ERROR THETA",
                errorTheta);

        Log.log("ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/SUMMARY/IS X ERROR AGRETIOUS", errorX > .3);
        Log.log("ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/SUMMARY/IS Y ERROR AGRETIOUS", errorY > .3);
        Log.log(
                "ROBOT/UTILITIES/POSE_PREDICTOR/ERRORS/SUMMARY/IS THETA ERROR AGRETIOUS",
                errorTheta > 1);
    }
}
