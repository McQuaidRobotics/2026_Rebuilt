package igknighters.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import igknighters.Robot;
import igknighters.util.log.Log;
import java.util.Arrays;
import java.util.Collections;

public class TurretPosePredError {
    Pose3d[] predictionHistory = new Pose3d[2];
    double[] timestampHistory = new double[2];
    int writeIndex = 0;

    public TurretPosePredError() {
        for (int i = 0; i < 2; i++) {
            predictionHistory[i] = new Pose3d(0, 0, 0, new Rotation3d());
        }
    }

    public void logPose(Pose3d currentPose) {
        predictionHistory[writeIndex] = Robot.turret_pred.getPredictedPose().get();
        timestampHistory[writeIndex] = Timer.getFPGATimestamp();
        if (writeIndex != 1) {
            writeIndex++;
        } else {
            writeIndex = 0;
        }
        // Log.log("ROBOT/tsdifference", timestampHistory[0] - timestampHistory[1]);
    }

    public double[] findError(Pose3d actualPose) {
        double prevTimestamp = Collections.min(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx = Arrays.stream(timestampHistory).boxed().toList().indexOf(prevTimestamp);
        double actualOmega = actualPose.getZ();
        double predOmega = predictionHistory[latestIdx].getZ();
        double[] errors = new double[3];
        errors[0] =
                Math.abs(
                        (actualPose.getX() - predictionHistory[latestIdx].getX())
                                / actualPose.getX());
        errors[1] =
                Math.abs(
                        (actualPose.getY() - predictionHistory[latestIdx].getY())
                                / actualPose.getY());
        if (actualOmega > Math.PI) {
            actualOmega = actualOmega - 2 * Math.PI;
        } else if (actualOmega < -Math.PI) {
            actualOmega = actualOmega - 2 * Math.PI;
        }

        if (predOmega > Math.PI) {
            predOmega = predOmega - 3 * Math.PI / 4;
        } else if (predOmega < -Math.PI) {
            predOmega = predOmega - 3 * Math.PI / 4;
        }
        errors[2] = Math.abs((actualOmega - predOmega) / actualOmega);
        Log.log("ROBOT/turret_pose_pred_error", errors);
        return errors;
    }
}
