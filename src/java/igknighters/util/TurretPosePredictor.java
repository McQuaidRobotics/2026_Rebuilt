package igknighters.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import igknighters.Robot;
import igknighters.constants.Conv;
import igknighters.util.log.Log;
import java.util.Arrays;
import java.util.Collections;
import java.util.function.Supplier;

public class TurretPosePredictor {

    Pose3d[] currentPose = new Pose3d[3];
    int writeIdx = 0;
    double[] timestampHistory = new double[3];

    public void logTurretPose(Pose3d turretPose) {
        double now = Timer.getFPGATimestamp();
        writeIdx++;
        if (writeIdx > 2) {
            writeIdx = 0;
        }
        currentPose[writeIdx] = turretPose;
        timestampHistory[writeIdx] = now;
    }

    // will return field-relative turret position (robot pose with offset)
    // currently, the rotation is not predicted but is assumed to be the same, as predicting the
    // rotation of the turret may require rewriting shooter commands
    public Supplier<Pose3d> getPredictedPose() {

        Pose2d predRobotPose = Robot.pose_pred.getDynamicPredictedPose();
        Log.log("ROBOT/pose", predRobotPose);
        double mostRecentTimestamp =
                Collections.max(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx =
                Arrays.stream(timestampHistory).boxed().toList().indexOf(mostRecentTimestamp);
        Pose3d predTurretPose = getTurretPoseFieldRelativeOffset(predRobotPose);
        Robot.turret_pred_error.findError(currentPose[latestIdx]);
        return () -> predTurretPose;
    }

    public Supplier<Pose2d> getPredictedPose2d() {
        return () -> getPredictedPose().get().toPose2d();
    }

    public Supplier<ChassisSpeeds> getPredictedVelos() {
        ChassisSpeeds predRobotVelos = Robot.pose_pred.getDynamicPredictedSpeeds();
        ChassisSpeeds predTurretVelos = new ChassisSpeeds();
        predTurretVelos.vxMetersPerSecond =
                predRobotVelos.vxMetersPerSecond
                        - predRobotVelos.omegaRadiansPerSecond * 5 * Conv.INCHES_TO_METERS;
        predTurretVelos.vyMetersPerSecond =
                predRobotVelos.vyMetersPerSecond
                        + predRobotVelos.omegaRadiansPerSecond * 5 * Conv.INCHES_TO_METERS;
        return () -> predTurretVelos;
    }

    public Pose3d getTurretPoseFieldRelativeOffset(Pose2d robotPose) {
        double xMeterOffset =
                Robot.consts.shooter().kTurret().TURRET_ROBOT_DISTANCE_FROM_CENTERS()
                        * Conv.INCHES_TO_METERS
                        * Math.cos(robotPose.getRotation().getRadians() - 3 * Math.PI / 4);
        double yMeterOffset =
                Robot.consts.shooter().kTurret().TURRET_ROBOT_DISTANCE_FROM_CENTERS()
                        * Conv.INCHES_TO_METERS
                        * Math.sin(robotPose.getRotation().getRadians() - 3 * Math.PI / 4);
        double zMeterOffset = 0.3; // Height of the turret from the ground
        return new Pose3d(
                robotPose.getX() + xMeterOffset,
                robotPose.getY() + yMeterOffset,
                zMeterOffset,
                new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
    }
}
