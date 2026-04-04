package igknighters.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    public Pose3d getPredictedPose() {
        Pose2d predRobotPose = Robot.pose_pred.getPredictedPose();
        Log.log("ROBOT/pose", predRobotPose);
        double xOffset = getXTurretOffsetFieldRelative(predRobotPose.getRotation().getRadians());
        double yOffset = getYTurretOffsetFieldRelative(predRobotPose.getRotation().getRadians());
        double mostRecentTimestamp =
                Collections.max(Arrays.stream(timestampHistory).boxed().toList());
        int latestIdx =
                Arrays.stream(timestampHistory).boxed().toList().indexOf(mostRecentTimestamp);
        Pose3d predTurretPose =
                new Pose3d(
                        predRobotPose.getX(),
                        predRobotPose.getY(),
                        .3,
                        currentPose[latestIdx].getRotation());

        return predTurretPose;
    }

    // public Supplier<Pose2d> getPredictedPose2d() {
    //     return () -> getPredictedPose().get().toPose2d();
    // }

    public Supplier<ChassisSpeeds> getPredictedVelos() {
        ChassisSpeeds predRobotVelos = Robot.pose_pred.getPredictedVelos();
        ChassisSpeeds predTurretVelos = new ChassisSpeeds();
        // 5 MAY HAVE TO BE FIELD RELATIVE X
        predTurretVelos.vxMetersPerSecond =
                predRobotVelos.vxMetersPerSecond
                        - predRobotVelos.omegaRadiansPerSecond * 5 * Conv.INCHES_TO_METERS;
        // 5 MAY HAVE TO BE FIELD RELATIVE Y
        predTurretVelos.vyMetersPerSecond =
                predRobotVelos.vyMetersPerSecond
                        + predRobotVelos.omegaRadiansPerSecond * 5 * Conv.INCHES_TO_METERS;
        return () -> predTurretVelos;
    }

    private double getXTurretOffsetFieldRelative(double robotRotation) {
        double xOffset = 5 * Conv.INCHES_TO_METERS * Math.cos(robotRotation);
        return xOffset;
    }

    private double getYTurretOffsetFieldRelative(double robotRotation) {
        double yOffset = 5 * Conv.INCHES_TO_METERS * Math.sin(robotRotation);
        return yOffset;
    }
}
