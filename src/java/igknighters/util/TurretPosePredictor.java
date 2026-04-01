package igknighters.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import igknighters.Robot;
import igknighters.constants.Conv;
import java.util.function.Supplier;

public class TurretPosePredictor {

    Pose3d currentPose;

    public void logTurretPose(Pose3d turretPose) {
        currentPose = turretPose;
    }

    // will return field-relative turret position (robot pose with offset)
    // currently, the rotation is not predicted but is assumed to be the same, as predicting the
    // rotation of the turret may require rewriting shooter commands
    public Supplier<Pose3d> getPredictedPose() {
        if (currentPose == null) {
            currentPose = new Pose3d();
        }
        Pose2d predRobotPose = Robot.pose_pred.getPredictedPose();
        double xOffset = getXTurretOffsetFieldRelative(predRobotPose.getRotation().getRadians());
        double yOffset = getYTurretOffsetFieldRelative(predRobotPose.getRotation().getRadians());
        Pose3d predTurretPose =
                new Pose3d(
                        predRobotPose.getX() - xOffset,
                        predRobotPose.getY() - yOffset,
                        Robot.consts.shooter().kFlywheels().ShooterHeightMeters(),
                        currentPose.getRotation());

        return () -> predTurretPose;
    }

    public Supplier<Pose2d> getPredictedPose2d() {
        return () -> getPredictedPose().get().toPose2d();
    }

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
