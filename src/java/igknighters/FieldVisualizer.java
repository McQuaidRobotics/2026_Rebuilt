package igknighters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableBoolean;
import java.util.List;

/**
 * FieldVisualizer provides methods to update specific objects on the field (Field2d) independently
 * of the main telemetry loop. This allows for updating vision targets, detected objects, and other
 * field elements only when necessary.
 */
public class FieldVisualizer {

    // Create the single master Field2d instance
    private final Field2d m_field = new Field2d();

    private FieldVisualizer() {
        // Publish the unified field to SmartDashboard
        SmartDashboard.putData("Field", m_field);
    }

    private static class SingletonHelper {
        private static final FieldVisualizer INSTANCE = new FieldVisualizer();
    }

    public static FieldVisualizer getInstance() {
        return SingletonHelper.INSTANCE;
    }

    private final TunableBoolean shouldShowBalls =
            TunableValues.getBoolean("FieldVisualizer/ShowBalls", true);
    private final TunableBoolean shouldShowShootingTarget =
            TunableValues.getBoolean("FieldVisualizer/ShowShootingTarget", true);
    private final TunableBoolean shouldShowDrivingTarget =
            TunableValues.getBoolean("FieldVisualizer/ShowDrivingTarget", true);

    /**
     * Updates the main robot pose on the field. Call this from your Telemetry loop.
     *
     * @param robotPose The current odometry pose of the robot.
     */
    public void updateRobotPose(Pose2d robotPose) {
        if (robotPose != null) {
            m_field.setRobotPose(robotPose);
        }
    }

    /**
     * Updates the shooting target pose on the field.
     *
     * @param target The pose of the shooting target, or null to clear.
     */
    public void updateShootingTarget(Pose2d target) {
        var obj = m_field.getObject("ShootingTarget");
        if (target == null || !shouldShowShootingTarget.value()) {
            obj.setPoses(); // Clears the object from the field
            return;
        }
        obj.setPose(target);
    }

    /**
     * Updates the predicted future pose of the robot on the field. * @param pred_pose The predicted
     * future pose, or null to clear.
     */
    public void updatePredictedPose(Pose2d pred_pose) {
        var obj = m_field.getObject("FuturePose");
        if (pred_pose == null) {
            obj.setPoses();
            return;
        }
        obj.setPose(pred_pose);
    }

    /**
     * Updates the driving target pose on the field.
     *
     * @param target The pose of the driving target, or null to clear.
     */
    public void updateDrivingTarget(Pose2d target) {
        var obj = m_field.getObject("DrivingTarget");
        if (target == null || !shouldShowDrivingTarget.value()) {
            obj.setPoses();
            return;
        }
        obj.setPose(target);
    }

    /**
     * Updates the turret orientation relative to the field. * @param turretAngleDegrees The
     * rotation of the turret relative to the robot chassis.
     *
     * @param robotPose The current position of the robot.
     */
    public void updateTurret(double turretAngleDegrees, Pose2d robotPose) {
        var obj = m_field.getObject("Turret");
        if (robotPose == null) {
            obj.setPoses();
            return;
        }
        Pose2d turretPose =
                new Pose2d(
                        robotPose.getX(),
                        robotPose.getY(),
                        robotPose.getRotation().plus(Rotation2d.fromDegrees(turretAngleDegrees)));
        obj.setPose(turretPose);
    }

    /**
     * Updates the list of detected objects (e.g., game pieces) on the field.
     *
     * @param objects A list of poses for detected objects, or null/empty to clear.
     */
    public void updateDetectedObjects(List<Pose2d> objects) {
        var obj = m_field.getObject("DetectedObjects");
        if (objects == null || objects.isEmpty() || !shouldShowBalls.value()) {
            obj.setPoses();
            return;
        }
        obj.setPoses(objects);
    }

    /**
     * Updates the field with seen tags dynamically split into numbered layers (max 8 per layer).
     *
     * @param seenChunks A list containing split chunks of seen tag poses.
     */
    public void updateSeenTagsSplit(List<List<Pose2d>> seenChunks) {
        // 1. Loop through and dynamically update SeenTags1, SeenTags2, etc.
        for (int i = 0; i < seenChunks.size(); i++) {
            String layerName = "SeenTags" + (i + 1);
            m_field.getObject(layerName).setPoses(seenChunks.get(i));
        }

        // 2. Clear out any leftover higher-numbered layers from previous cycles
        int layerCheck = seenChunks.size() + 1;
        while (true) {
            var oldObj = m_field.getObject("SeenTags" + layerCheck);
            if (oldObj.getPoses().size() > 0) {
                oldObj.setPoses();
                layerCheck++;
            } else {
                break;
            }
        }
    }

    /**
     * Updates all vision-related targets on the field in one call.
     *
     * @param detectedObjects List of detected object poses.
     * @param shootingTarget Shooting target pose.
     * @param drivingTarget Driving target pose.
     */
    public void updateVisionTargets(
            List<Pose2d> detectedObjects, Pose2d shootingTarget, Pose2d drivingTarget) {
        updateDetectedObjects(detectedObjects);
        updateShootingTarget(shootingTarget);
        updateDrivingTarget(drivingTarget);
    }
}
