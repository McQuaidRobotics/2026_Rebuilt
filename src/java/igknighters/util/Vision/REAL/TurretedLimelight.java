package igknighters.util.Vision.REAL;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import igknighters.Robot;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.util.Vision.LocalizationCamera;
import igknighters.util.log.Log;
import java.util.ArrayList;

public class TurretedLimelight extends LocalizationCamera {
    private final String cameraName;
    private final Translation3d turretCenterInRobotSpace;
    private final Translation2d cameraOffsetFromTurretCenter;
    private final double cameraHeight;
    private double lastTimeStamp;
    private double lastDoubleTagTimeStamp;
    private ArrayList<Integer> visibleTagIds;
    private final Rotation3d cameraDefaultRotation;

    public TurretedLimelight(
            String cameraName,
            Translation3d turretCenterInRobotSpace,
            Translation3d cameraRelativeToTurret,
            Rotation3d cameraRotation) {

        this.cameraName = cameraName;
        this.lastTimeStamp = 0.0;
        this.turretCenterInRobotSpace = turretCenterInRobotSpace;
        this.cameraDefaultRotation = cameraRotation;
        this.visibleTagIds = new ArrayList<>();

        // We split the offset into 2D (for rotation) and Z (static height)
        this.cameraOffsetFromTurretCenter =
                new Translation2d(cameraRelativeToTurret.getX(), cameraRelativeToTurret.getY());
        this.cameraHeight = turretCenterInRobotSpace.getZ() + cameraRelativeToTurret.getZ();
    }

    public void update(Angle turretAngle) {
        Rotation2d turretRot = Rotation2d.fromRadians(turretAngle.in(Radians));

        // 1. Calculate new XY translation by rotating the camera offset by the turret's angle
        Translation2d rotatedOffset = cameraOffsetFromTurretCenter.rotateBy(turretRot);

        double finalX = turretCenterInRobotSpace.getX() + rotatedOffset.getX();
        double finalY = turretCenterInRobotSpace.getY() + rotatedOffset.getY();
        double finalZ = cameraHeight;

        // 2. Calculate the new rotation (Robot Yaw = Turret Yaw + Camera Local Yaw)
        double finalRoll = cameraDefaultRotation.getX();
        double finalPitch = cameraDefaultRotation.getY();
        double finalYaw = cameraDefaultRotation.getZ() + turretAngle.in(Degrees);

        // 3. Update Limelight
        LimelightHelpers.setCameraPose_RobotSpace(
                cameraName, finalX, finalY, finalZ, finalRoll, finalPitch, finalYaw);
    }

    public double getLastTimeStamp() {
        return lastTimeStamp;
    }

    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate,
            Angle turretAngle) {

        update(turretAngle);

        visibleTagIds.clear();

        // Feed gyro to Limelight (for MT2)
        LimelightHelpers.SetRobotOrientation(
                cameraName, yaw, yawRate, pitch, pitchRate, roll, rollRate);

        // Get both MT2 and MT1 estimates
        var mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        var mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

        Pose2d robotPose2d = null;

        if (mt2Estimate != null && mt1Estimate != null && mt1Estimate.tagCount > 0) {
            lastTimeStamp = mt2Estimate.timestampSeconds;

            // --- ROTATION SELECTION LOGIC ---
            Rotation2d rotationToUse;
            if (mt1Estimate.tagCount >= 2) {
                rotationToUse = mt1Estimate.pose.getRotation(); // vision rotation
                lastDoubleTagTimeStamp = mt1Estimate.timestampSeconds;
            } else {
                rotationToUse = mt2Estimate.pose.getRotation(); // fallback gyro-based
            }

            for (var fid : mt2Estimate.rawFiducials) {
                visibleTagIds.add(fid.id);
            }

            // MT2 translation + selected rotation
            robotPose2d = new Pose2d(mt2Estimate.pose.getTranslation(), rotationToUse);

            // Optional: log rotation source
            if (!Robot.consts.limelightVision().disableVisionLogs()) {
                Log.log(
                        "Subsystems/Vision/LimeLightVision/Source_" + cameraName,
                        (mt1Estimate.tagCount >= 2) ? "VISION_CORRECTION" : "ROBOT_GYRO_ONLY");
            }
        } else {
            return null;
        }

        return robotPose2d;
    }

    @Override
    public ArrayList<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    @Override
    public double getLastDoubleTagTimeStamp() {
        return lastDoubleTagTimeStamp;
    }
}
