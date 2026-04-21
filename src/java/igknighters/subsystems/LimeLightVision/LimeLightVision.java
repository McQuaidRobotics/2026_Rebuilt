package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.util.Vision.REAL.StaticCamera;
import igknighters.util.Vision.SIM.SimCamera;
import igknighters.util.log.Log;
import java.util.List;

public class LimeLightVision extends SubsystemBase {
    private VisionOrchestrator vision;

    public LimeLightVision() {
        if (Robot.isReal()) {
            vision =
                    new VisionOrchestrator(
                            List.of(
                                    new StaticCamera(Robot.consts.limelightVision().intakeCam()),
                                    new StaticCamera(Robot.consts.limelightVision().backCam()),
                                    new StaticCamera(Robot.consts.limelightVision().rightCam()),
                                    new StaticCamera(Robot.consts.limelightVision().turretCam())));
        } else {
            vision =
                    new VisionOrchestrator(
                            List.of(
                                    new SimCamera("Intake Cam", -45, 45),
                                    new SimCamera("Back Cam", 45, 135),
                                    new SimCamera("Right Cam", 135, 225),
                                    new SimCamera("Turret Cam", 225, 315)));
        }
    }

    public List<Integer> getVisibleTagIds() {
        return vision.getVisibleTagIds();
    }

    public void enableCameras(int IMU_MODE) {
        vision.enableCameras(IMU_MODE);
    }

    public void disableCameras() {
        vision.saveCameras();
    }

    public double getLastTimeStamp() {
        return vision.getTimeStamp();
    }

    public double timeSinceLastSample() {
        return vision.getTimeSinceLastUpdate();
    }

    // if no turret just put whatever you want in for angle it will not matter it is only used for
    // the turretLocalizer class

    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate,
            Angle turretAngle) {
        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vison/Limelight/ENABLED", true);
        }
        return vision.getPose(yaw, yawRate, pitch, pitchRate, roll, rollRate, turretAngle);
    }
}
