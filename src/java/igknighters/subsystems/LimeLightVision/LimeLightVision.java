package igknighters.subsystems.LimeLightVision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLightVisionReal;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLightVisionSim;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLights;
import igknighters.subsystems.Subsystems.SharedSubsystem;
import java.util.List;

public class LimeLightVision implements SharedSubsystem {
    private LimeLights vision;

    public LimeLightVision() {
        if (Robot.isReal()) {
            vision =
                    new LimeLightVisionReal(
                            SubsystemConstants.kLimelightVision.backLeft,
                            SubsystemConstants.kLimelightVision.backRight);
        } else {
            vision = new LimeLightVisionSim();
        }
    }

    public List<Integer> getVisibleTagIds() {
        return vision.getVisibleTagIds();
    }

    public double getLastTimeStamp() {
        return vision.getLastTimeStamp();
    }

    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        DogLog.log("Subsystems/Vison/Limelight/", true);
        return vision.getRobotPoseFromVision(yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }
}
