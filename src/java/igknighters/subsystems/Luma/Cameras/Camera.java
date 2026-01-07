package igknighters.subsystems.Luma.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.subsystems.Subsystems.SharedSubsystem;

public abstract class Camera implements SharedSubsystem {
    public abstract void periodic();

    public abstract void simulationPeriodic();

    public abstract String getName();

    public abstract Translation2d getGamePieceOffset();

    public abstract Pose2d getRobotPose();
}
