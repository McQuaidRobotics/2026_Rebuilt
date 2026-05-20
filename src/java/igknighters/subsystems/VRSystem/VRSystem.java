package igknighters.subsystems.VRSystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.subsystems.swerve.Swerve;

/**
 * Subsystem to handle telemetry for the VR Headset.
 * It provides the robot's heading and pose to NetworkTables so the VR client
 * can orient the 360 video and display a mini-map or overlays.
 */
public class VRSystem extends SubsystemBase {
    private final Swerve swerve;
    private final NetworkTable table;
    
    private final DoublePublisher headingPub;
    private final StructPublisher<Pose2d> posePub;

    public VRSystem(Swerve swerve) {
        this.swerve = swerve;
        this.table = NetworkTableInstance.getDefault().getTable("VRTelemetry");
        
        this.headingPub = table.getDoubleTopic("headingDegrees").publish();
        this.posePub = table.getStructTopic("robotPose", Pose2d.struct).publish();
    }

    @Override
    public void periodic() {
        // Publish heading for video orientation stabilization
        headingPub.set(swerve.getState().Pose.getRotation().getDegrees());
        
        // Publish pose for a mini-map in VR
        posePub.set(swerve.getState().Pose);
    }

    @Override
    public String getName() {
        return "VRSystem";
    }
}
