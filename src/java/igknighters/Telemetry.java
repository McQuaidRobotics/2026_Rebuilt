package igknighters;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.subsystems.Subsystems;
import igknighters.util.AprilTagLayout;
import java.io.IOException;
import java.util.List;
import java.util.Map;

/**
 * Handles the collection and publishing of robot telemetry data. This class is responsible for
 * sending robot pose, swerve module states, vision targets, and other diagnostic information to
 * NetworkTables and logs.
 *
 * <p>It facilitates real-time visualization in tools like AdvantageScope and SmartDashboard, and
 * ensures high-frequency data is captured by SignalLogger.
 */
public class Telemetry {
    private final double MaxSpeed;
    private final Subsystems subsystems;
    private AprilTagLayout aprilTagLayout;

    /**
     * Constructs a Telemetry object with the specified maximum robot speed.
     *
     * @param maxSpeed Maximum theoretical speed in meters per second.
     * @param subsystems The robot subsystems for context.
     */
    public Telemetry(double maxSpeed, Subsystems subsystems) {
        MaxSpeed = maxSpeed;
        this.subsystems = subsystems;
        try {
            aprilTagLayout = new AprilTagLayout();
        } catch (IOException e) {
            e.printStackTrace();
        }
        SignalLogger.start();
    }

    /* NetworkTables instance for publishing data */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Swerve drive state publishers */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose =
            driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds =
            driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates =
            driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets =
            driveStateTable
                    .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
                    .publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
            driveStateTable
                    .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
                    .publish();
    private final DoublePublisher driveTimestamp =
            driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency =
            driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Field positioning publishers for AdvantageScope visualization */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();
    private final DoubleArrayPublisher seenTagsPub =
            table.getDoubleArrayTopic("seenTags").publish();
    private final DoubleArrayPublisher unseenTagsPub =
            table.getDoubleArrayTopic("unseenTags").publish();

    private final DoubleArrayPublisher shootingTargetPosesPub =
            table.getDoubleArrayTopic("shootingTargetPose").publish();

    private final DoubleArrayPublisher drivingTargetPub =
            table.getDoubleArrayTopic("drivingTargetPose").publish();

    private final DoubleArrayPublisher detectedObjectsPub =
            table.getDoubleArrayTopic("detectedObjects").publish();

    /* Mechanism2d visualizers for swerve modules */
    private final Mechanism2d[] m_moduleMechanisms =
            new Mechanism2d[] {
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
            };

    private final MechanismLigament2d[] m_moduleSpeeds =
            new MechanismLigament2d[] {
                m_moduleMechanisms[0]
                        .getRoot("RootSpeed", 0.5, 0.5)
                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                m_moduleMechanisms[1]
                        .getRoot("RootSpeed", 0.5, 0.5)
                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                m_moduleMechanisms[2]
                        .getRoot("RootSpeed", 0.5, 0.5)
                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                m_moduleMechanisms[3]
                        .getRoot("RootSpeed", 0.5, 0.5)
                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
            };

    private final MechanismLigament2d[] m_moduleDirections =
            new MechanismLigament2d[] {
                m_moduleMechanisms[0]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                m_moduleMechanisms[1]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                m_moduleMechanisms[2]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                m_moduleMechanisms[3]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            };

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    /**
     * Processes the current swerve drive state and publishes it to all active sinks. This includes
     * NetworkTables for live visualization and SignalLogger for high-speed data capture.
     *
     * @param state The current {@link SwerveDriveState} from the drivetrain.
     */
    public void telemeterize(SwerveDriveState state) {
        /* Publish to NetworkTables */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Write to high-speed log file */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Update field visualization */
        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);

        // Visualize seen/unseen AprilTags if layout is available.
        if (aprilTagLayout != null) {
            List<Integer> visibleIds = subsystems.vision.getVisibleTagIds();
            Map<Integer, Pose3d> allTagPoses = aprilTagLayout.getTagPoses();
            List<Pose2d> seenTagPoses = new java.util.ArrayList<>();
            List<Pose2d> unseenTagPoses = new java.util.ArrayList<>();

            for (Map.Entry<Integer, Pose3d> entry : allTagPoses.entrySet()) {
                if (visibleIds.contains(entry.getKey())) {
                    seenTagPoses.add(entry.getValue().toPose2d());
                } else {
                    unseenTagPoses.add(entry.getValue().toPose2d());
                }
            }

            double[] seenTagsArray = new double[seenTagPoses.size() * 3];
            int i = 0;
            for (Pose2d pose : seenTagPoses) {
                seenTagsArray[i++] = pose.getX();
                seenTagsArray[i++] = pose.getY();
                seenTagsArray[i++] = pose.getRotation().getDegrees();
            }
            seenTagsPub.set(seenTagsArray);

            double[] unseenTagsArray = new double[unseenTagPoses.size() * 3];
            i = 0;
            for (Pose2d pose : unseenTagPoses) {
                unseenTagsArray[i++] = pose.getX();
                unseenTagsArray[i++] = pose.getY();
                unseenTagsArray[i++] = pose.getRotation().getDegrees();
            }
            unseenTagsPub.set(unseenTagsArray);
        }

        /* Update swerve module visualizers in SmartDashboard */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(
                    state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            SmartDashboard.putData("Visualizers/Swerve/Module " + i, m_moduleMechanisms[i]);
        }
    }

    /**
     * Publishes a target pose for shooting visualization on the field map.
     *
     * @param targetPose The destination {@link Pose2d}.
     */
    public void addShootingTargetPose(Pose2d targetPose) {
        double[] targetPoseArray = new double[3];
        targetPoseArray[0] = targetPose.getX();
        targetPoseArray[1] = targetPose.getY();
        targetPoseArray[2] = targetPose.getRotation().getDegrees();
        shootingTargetPosesPub.set(targetPoseArray);
    }

    /**
     * Publishes a target pose for driving visualization on the field map.
     *
     * @param targetPose The destination {@link Pose2d}.
     */
    public void addDrivingTargetPose(Pose2d targetPose) {
        double[] targetPoseArray = new double[3];
        targetPoseArray[0] = targetPose.getX();
        targetPoseArray[1] = targetPose.getY();
        targetPoseArray[2] = targetPose.getRotation().getDegrees();
        drivingTargetPub.set(targetPoseArray);
    }

    /**
     * Publishes a list of detected objects (e.g., game pieces) for field visualization.
     *
     * @param objectPoses List of {@link Pose2d} representing detected objects.
     */
    public void publishDetectedObjects(List<Pose2d> objectPoses) {
        double[] objectPosesArray = new double[objectPoses.size() * 3];
        int i = 0;
        for (Pose2d pose : objectPoses) {
            objectPosesArray[i++] = pose.getX();
            objectPosesArray[i++] = pose.getY();
            objectPosesArray[i++] = pose.getRotation().getDegrees();
        }
        detectedObjectsPub.set(objectPosesArray);
    }
}
