package igknighters;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import java.util.Optional;

public class Telemetry {
    private final double MaxSpeed;
    private final Subsystems subsystems;
    public SwerveDriveState latestState = new SwerveDriveState();
    private AprilTagLayout aprilTagLayout;

    // Create the Field2d instance
    private final Field2d m_field = new Field2d();

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
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

        // Publish the Field2d widget to SmartDashboard so Glass/AdvantageScope can see it
        SmartDashboard.putData("Field", m_field);

        latestState.ModuleStates =
                new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
                };
        latestState.Pose = new edu.wpi.first.math.geometry.Pose2d();

        SmartDashboard.putData(
                "Swerve Drive",
                new Sendable() {
                    @Override
                    public void initSendable(SendableBuilder builder) {
                        builder.setSmartDashboardType("SwerveDrive");

                        // Front Left (Index 0 in CTRE Phoenix 6)
                        builder.addDoubleProperty(
                                "Front Left Angle",
                                () ->
                                        latestState.ModuleStates != null
                                                ? latestState.ModuleStates[0].angle.getDegrees()
                                                : 0.0,
                                null);
                        builder.addDoubleProperty(
                                "Front Left Velocity",
                                () ->
                                        latestState.ModuleStates != null
                                                ? latestState.ModuleStates[0].speedMetersPerSecond
                                                : 0.0,
                                null);

                        // Front Right (Index 1)
                        builder.addDoubleProperty(
                                "Front Right Angle",
                                () ->
                                        latestState.ModuleStates != null
                                                ? latestState.ModuleStates[1].angle.getDegrees()
                                                : 0.0,
                                null);
                        builder.addDoubleProperty(
                                "Front Right Velocity",
                                () ->
                                        latestState.ModuleStates != null
                                                ? latestState.ModuleStates[1].speedMetersPerSecond
                                                : 0.0,
                                null);

                        // Back Left (Index 2)
                        builder.addDoubleProperty(
                                "Back Left Angle",
                                () ->
                                        latestState.ModuleStates != null
                                                ? latestState.ModuleStates[2].angle.getDegrees()
                                                : 0.0,
                                null);
                        builder.addDoubleProperty(
                                "Back Left Velocity",
                                () ->
                                        latestState.ModuleStates != null
                                                ? latestState.ModuleStates[2].speedMetersPerSecond
                                                : 0.0,
                                null);

                        // Back Right (Index 3)
                        builder.addDoubleProperty(
                                "Back Right Angle",
                                () ->
                                        latestState.ModuleStates != null
                                                ? latestState.ModuleStates[3].angle.getDegrees()
                                                : 0.0,
                                null);
                        builder.addDoubleProperty(
                                "Back Right Velocity",
                                () ->
                                        latestState.ModuleStates != null
                                                ? latestState.ModuleStates[3].speedMetersPerSecond
                                                : 0.0,
                                null);

                        // Heading / Gyro orientation
                        builder.addDoubleProperty(
                                "Robot Angle",
                                () ->
                                        latestState.Pose != null
                                                ? latestState.Pose.getRotation().getRadians()
                                                : 0.0,
                                null);
                    }
                });
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
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
    private final DoublePublisher matchTime = driveStateTable.getDoubleTopic("MatchTime").publish();
    private final BooleanPublisher hubActive =
            driveStateTable.getBooleanTopic("HubActive").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms =
            new Mechanism2d[] {
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
                new Mechanism2d(1, 1),
            };
    /* A direction and length changing ligament for speed representation */
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
    /* A direction changing and length constant ligament for module direction */
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

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        matchTime.set(DriverStation.getMatchTime());
        hubActive.set(isHubActive());
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        latestState = state;
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
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

        /* Update the main robot pose on our Field2d object */
        FieldVisualizer.getInstance().updateRobotPose(state.Pose);

        // Inside Telemetry.java -> telemeterize() method:

        if (aprilTagLayout != null) {
            List<Integer> visibleIds = subsystems.vision.getVisibleTagIds();
            Map<Integer, Pose3d> allTagPoses = aprilTagLayout.getTagPoses();
            List<Pose2d> seenTagPoses = new java.util.ArrayList<>();

            // Only collect the tags that are currently visible
            for (Map.Entry<Integer, Pose3d> entry : allTagPoses.entrySet()) {
                if (visibleIds.contains(entry.getKey())) {
                    seenTagPoses.add(entry.getValue().toPose2d());
                }
            }

            // Dynamically split seen tags into groups of 8
            List<List<Pose2d>> seenChunks = new java.util.ArrayList<>();
            int totalSeen = seenTagPoses.size();

            for (int i = 0; i < totalSeen; i += 8) {
                int endIdx = Math.min(i + 8, totalSeen);
                seenChunks.add(seenTagPoses.subList(i, endIdx));
            }

            // Push the chunked seen tags to the field visualizer
            FieldVisualizer.getInstance().updateSeenTagsSplit(seenChunks);
        }

        /* Telemeterize the module states to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(
                    state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            SmartDashboard.putData("Visualizers/Swerve/Module " + i, m_moduleMechanisms[i]);
        }
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in
        // teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    public void addShootingTargetPose(Pose2d targetPose) {
        m_field.getObject("ShootingTarget").setPose(targetPose);
    }

    public void addDrivingTargetPose(Pose2d targetPose) {
        m_field.getObject("DrivingTarget").setPose(targetPose);
    }

    public void publishDetectedObjects(List<Pose2d> objectPoses) {
        m_field.getObject("DetectedObjects").setPoses(objectPoses);
    }
}
