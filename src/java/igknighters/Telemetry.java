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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.subsystems.Subsystems;
import igknighters.util.AprilTagLayout;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class Telemetry {
    private final double MaxSpeed;
    private final Subsystems subsystems;
    // FL/FR/BL/BR angle (rad), FL/FR/BL/BR speed (m/s), heading (rad). Swapped as a whole array
    // by the odometry thread so the main thread's dashboard reads never see a half-written tick.
    private volatile double[] swerveWidget = new double[9];
    private AprilTagLayout aprilTagLayout;
    // Tag poses never change; flattened once instead of on every odometry tick.
    private int[] tagIds = new int[0];
    private Pose2d[] tagPoses = new Pose2d[0];

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
            Map<Integer, Pose3d> layoutPoses = aprilTagLayout.getTagPoses();
            tagIds = new int[layoutPoses.size()];
            tagPoses = new Pose2d[layoutPoses.size()];
            int t = 0;
            for (Map.Entry<Integer, Pose3d> entry : layoutPoses.entrySet()) {
                tagIds[t] = entry.getKey();
                tagPoses[t] = entry.getValue().toPose2d();
                t++;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        SignalLogger.start();

        SmartDashboard.putData(
                "Swerve Drive",
                new Sendable() {
                    @Override
                    public void initSendable(SendableBuilder builder) {
                        builder.setSmartDashboardType("SwerveDrive");
                        String[] names = {"Front Left", "Front Right", "Back Left", "Back Right"};
                        for (int i = 0; i < 4; i++) {
                            final int idx = i;
                            builder.addDoubleProperty(
                                    names[i] + " Angle", () -> swerveWidget[idx], null);
                            builder.addDoubleProperty(
                                    names[i] + " Velocity", () -> swerveWidget[idx + 4], null);
                        }
                        builder.addDoubleProperty("Robot Angle", () -> swerveWidget[8], null);
                    }
                });
        // Registered once; SmartDashboard.updateValues() keeps them current. Re-registering every
        // odometry tick contended with the main loop on SmartDashboard's lock.
        for (int i = 0; i < m_moduleMechanisms.length; ++i) {
            SmartDashboard.putData("Visualizers/Swerve/Module " + i, m_moduleMechanisms[i]);
        }
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
        double[] widget = new double[9];
        for (int i = 0; i < 4; i++) {
            widget[i] = state.ModuleStates[i].angle.getRadians();
            widget[i + 4] = state.ModuleStates[i].speedMetersPerSecond;
        }
        widget[8] = state.Pose.getRotation().getRadians();
        swerveWidget = widget;
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

        if (aprilTagLayout != null) {
            List<Integer> visibleIds = subsystems.vision.getVisibleTagIds();
            List<Pose2d> seenTagPoses = new ArrayList<>();
            for (int t = 0; t < tagIds.length; t++) {
                if (visibleIds.contains(tagIds[t])) {
                    seenTagPoses.add(tagPoses[t]);
                }
            }
            FieldVisualizer.getInstance().updateSeenTags(seenTagPoses);
        }

        /* Telemeterize the module states to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(
                    state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
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
}
