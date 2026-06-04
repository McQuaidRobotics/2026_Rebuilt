package igknighters.util.Prediction;

import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.Merging.PoseMerger;
import igknighters.util.Prediction.VisionFuser.FusedVisionData;
import java.util.Optional;

public class Localizer {
    // The single global instance
    private static Localizer instance = null;

    private final Swerve swerve;
    private final InertialPosePredictor inertialPosePredictor = new InertialPosePredictor();

    /** Private constructor guarantees nobody can call 'new Localizer()' outside this class. */
    private Localizer(Swerve swerve) {
        this.swerve = swerve;
    }

    /** Initializes the Singleton. Call this ONCE in RobotContainer. */
    public static synchronized Localizer initialize(Swerve swerve) {
        if (instance == null) {
            instance = new Localizer(swerve);
        }
        return instance;
    }

    /** Global access point to get the instance anywhere else in the robot code. */
    public static synchronized Localizer getInstance() {
        if (instance == null) {
            throw new IllegalStateException(
                    "Localizer must be initialized with a Swerve reference first!");
        }
        return instance;
    }

    /** Ticks the physics engine history. Call this in robotPeriodic(). */
    public void update() {
        SwerveDriveState swerveState = swerve.getState();

        // Convert microseconds directly to seconds
        double timestampSeconds = swerveState.Timestamp;

        // 1. Fetch raw ROBOT-CENTRIC accelerations from the IMU
        double robotAccelX = swerve.getXAcceleration(); // Forward/backward relative to robot front
        double robotAccelY = swerve.getYAcceleration(); // Left/right relative to robot left

        // 2. Get the robot's current rotation heading
        Rotation2d robotHeading = swerveState.Pose.getRotation();

        // 3. Rotate the robot-centric acceleration vector into a FIELD-RELATIVE vector
        // Formula:
        // fieldX = robotX * cos(theta) - robotY * sin(theta)
        // fieldY = robotX * sin(theta) + robotY * cos(theta)
        double fieldAccelX =
                (robotAccelX * robotHeading.getCos()) - (robotAccelY * robotHeading.getSin());
        double fieldAccelY =
                (robotAccelX * robotHeading.getSin()) + (robotAccelY * robotHeading.getCos());

        double gyroOmega = swerve.getRotationalVelocity();
        Pose2d currentPose = swerveState.Pose;
        ChassisSpeeds robotSpeeds = swerve.getFieldRelativeSpeeds();

        // 4. Pass the true field-relative accelerations into the predictor
        inertialPosePredictor.update(
                currentPose,
                robotSpeeds,
                fieldAccelX, // Fused correctly!
                fieldAccelY, // Fused correctly!
                gyroOmega,
                timestampSeconds);
    }

    public SwerveDriveState getSwerveState() {
        return swerve.getState();
    }

    public void updateVision(VisionSnapshot visionSnapshot) {
        FusedVisionData fusedVisionData =
                VisionFuser.getFusedVision(visionSnapshot, swerve.getState());

        if (fusedVisionData != null) {
            swerve.addVisionMeasurement(
                    fusedVisionData.fusedPose,
                    fusedVisionData.averageTimeStamp,
                    VecBuilder.fill(
                            fusedVisionData.stdDevs[0],
                            fusedVisionData.stdDevs[1],
                            fusedVisionData.stdDevs[2]));
        }
    }

    /**
     * Blends real-world physics tracking with planned Choreo trajectories.
     *
     * @param lookaheadTimeSeconds Future time horizon in SECONDS
     */
    public Pose2d getPredictedPose(double lookaheadTimeSeconds) {
        Pose2d inertialPose = inertialPosePredictor.predict(lookaheadTimeSeconds);
        AutoTrajectory activeTraj = swerve.getActiveTrajectory();

        if (activeTraj == null) {
            return inertialPose;
        }

        double futureTime = swerve.getAutoTime() + lookaheadTimeSeconds;
        Trajectory<SwerveSample> trajectory = activeTraj.getRawTrajectory();
        Optional<SwerveSample> futureSampleOptional = trajectory.sampleAt(futureTime, true);

        Pose2d plannedFuturePose =
                futureSampleOptional.map(SwerveSample::getPose).orElse(inertialPose);

        return PoseMerger.trustedMerge(inertialPose, plannedFuturePose);
    }

    /**
     * Blends real-world physics velocity tracking with planned Choreo trajectory velocities.
     *
     * @param lookaheadTimeSeconds Future time horizon in SECONDS
     */
    public ChassisSpeeds getPredictedVelocity(double lookaheadTimeSeconds) {
        ChassisSpeeds inertialVelocity =
                inertialPosePredictor.predictVelocity(lookaheadTimeSeconds);
        AutoTrajectory activeTraj = swerve.getActiveTrajectory();

        if (activeTraj == null) {
            return inertialVelocity;
        }

        double futureTime = swerve.getAutoTime() + lookaheadTimeSeconds;
        Trajectory<SwerveSample> trajectory = activeTraj.getRawTrajectory();
        Optional<SwerveSample> futureSampleOptional = trajectory.sampleAt(futureTime, true);

        if (futureSampleOptional.isPresent()) {
            SwerveSample sample = futureSampleOptional.get();
            ChassisSpeeds plannedVelocity = new ChassisSpeeds(sample.vx, sample.vy, sample.omega);

            double physicsTrust = 0.70;
            return new ChassisSpeeds(
                    (inertialVelocity.vxMetersPerSecond * physicsTrust)
                            + (plannedVelocity.vxMetersPerSecond * (1.0 - physicsTrust)),
                    (inertialVelocity.vyMetersPerSecond * physicsTrust)
                            + (plannedVelocity.vyMetersPerSecond * (1.0 - physicsTrust)),
                    (inertialVelocity.omegaRadiansPerSecond * physicsTrust)
                            + (plannedVelocity.omegaRadiansPerSecond * (1.0 - physicsTrust)));
        }

        return inertialVelocity;
    }
}
