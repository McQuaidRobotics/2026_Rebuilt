package igknighters.util.Prediction;

import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.Merging.PoseMerger;
import java.util.Optional;

public class Localizer {
    // The single global instance
    private static Localizer instance = null;

    private final Swerve swerve;
    private final InertialPosePredictor inertialPosePredictor = new InertialPosePredictor();

    // Per-loop caches, cleared in update(). Callers all use the same lookahead.
    private SwerveDriveState lastState;
    private Pose2d cachedPose;
    private double cachedPoseLookahead;
    private ChassisSpeeds cachedVelocity;
    private double cachedVelocityLookahead;

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
        // Swerve state is a JNI round trip; read it once per loop and reuse it everywhere.
        lastState = swerve.getState();
        cachedPose = null;
        cachedVelocity = null;

        // The predictor works in the ROBOT frame (Coriolis correction, encoder accel and
        // Pose2d.exp all assume it), so the IMU accel goes in unrotated.
        inertialPosePredictor.update(
                lastState.Pose,
                lastState.Speeds,
                swerve.getXAcceleration(),
                swerve.getYAcceleration(),
                swerve.getRotationalVelocity(),
                lastState.Timestamp);
    }

    /** The swerve state captured by the last {@link #update()}. */
    public SwerveDriveState getSwerveState() {
        return lastState != null ? lastState : swerve.getState();
    }

    public void updateVision(VisionSnapshot visionSnapshot) {
        // One measurement per camera (same as the YALL branch); the Kalman filter weighs them.
        Pose2d[] poses = visionSnapshot.getPose();
        Double[] timestamps = visionSnapshot.getTimestamp();
        if (poses == null || timestamps == null) return;

        for (int i = 0; i < Math.min(poses.length, timestamps.length); i++) {
            swerve.addVisionMeasurement(poses[i], timestamps[i]);
        }
    }

    /**
     * Blends real-world physics tracking with planned Choreo trajectories.
     *
     * @param lookaheadTimeSeconds Future time horizon in SECONDS
     */
    public Pose2d getPredictedPose(double lookaheadTimeSeconds) {
        if (cachedPose == null || cachedPoseLookahead != lookaheadTimeSeconds) {
            cachedPose = computePredictedPose(lookaheadTimeSeconds);
            cachedPoseLookahead = lookaheadTimeSeconds;
        }
        return cachedPose;
    }

    private Pose2d computePredictedPose(double lookaheadTimeSeconds) {
        Pose2d inertialPose = inertialPosePredictor.predict(lookaheadTimeSeconds);
        AutoTrajectory activeTraj = swerve.getActiveTrajectory();

        if (activeTraj == null) {
            return inertialPose;
        }

        double futureTime = swerve.getAutoTime() + lookaheadTimeSeconds;
        Trajectory<SwerveSample> trajectory = activeTraj.getRawTrajectory();
        Optional<SwerveSample> futureSampleOptional =
                trajectory.sampleAt(futureTime, isRedAlliance());

        Pose2d plannedFuturePose =
                futureSampleOptional.map(SwerveSample::getPose).orElse(inertialPose);

        return PoseMerger.trustedMerge(inertialPose, plannedFuturePose);
    }

    /** The AutoFactory flips paths for red, so samples must be mirrored to match. */
    private static boolean isRedAlliance() {
        return DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
    }

    public ChassisSpeeds getInstantaneousTurretFieldSpeeds(Translation2d turretPosition) {
        SwerveDriveState state = swerve.getState();
        ChassisSpeeds fieldRelativeCurrentRobotSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
        double omega = fieldRelativeCurrentRobotSpeeds.omegaRadiansPerSecond;

        // Field-centric linear velocity contributions from chassis translation
        double baseVxFc = fieldRelativeCurrentRobotSpeeds.vxMetersPerSecond;
        double baseVyFc = fieldRelativeCurrentRobotSpeeds.vyMetersPerSecond;

        // Robot-centric linear velocity contributions from chassis rotation (w x r)
        double tangentialVxRc = -omega * turretPosition.getY();
        double tangentialVyRc = omega * turretPosition.getX();

        // Rotate the robot-centric tangential velocity into the field-centric frame
        Translation2d tangentialVelocityRc = new Translation2d(tangentialVxRc, tangentialVyRc);
        Translation2d tangentialVelocityFc =
                tangentialVelocityRc.rotateBy(state.Pose.getRotation());

        // Fuse field-centric translation and field-centric rotation velocities together
        double apparatusSpeedX = baseVxFc + tangentialVelocityFc.getX();
        double apparatusSpeedY = baseVyFc + tangentialVelocityFc.getY();

        // Return the final field-centric ChassisSpeeds
        return new ChassisSpeeds(apparatusSpeedX, apparatusSpeedY, omega);
    }

    /**
     * Blends real-world physics velocity tracking with planned Choreo trajectory velocities.
     * Returns FIELD-RELATIVE speeds for the rest of the robot to use.
     *
     * @param lookaheadTimeSeconds Future time horizon in SECONDS
     */
    public ChassisSpeeds getPredictedVelocity(double lookaheadTimeSeconds) {
        if (cachedVelocity == null || cachedVelocityLookahead != lookaheadTimeSeconds) {
            cachedVelocity = computePredictedVelocity(lookaheadTimeSeconds);
            cachedVelocityLookahead = lookaheadTimeSeconds;
        }
        return cachedVelocity;
    }

    private ChassisSpeeds computePredictedVelocity(double lookaheadTimeSeconds) {
        // 1. Get the ROBOT-RELATIVE predicted velocity from our physics engine
        ChassisSpeeds robotRelativeInertialVelocity =
                inertialPosePredictor.predictVelocity(lookaheadTimeSeconds);

        // 2. Get the PREDICTED HEADING to accurately rotate these speeds into the field frame
        Rotation2d predictedHeading = getPredictedPose(lookaheadTimeSeconds).getRotation();

        // 3. Convert to FIELD-RELATIVE speeds using WPILib's built-in kinematics
        ChassisSpeeds fieldRelativeInertialVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        robotRelativeInertialVelocity.vxMetersPerSecond,
                        robotRelativeInertialVelocity.vyMetersPerSecond,
                        robotRelativeInertialVelocity.omegaRadiansPerSecond,
                        predictedHeading);

        AutoTrajectory activeTraj = swerve.getActiveTrajectory();

        if (activeTraj == null) {
            return fieldRelativeInertialVelocity;
        }

        double futureTime = swerve.getAutoTime() + lookaheadTimeSeconds;
        Trajectory<SwerveSample> trajectory = activeTraj.getRawTrajectory();
        Optional<SwerveSample> futureSampleOptional =
                trajectory.sampleAt(futureTime, isRedAlliance());

        if (futureSampleOptional.isPresent()) {
            SwerveSample sample = futureSampleOptional.get();
            // Choreo trajectory samples natively provide FIELD-RELATIVE velocities
            ChassisSpeeds plannedVelocity = new ChassisSpeeds(sample.vx, sample.vy, sample.omega);

            double physicsTrust = 0.70;

            // 4. Safely blend our Field-Relative physics with the Field-Relative Choreo plan
            return new ChassisSpeeds(
                    (fieldRelativeInertialVelocity.vxMetersPerSecond * physicsTrust)
                            + (plannedVelocity.vxMetersPerSecond * (1.0 - physicsTrust)),
                    (fieldRelativeInertialVelocity.vyMetersPerSecond * physicsTrust)
                            + (plannedVelocity.vyMetersPerSecond * (1.0 - physicsTrust)),
                    (fieldRelativeInertialVelocity.omegaRadiansPerSecond * physicsTrust)
                            + (plannedVelocity.omegaRadiansPerSecond * (1.0 - physicsTrust)));
        }

        return fieldRelativeInertialVelocity;
    }
}
