package igknighters.util.Prediction;

import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        ChassisSpeeds robotSpeeds = swerve.getState().Speeds;

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

    public ChassisSpeeds getInstantaneousTurretFieldSpeeds(Translation2d turretPosition) {
        ChassisSpeeds fieldRelativeCurrentRobotSpeeds = swerve.getFieldRelativeSpeeds();
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
                tangentialVelocityRc.rotateBy(swerve.getState().Pose.getRotation());

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
        Optional<SwerveSample> futureSampleOptional = trajectory.sampleAt(futureTime, true);

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
