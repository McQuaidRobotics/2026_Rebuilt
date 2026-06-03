package igknighters.util.Prediction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.Prediction.InertialPosePredictor;

public class Localizer {
    // 1. Singleton instance of the Localizer itself
    private static Localizer instance = null;

    // 2. Static references to your dependency and your predictor
    private static Swerve swerve;
    private static final InertialPosePredictor inertialPosePredictor = new InertialPosePredictor();

    /**
     * The constructor should be private! 
     * This prevents other parts of the code from accidentally calling `new Localizer(swerve)` 
     * and messing up your static assignments.
     */
    private Localizer(Swerve swerveSubsystem) {
        swerve = swerveSubsystem;
    }

    /**
     * Global access point to initialize or get the static Localizer instance.
     * Call this in RobotContainer when initializing subsystems.
     */
    public static synchronized Localizer getInstance(Swerve swerve) {
        if (instance == null) {
            instance = new Localizer(swerve);
        }
        return instance;
    }

    /**
     * Alternative getter if you just need to access the localizer after it's been initialized.
     */
    public static synchronized Localizer getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Localizer was accessed before being initialized via getInstance(Swerve)!");
        }
        return instance;
    }

    /**
     * Periodic update loop. Call this from Robot.java or a dedicated thread/subsystem periodic.
     */
    public void update() {
        SwerveDriveState swerveState = swerve.getState();
        
        // Convert microseconds to seconds
        double timestampSeconds = swerveState.Timestamp;

        // Fetch pristine hardware data from your Swerve subsystem wrapper
        double imuAccelX = swerve.getXAcceleration();
        double imuAccelY = swerve.getYAcceleration();
        double gyroOmega = swerve.getRotationalVelocity();
        
        Pose2d currentPose = swerveState.Pose; 
        ChassisSpeeds robotSpeeds = swerveState.Speeds;

        // Thread-safe update via our synchronized predictor methods
        inertialPosePredictor.update(
            currentPose,
            robotSpeeds,
            imuAccelX,
            imuAccelY,
            gyroOmega,
            timestampSeconds
        );
    }

    /**
     * Exposes the static physics prediction out to your commands.
     */
    public static Pose2d getPredictedPose(double lookaheadTimeSeconds) {
        return inertialPosePredictor.predict(lookaheadTimeSeconds);
    }
}