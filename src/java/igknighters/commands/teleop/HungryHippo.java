package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.Radians;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import igknighters.subsystems.swerve.Swerve;

public class HungryHippo {

    public static PIDController thetaController;
    public static SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    public HungryHippo() {
        thetaController = new PIDController(4.0, 0.0, 0.0);
    }
    public static Translation2d getAtractiveForce(double tx, double ty) {
        // 1. If the target is behind the robot (negative Y in robot-relative terms), 
        // or very far away, we apply no force.
        if (ty <= 0) {
            return new Translation2d(0, 0);
        }

        // 2. Calculate Euclidean distance: sqrt(tx^2 + ty^2)
        double distance = Math.hypot(tx, ty);

        // 3. Sensitivity constant (k). 
        // Adjust this: 0.5 means force drops to near-zero at ~10 meters.
        // 1.0 means force drops to near-zero at ~5 meters.
        double k = 0.5; 

        // 4. Calculate magnitude using exponential decay
        // Result is 1.0 when distance is 0, approaching 0.0 as distance grows.
        double magnitude = Math.exp(-k * distance);

        // 5. Normalize the (tx, ty) vector and scale it by our magnitude score.
        // This creates a vector pointing at the piece with a length of 'magnitude'.
        if (distance == 0) return new Translation2d(0, 0);
        
        double forceX = (tx / distance) * magnitude;
        double forceY = (ty / distance) * magnitude;

        return new Translation2d(forceX, forceY);
    }
    public static Translation2d getTotalAtractiveForce(Translation2d[] gamePieces) {
        double totalX = 0.0;
        double totalY = 0.0;
        for (Translation2d piece : gamePieces) {
            Translation2d forces = getAtractiveForce(piece.getX(), piece.getY());
            totalX += forces.getX();
            totalY += forces.getY();
        }
        return new Translation2d(totalX, totalY);
    }

    public static Angle getAngleFromForce(Translation2d force) {
        return Radians.of(Math.atan2(force.getY(), force.getX()));
    }
    // Track velocity across loops
    public static double currentVx = 0.0;
    public static double currentVy = 0.0;

    // Tuning constants
    public static final double ACCEL_SCALER = 0.5; // How "punchy" the acceleration is
    public static final double FRICTION = 0.85;    // 1.0 = ice, 0.0 = instant stop. 0.8-0.9 is usually sweet.

    public static void hippo(Swerve swerve, ChassisSpeeds maxSpeeds, Supplier<Translation2d[]> gamePieces, Supplier<Pose2d> robotPose, boolean allowedToCrossMidline) {
        // 1. Get the net attractive force (our "Acceleration")
        Translation2d accelVector = getTotalAtractiveForce(gamePieces.get());

        // 2. Physics Step: Velocity = (Current Velocity * Friction) + Acceleration
        // We multiply by ACCEL_SCALER to turn our 0-1 force into m/s units
        currentVx = (currentVx * FRICTION) + (accelVector.getX() * ACCEL_SCALER);
        currentVy = (currentVy * FRICTION) + (accelVector.getY() * ACCEL_SCALER);

        // 3. Rotation (using your existing PID logic)
        // We only rotate if there is actually a force pulling us
        double thetaOutput = 0;
        if (accelVector.getNorm() > 0.1) {
            Angle targetAngle = getAngleFromForce(accelVector);
            thetaOutput = thetaController.calculate(
                robotPose.get().getRotation().getRadians(), 
                targetAngle.in(Radians)
            );
        }

        // 4. Create and Clamp Speeds
        // Ensure we don't try to go faster than the drivetrain allows
        double finalVx = Math.max(-maxSpeeds.vxMetersPerSecond, Math.min(maxSpeeds.vxMetersPerSecond, currentVx));
        double finalVy = Math.max(-maxSpeeds.vyMetersPerSecond, Math.min(maxSpeeds.vyMetersPerSecond, currentVy));

        ChassisSpeeds targetSpeeds = new ChassisSpeeds(finalVx, finalVy, thetaOutput);

        // 5. Drive!
        swerve.setControl(applyRobotSpeeds.withSpeeds(targetSpeeds));
    }
}
