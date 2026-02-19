package igknighters.commands.teleop;

import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.swerve.Swerve;
import java.util.function.Supplier;

public class HungryHippo {

    private static final PIDController thetaController = new PIDController(4.0, 0.0, 0.0);
    private static final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds =
            new SwerveRequest.ApplyRobotSpeeds();

    // Tuning constants
    public static final double ACCEL_SCALER = 0.5;
    public static final double FRICTION = 0.85;
    public static final double FIELD_MIDLINE_X = 8.25; // Standard FRC Midline

    // Velocity state
    private static double currentVx = 0.0;
    private static double currentVy = 0.0;

    static {
        // Essential for rotation! Prevents the robot from spinning the "long way"
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static Command hippoCommand(
            Swerve swerve,
            ChassisSpeeds maxSpeeds,
            Supplier<Translation2d[]> gamePieces,
            Supplier<Pose2d> robotPose,
            boolean allowedToCrossMidline) {

        return swerve.run(
                        () ->
                                hippo(
                                        swerve,
                                        maxSpeeds,
                                        gamePieces,
                                        robotPose,
                                        allowedToCrossMidline))
                .beforeStarting(
                        () -> {
                            // Reset velocity when the command starts so it doesn't "jump"
                            currentVx = 0;
                            currentVy = 0;
                        });
    }

    public static void hippo(
            Swerve swerve,
            ChassisSpeeds maxSpeeds,
            Supplier<Translation2d[]> gamePieces,
            Supplier<Pose2d> robotPose,
            boolean allowedToCrossMidline) {

        Translation2d accelVector = getTotalAtractiveForce(gamePieces.get());

        // Physics Step
        currentVx = (currentVx * FRICTION) + (accelVector.getX() * ACCEL_SCALER);
        currentVy = (currentVy * FRICTION) + (accelVector.getY() * ACCEL_SCALER);

        // Rotation Logic
        double thetaOutput = 0;
        if (accelVector.getNorm() > 0.05) {
            // Note: Use Math.atan2(y, x) for the heading of the vector
            double targetRad = Math.atan2(accelVector.getY(), accelVector.getX());
            thetaOutput =
                    thetaController.calculate(
                            robotPose.get().getRotation().getRadians(), targetRad);
        }

        // Midline Enforcement
        // If not allowed to cross and robot is at midline moving toward it, kill the X velocity
        double robotX = robotPose.get().getX();
        if (!allowedToCrossMidline) {
            if ((robotX > FIELD_MIDLINE_X && currentVx > 0)
                    || (robotX < FIELD_MIDLINE_X && currentVx < 0)) {
                // Simplified: if you're on the far side or trying to cross, stop X
                currentVx = 0;
            }
        }

        // Clamp to Max Speeds
        double finalVx =
                Math.max(
                        -maxSpeeds.vxMetersPerSecond,
                        Math.min(maxSpeeds.vxMetersPerSecond, currentVx));
        double finalVy =
                Math.max(
                        -maxSpeeds.vyMetersPerSecond,
                        Math.min(maxSpeeds.vyMetersPerSecond, currentVy));

        // Logging
        DogLog.log("Commands/Hippo/Accel X", accelVector.getX());
        DogLog.log("Commands/Hippo/Accel Y", accelVector.getY());
        DogLog.log("Commands/Hippo/x", robotX);
        DogLog.log("Commands/Hippo/y", robotPose.get().getY());
        DogLog.log("Commands/Hippo/VX", finalVx);
        DogLog.log("Commands/Hippo/VY", finalVy);

        swerve.setControl(
                applyRobotSpeeds.withSpeeds(new ChassisSpeeds(finalVx, finalVy, thetaOutput)));
    }

    public static Translation2d getAtractiveForce(double tx, double ty) {
        if (ty <= 0) return new Translation2d(0, 0); // Ignore behind

        double distance = Math.hypot(tx, ty);
        double k = 0.5;
        double magnitude = Math.exp(-k * distance);

        if (distance < 1e-6) return new Translation2d(0, 0);

        return new Translation2d((tx / distance) * magnitude, (ty / distance) * magnitude);
    }

    public static Translation2d getTotalAtractiveForce(Translation2d[] gamePieces) {
        double totalX = 0, totalY = 0;
        if (gamePieces == null) return new Translation2d(0, 0);
        for (Translation2d piece : gamePieces) {
            Translation2d force = getAtractiveForce(piece.getX(), piece.getY());
            totalX += force.getX();
            totalY += force.getY();
        }
        return new Translation2d(totalX, totalY);
    }
}
