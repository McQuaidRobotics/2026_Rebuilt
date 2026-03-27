package igknighters.subsystems.shooter.solvers.Math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import igknighters.Robot;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.util.*;
import igknighters.util.LerpTable.LerpTableEntry;

public class LerpSolveShot {
    // minimal change in RPM most of the change will come from the hood
    static LerpTable HOOD_LERP =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1.5, kHood.MIN_ANGLE_DEGREES),
                        new LerpTableEntry(2.5, 25.0),
                        new LerpTableEntry(3.5, 35.0),
                        new LerpTableEntry(4.5, 40.0),
                        new LerpTableEntry(6.0, 43.0)
                    });

    static LerpTable RPM_LERP =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1.5, 2500),
                        new LerpTableEntry(2.0, 2750),
                        new LerpTableEntry(2.5, 2800),
                        new LerpTableEntry(3.5, 3200),
                        new LerpTableEntry(4.0, 3300),
                        new LerpTableEntry(4.5, 3400),
                        new LerpTableEntry(5.0, 3600),
                        new LerpTableEntry(6.0, 3800)
                    });

    static LerpTable TIME_OF_FLIGHT_LERP =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1, .3),
                        new LerpTableEntry(2, .3),
                        new LerpTableEntry(3, .3),
                        new LerpTableEntry(4, .3),
                        new LerpTableEntry(5, .3),
                        new LerpTableEntry(6, .3),
                        new LerpTableEntry(10, .3)
                    });

    public static ShooterState solve(
            Pose3d robotPose, Pose3d goalPose, double currentRPM, double latencyCompensation) {

        Pose3d shooterPose = Robot.pose_pred.getPredictedShooterPose(robotPose);

        ChassisSpeeds robotSpeeds = Robot.pose_pred.getPredictedVelos();
        Translation2d rawRobotVelocity =
                new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        double kConversion = SubsystemConstants.kShooter.kFlywheels.RPM_TO_METERS_PER_SECOND_FACTOR;

        // --- NEW: Towards/Away Multiplier Logic ---
        Translation2d vectorToGoal =
                goalPose.getTranslation()
                        .toTranslation2d()
                        .minus(shooterPose.getTranslation().toTranslation2d());

        // Dot product: Positive = moving towards, Negative = moving away
        double dotProduct =
                (vectorToGoal.getX() * rawRobotVelocity.getX())
                        + (vectorToGoal.getY() * rawRobotVelocity.getY());

        // TODO: Tune these! You might want to pull them out into TunableDoubles for
        // Glass/AdvantageScope
        double towardsMultiplier = 1.0;
        double awayMultiplier = 0.75; // Reduce this until the overshooting stops

        double velocityMultiplier = (dotProduct >= 0) ? towardsMultiplier : awayMultiplier;
        Translation2d tunedRobotVelocity = rawRobotVelocity.times(velocityMultiplier);
        // ------------------------------------------

        // --- STEP 1: Initial Estimate ---
        double actualDistance = vectorToGoal.getNorm(); // Reused the vector we made above
        double tof = TIME_OF_FLIGHT_LERP.lerp(actualDistance);

        double requiredTableRpm = 0;
        Rotation2d fieldRelativeTurretAngle = new Rotation2d();

        for (int i = 0; i < 2; i++) {
            // Find where the goal "will be" relative to the ball
            // USE TUNED VELOCITY HERE
            Translation2d movingCompensation = tunedRobotVelocity.times(tof + latencyCompensation);
            Translation2d relativeGoal2d = vectorToGoal;

            Translation2d compensatedVector = relativeGoal2d.minus(movingCompensation);
            double virtualDistance = compensatedVector.getNorm();

            // Get the RPM we WOULD use if we were standing still at this virtual spot
            double baselineRpm = RPM_LERP.lerp(virtualDistance);
            double baselineExitVelocity = baselineRpm * kConversion;

            // Vector Subtraction: (Goal Velocity) - (Robot Velocity) = (Needed Shooter Velocity)
            Translation2d targetDirection = compensatedVector.div(virtualDistance);
            Translation2d fieldRelativeVelocityVector = targetDirection.times(baselineExitVelocity);

            // USE TUNED VELOCITY HERE
            Translation2d requiredShooterVector =
                    fieldRelativeVelocityVector.minus(tunedRobotVelocity);

            // Update our values
            double requiredExitVelocity = requiredShooterVector.getNorm();
            requiredTableRpm = requiredExitVelocity / kConversion;
            fieldRelativeTurretAngle = requiredShooterVector.getAngle();

            // RE-CALCULATE TOF
            double effectiveDistance = RPM_LERP.inverseLerp(requiredTableRpm);
            tof = TIME_OF_FLIGHT_LERP.lerp(effectiveDistance);
        }

        // --- STEP 3: Final Outputs ---
        double finalEffectiveDistance = RPM_LERP.inverseLerp(requiredTableRpm);
        double finalHoodAngle = HOOD_LERP.lerp(finalEffectiveDistance);
        Rotation2d robotRelativeTurretAngle =
                fieldRelativeTurretAngle.minus(shooterPose.getRotation().toRotation2d());

        ShootInformation.getInstance().setPossibleShot(requiredTableRpm < 6000);

        return new ShooterState(
                RPM.of(requiredTableRpm),
                Radians.of(-robotRelativeTurretAngle.getRadians()),
                Degrees.of(finalHoodAngle));
    }
}
