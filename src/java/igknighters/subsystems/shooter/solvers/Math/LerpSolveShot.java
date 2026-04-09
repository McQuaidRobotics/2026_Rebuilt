package igknighters.subsystems.shooter.solvers.Math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import igknighters.FieldVisualizer;
import igknighters.Robot;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.util.*;
import igknighters.util.LerpTable.LerpTableEntry;

public class LerpSolveShot {
    // minimal change in RPM most of the change will come from the hood

    // RADIAL_TOWARDS RADIAL_AWAY TANGENTIAL all the inputs are in hyoptenuse of (VX, VY) of robot

    static LerpTable RADIAL_TOWARDS =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1.0, .6),
                        new LerpTableEntry(2.0, .7),
                        new LerpTableEntry(3, .8),
                        new LerpTableEntry(4.5, .9),
                        new LerpTableEntry(5, 1.0)
                    });

    static LerpTable TANGENTIAL =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1.0, .7),
                        new LerpTableEntry(2.0, .85),
                        new LerpTableEntry(3, .95),
                        new LerpTableEntry(4.5, 1.05),
                        new LerpTableEntry(5, 1.09)
                    });

    static LerpTable RADIAL_AWAY =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1.0, .6),
                        new LerpTableEntry(2.0, .7),
                        new LerpTableEntry(3, .8),
                        new LerpTableEntry(4.5, .9),
                        new LerpTableEntry(5, 1.0)
                    });

    static LerpTable HOOD_LERP =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1.5, kHood.MIN_ANGLE_DEGREES),
                        new LerpTableEntry(2.5, 25.0),
                        new LerpTableEntry(3.5, 30.0),
                        new LerpTableEntry(4.5, 33.0),
                        new LerpTableEntry(5.5, 37.0),
                        new LerpTableEntry(6.0, 38.0),
                        new LerpTableEntry(10.0, 45)
                    });

    static LerpTable RPM_LERP =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1.5, 2500),
                        new LerpTableEntry(2.0, 2750),
                        new LerpTableEntry(2.5, 2800),
                        new LerpTableEntry(3.5, 3100),
                        new LerpTableEntry(4.0, 3200),
                        new LerpTableEntry(4.5, 3300),
                        new LerpTableEntry(5.0, 3680),
                        new LerpTableEntry(5.5, 3720),
                        new LerpTableEntry(6.0, 3800),
                        new LerpTableEntry(8.0, 4000),
                        new LerpTableEntry(10.0, 4200),
                        new LerpTableEntry(20, 5500)
                    });

    static LerpTable TIME_OF_FLIGHT_LERP =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1, .9),
                        new LerpTableEntry(2.5, 1.1),
                        new LerpTableEntry(3.5, 1.05),
                        new LerpTableEntry(4, 1.1),
                        new LerpTableEntry(5.5, 1.15),
                        new LerpTableEntry(6, 1)
                    });

    public static ShooterState solve(
            Pose3d goalPose, double currentRPM, double latencyCompensation) {

        Pose3d shooterPose = Robot.turret_pred.getPredictedPose().get();

        ChassisSpeeds robotSpeeds = Robot.pose_pred.getDynamicPredictedSpeeds();
        Translation2d rawRobotVelocity =
                new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        double kConversion = SubsystemConstants.kShooter.kFlywheels.RPM_TO_METERS_PER_SECOND_FACTOR;

        // --- NEW: Radial and Tangential Separation ---
        Translation2d vectorToGoal =
                goalPose.getTranslation()
                        .toTranslation2d()
                        .minus(shooterPose.getTranslation().toTranslation2d());

        double actualDistance = vectorToGoal.getNorm();

        // 1. Find the unit vector pointing straight at the goal
        Translation2d unitVectorToGoal =
                actualDistance > 1e-6 ? vectorToGoal.div(actualDistance) : new Translation2d();

        // 2. Project robot velocity onto the unit vector (Radial magnitude)
        // Positive = moving towards goal, Negative = moving away
        double radialVelocityMag =
                (unitVectorToGoal.getX() * rawRobotVelocity.getX())
                        + (unitVectorToGoal.getY() * rawRobotVelocity.getY());

        // 3. Separate into radial and tangential vectors
        Translation2d radialVelocity = unitVectorToGoal.times(radialVelocityMag);
        Translation2d tangentialVelocity = rawRobotVelocity.minus(radialVelocity);

        // 4. TODO: Tune these! Pull them out into TunableDoubles for Glass/AdvantageScope
        double radialTowardsMultiplier = RADIAL_TOWARDS.lerp(rawRobotVelocity.getNorm());
        double radialAwayMultiplier =
                RADIAL_AWAY.lerp(
                        rawRobotVelocity.getNorm()); // Keep reducing until overshooting away stops
        double tangentialMultiplier =
                TANGENTIAL.lerp(
                        rawRobotVelocity
                                .getNorm()); // Tune this if your shots drift left/right while
        // strafing

        double radialMultiplierToUse =
                (radialVelocityMag >= 0) ? radialTowardsMultiplier : radialAwayMultiplier;

        // 5. Apply multipliers and recombine
        Translation2d tunedRadialVelocity = radialVelocity.times(radialMultiplierToUse);
        Translation2d tunedTangentialVelocity = tangentialVelocity.times(tangentialMultiplier);

        Translation2d tunedRobotVelocity = tunedRadialVelocity.plus(tunedTangentialVelocity);
        // ------------------------------------------

        // --- STEP 1: Initial Estimate ---
        double tof = TIME_OF_FLIGHT_LERP.lerp(actualDistance);

        double requiredTableRpm = 0;
        Rotation2d fieldRelativeTurretAngle = new Rotation2d();

        for (int i = 0; i < 2; i++) {
            // Find where the goal "will be" relative to the ball
            // USE TUNED VELOCITY HERE
            Translation2d movingCompensation = tunedRobotVelocity.times(tof + latencyCompensation);
            Translation2d relativeGoal2d = vectorToGoal;

            Translation2d compensatedVector = relativeGoal2d.minus(movingCompensation);

            FieldVisualizer.getInstance()
                    .updateShootingTarget(
                            new Pose2d(
                                    goalPose.getTranslation()
                                            .toTranslation2d()
                                            .minus(movingCompensation),
                                    new Rotation2d()));
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
                Radians.of(robotRelativeTurretAngle.getRadians()),
                Degrees.of(finalHoodAngle));
    }
}
