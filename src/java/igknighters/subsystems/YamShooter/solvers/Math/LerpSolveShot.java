package igknighters.subsystems.YamShooter.solvers.Math;

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
import igknighters.subsystems.YamShooter.ShooterState;
import igknighters.util.*;
import igknighters.util.log.Log;

public class LerpSolveShot {
    // minimal change in RPM most of the change will come from the hood

    // RADIAL_TOWARDS RADIAL_AWAY TANGENTIAL all the inputs are in hyoptenuse of (VX, VY) of robot

    static LerpTable RADIAL_TOWARDS = Robot.consts.lerp().kRadialSOTM().towards();

    static LerpTable TANGENTIAL = Robot.consts.lerp().kTangentialSOTM().table();

    static LerpTable RADIAL_AWAY = Robot.consts.lerp().kRadialSOTM().away();

    static LerpTable HOOD_LERP = Robot.consts.lerp().kHoodAngle().table();

    static LerpTable RPM_LERP = Robot.consts.lerp().kRPM().table();

    static LerpTable TIME_OF_FLIGHT_LERP = Robot.consts.lerp().kTimeOfFlight().table();

    public static ShooterState solve(
            Pose3d goalPose, double currentRPM, double latencyCompensation) {

        Pose3d shooterPose = Robot.turret_pred.getPredictedPose().get();

        ChassisSpeeds robotSpeeds = Robot.pose_pred.getDynamicPredictedSpeeds();
        Translation2d rawRobotVelocity =
                new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        double kConversion = Robot.consts.shooter().kFlywheels().RPM_TO_METERS_PER_SECOND_FACTOR();

        Translation2d vectorToGoal =
                goalPose.getTranslation()
                        .toTranslation2d()
                        .minus(shooterPose.getTranslation().toTranslation2d());

        double actualDistance = vectorToGoal.getNorm();
        Log.log("ROBOT/COMMANDS/LERPSOLVE/TURRETDISTANCE", actualDistance);
        Log.log("ROBOT/COMMANDS/LERPSOLVE/TURRETDISTANCE", actualDistance);

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

        // --- UPDATED: Radial and Tangential Speed Inputs ---
        Log.log("ROBOT/COMMANDS/LERPSOLVE/RADIAL VELO", radialVelocity.getNorm());
        Log.log("ROBOT/COMMANDS/LERPSOLVE/TANGENTIAL VELO", tangentialVelocity.getNorm());

        // Input is the absolute radial speed
        double radialTowardsMultiplier = RADIAL_TOWARDS.lerp(Math.abs(radialVelocityMag));
        double radialAwayMultiplier = RADIAL_AWAY.lerp(Math.abs(radialVelocityMag));

        // Input is the magnitude of the tangential component
        double tangentialMultiplier = TANGENTIAL.lerp(tangentialVelocity.getNorm());
        // --- UPDATED: Radial and Tangential Speed Inputs ---
        Log.log("ROBOT/COMMANDS/LERPSOLVE/RADIAL VELO", radialVelocity.getNorm());
        Log.log("ROBOT/COMMANDS/LERPSOLVE/TANGENTIAL VELO", tangentialVelocity.getNorm());

        double radialMultiplierToUse =
                (radialVelocityMag >= 0) ? radialTowardsMultiplier : radialAwayMultiplier;

        // 5. Apply multipliers and recombine
        Translation2d tunedRadialVelocity = radialVelocity.times(radialMultiplierToUse);
        Translation2d tunedTangentialVelocity = tangentialVelocity.times(tangentialMultiplier);

        Translation2d tunedRobotVelocity = tunedRadialVelocity.plus(tunedTangentialVelocity);
        // --------------------------------------------------
        // --------------------------------------------------

        // --- STEP 1: Initial Estimate ---
        double tof = TIME_OF_FLIGHT_LERP.lerp(actualDistance);

        double requiredTableRpm = 0;
        Rotation2d fieldRelativeTurretAngle = new Rotation2d();

        for (int i = 0; i < 2; i++) {
            // Find where the goal "will be" relative to the ball
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

            double baselineRpm = RPM_LERP.lerp(virtualDistance);
            double baselineExitVelocity = baselineRpm * kConversion;

            Translation2d targetDirection = compensatedVector.div(virtualDistance);
            Translation2d fieldRelativeVelocityVector = targetDirection.times(baselineExitVelocity);

            // Vector Subtraction: V_shot = V_target - V_robot
            // Vector Subtraction: V_shot = V_target - V_robot
            Translation2d requiredShooterVector =
                    fieldRelativeVelocityVector.minus(tunedRobotVelocity);

            double requiredExitVelocity = requiredShooterVector.getNorm();
            requiredTableRpm = requiredExitVelocity / kConversion;
            fieldRelativeTurretAngle = requiredShooterVector.getAngle();

            // RE-CALCULATE TOF based on the RPM we are actually shooting at
            // RE-CALCULATE TOF based on the RPM we are actually shooting at
            double effectiveDistance = RPM_LERP.inverseLerp(requiredTableRpm);
            tof = TIME_OF_FLIGHT_LERP.lerp(effectiveDistance);
        }

        // --- STEP 3: Final Outputs ---
        double finalEffectiveDistance = RPM_LERP.inverseLerp(requiredTableRpm);
        double finalHoodAngle = HOOD_LERP.lerp(finalEffectiveDistance);

        // Assuming turret zero is field-relative or robot-relative based on your pose provider

        // Assuming turret zero is field-relative or robot-relative based on your pose provider
        Rotation2d robotRelativeTurretAngle =
                fieldRelativeTurretAngle.minus(shooterPose.getRotation().toRotation2d());

        ShootInformation.getInstance().setPossibleShot(requiredTableRpm < 6000);

        return new ShooterState(
                RPM.of(requiredTableRpm),
                Radians.of(robotRelativeTurretAngle.getRadians()),
                Degrees.of(finalHoodAngle));
    }
}
