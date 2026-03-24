package igknighters.subsystems.shooter.solvers.Math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import igknighters.Robot;
import igknighters.constants.Conv;
import igknighters.constants.ShootInformation;
import igknighters.constants.SubsystemConstants;
import igknighters.constants.SubsystemConstants.kShooter.kHood;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.subsystems.shooter.solvers.Solver;
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
                        new LerpTableEntry(1, 1),
                        new LerpTableEntry(2, 1),
                        new LerpTableEntry(3, 1),
                        new LerpTableEntry(4, 1),
                        new LerpTableEntry(5, 1),
                        new LerpTableEntry(6, 1.0),
                        new LerpTableEntry(10, 1.0)
                    });

    public static ShooterState solve(
            Pose3d shooterPose, Pose3d goalPose, double currentRPM, double latencyCompensation) {
        // 1. Get current velocity (Assume this is FIELD-RELATIVE from pose estimator)
        ChassisSpeeds robotSpeeds = Robot.pose_pred.getPredictedVelos();
        Pose3d predictedShooterPose = Robot.pose_pred.getPredictedShooterPose(shooterPose);
        Translation2d robotVelocity =
                new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

        // 2. Calculate the REAL Floor Distance (2D) for stable RPM
        double floorDistance =
                predictedShooterPose
                        .toPose2d()
                        .getTranslation()
                        .getDistance(goalPose.toPose2d().getTranslation());
        double requiredRpm = RPM_LERP.lerp(floorDistance);

        // 3. Time of Flight based on actual distance
        double tof = TIME_OF_FLIGHT_LERP.lerp(floorDistance);

        // 4. Calculate Movement Compensation
        Translation2d movingCompensation = robotVelocity.times(tof + latencyCompensation);

        Translation2d relativeGoal2d =
                goalPose.getTranslation()
                        .toTranslation2d()
                        .minus(shooterPose.getTranslation().toTranslation2d());

        Translation2d compensatedVector = relativeGoal2d.minus(movingCompensation);

        // 5. EXTRACT RESULTS
        // This is the angle the turret needs to point RELATIVE TO THE FIELD
        Rotation2d fieldRelativeTurretAngle = compensatedVector.getAngle();

        // THIS IS THE FIX: Subtract the robot's heading to get the angle RELATIVE TO THE ROBOT
        // If the field angle is 90° and the robot is at 10°, the turret needs to be at 80°
        Rotation2d robotRelativeTurretAngle =
                fieldRelativeTurretAngle.minus(shooterPose.getRotation().toRotation2d());

        // 6. PRIORITIZE HOOD
        double effectiveDistance = compensatedVector.getNorm();
        double requiredHoodAngle = HOOD_LERP.lerp(effectiveDistance);

        if (requiredRpm < 6000) {

            ShootInformation.getInstance().setPossibleShot(true);

        } else {
            ShootInformation.getInstance().setPossibleShot(false);
        }

        Solver.publishShotTrajectory(
                requiredRpm
                        * SubsystemConstants.kShooter.kFlywheels.RPM_TO_METERS_PER_SECOND_FACTOR,
                (requiredHoodAngle * Conv.DEGREES_TO_RADIANS),
                fieldRelativeTurretAngle.getRadians(),
                shooterPose,
                goalPose);

        return new ShooterState(
                RPM.of(requiredRpm),
                Radians.of(-robotRelativeTurretAngle.getRadians()),
                Degrees.of(requiredHoodAngle));
    }
}
