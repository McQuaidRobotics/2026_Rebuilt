package igknighters.subsystems.shooter.solvers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import igknighters.Robot;
import igknighters.subsystems.shooter.ShooterState;
import igknighters.util.*;
import igknighters.util.LerpTable.LerpTableEntry;

public class LerpSolveShot {
    // minimal change in RPM most of the change will come from the hood
    static LerpTable RPM_LERP =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1, 4000),
                            new LerpTableEntry(2, 4100),
                            new LerpTableEntry(3, 4200),
                            new LerpTableEntry(4, 4300),
                            new LerpTableEntry(5, 4400),
                            new LerpTableEntry(6, 4500),
                            new LerpTableEntry(10, 4600)
                        });

    static LerpTable inverseRPM_LERP = new LerpTable(new LerpTableEntry[] {
        new LerpTableEntry(4000, 1),
        new LerpTableEntry(4100, 2),
        new LerpTableEntry(4200, 3),
        new LerpTableEntry(4300, 4),
        new LerpTableEntry(4400, 5),
        new LerpTableEntry(4500, 6),
        new LerpTableEntry(4600, 10)
    });

    static LerpTable HOOD_LERP =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1, 20.0),
                            new LerpTableEntry(2, 25.0),
                            new LerpTableEntry(3, 30.0),
                            new LerpTableEntry(4, 35.0),
                            new LerpTableEntry(5, 40.0),
                            new LerpTableEntry(6, 45.0),
                            new LerpTableEntry(10, 50.0)
                        });
    static LerpTable TIME_OF_FLIGHT_LERP =
                new LerpTable(
                        new LerpTableEntry[] {
                            new LerpTableEntry(1, 0.5),
                            new LerpTableEntry(2, 0.6),
                            new LerpTableEntry(3, 0.7),
                            new LerpTableEntry(4, 0.8),
                            new LerpTableEntry(5, 0.9),
                            new LerpTableEntry(6, 1.0),
                            new LerpTableEntry(10, 1.5)
                        });


    

    

   public ShooterState calculate(
    Pose3d robotPose, 
    Pose3d goalPose, 
    double latencyCompensation
) {
    // 1. Get current velocity and convert to a Translation2d (meters per second)
    ChassisSpeeds robotSpeeds = Robot.pose_pred.getPredictedVelos();
    Translation2d robotVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

    // 2. Calculate the REAL Floor Distance (2D) for stable RPM
    double floorDistance = robotPose.toPose2d().getTranslation()
                            .getDistance(goalPose.toPose2d().getTranslation());
    double requiredRpm = RPM_LERP.lerp(floorDistance);

    // 3. Calculate "Time of Flight" based on actual distance
    double tof = TIME_OF_FLIGHT_LERP.lerp(floorDistance);

    // 4. Calculate the Virtual Goal Position (Movement Compensation)
    // We shift the target based on how much the robot moves during TOF + Latency
    Translation2d movingCompensation = robotVelocity.times(tof + latencyCompensation);
    
    // Get the relative vector from robot to goal (2D)
    Translation2d relativeGoal2d = goalPose.getTranslation().toTranslation2d()
                                    .minus(robotPose.getTranslation().toTranslation2d());
    
    // The "Virtual" shot vector (aiming as if the goal shifted to counteract our movement)
    Translation2d compensatedVector = relativeGoal2d.minus(movingCompensation);
    
    // 5. Extract Results
    // Turret heading (Horizontal)
    Rotation2d turretAngle = compensatedVector.getAngle();
    
    // Effective Distance (The 'felt' distance used to prioritize hood adjustment)
    double effectiveDistance = compensatedVector.getNorm();
    double requiredHoodAngle = HOOD_LERP.lerp(effectiveDistance);

    // Return with proper WPILib Units
    return new ShooterState(
        RPM.of(requiredRpm), 
        Radians.of(turretAngle.getRadians()),
        Degrees.of(requiredHoodAngle)
    );
}
}
