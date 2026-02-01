package igknighters.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
import java.util.ArrayList;
import java.util.Arrays;

public class repulsor {
    public enum obstacleType {
        CIRCLE,
        SQUARE;
    }

    /**
     * {@summary}Holds the data for an obstacle used in the repulsor field navigation system.
     *
     * @param obstaclePose The Pose2d representing the position of the obstacle rotation is ignored
     *     measured from center of obstacle.
     * @param strength The repulsion strength of the obstacle. 1 is a good starting point.
     * @param width The width of the obstacle (center to edge)(used for visualization or collision
     *     detection).
     * @param height The height of the obstacle (center to edge)(used for visualization or collision
     *     detection).
     */
    public record obstacle(
            Pose2d obstaclePose, double strength, double width, double height, obstacleType type) {}

    static double REPELSCALE = 1.0;
    static double PUSHSCALE = 4.0;
    static double maxTime = 0.0;

    public static double getXComponents(
            Pose2d currentPose, ArrayList<repulsor.obstacle> obstacles, Pose2d target) {

        double currentTime = RobotController.getFPGATime() * 1000.0; // microseconds to milliseconds
        DogLog.log("Commands/repulsor/Time", currentTime);
        double xRepelForce = 0.0;
        for (repulsor.obstacle obs : obstacles) {
            double dist =
                    Math.hypot(
                            obs.obstaclePose.getX() - currentPose.getX(),
                            obs.obstaclePose.getY() - currentPose.getY());
            if (obs.obstaclePose.getX() - currentPose.getX() > 0) {
                xRepelForce += Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist);
            } else {
                xRepelForce -= Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist);
            }
            DogLog.log("Commands/repulsor/xRepel", xRepelForce);
        }
        double xGoalDist = target.getX() - currentPose.getX();
        DogLog.log("Commands/repulsor/xGoalDist", target.getX() - currentPose.getX());
        DogLog.log(
                "Commands/repulsor/FirstObsDistX",
                obstacles.get(0).obstaclePose.getX() - currentPose.getX());
        double xFinalForce = -xGoalDist * PUSHSCALE + xRepelForce * REPELSCALE;
        DogLog.log("Commands/repulsor/xFinalForce", xFinalForce);
        double deltaTime = Timer.getFPGATimestamp() * 1000 - currentTime;
        DogLog.log("Commands/repulsor/DeltaTime", deltaTime);
        if (deltaTime > maxTime) {
            maxTime = deltaTime;
            DogLog.log("Commands/repulsor/MaxDeltaTime", maxTime);
        }
        return xFinalForce;
    }

    public static double getYComponents(
            Pose2d currentPose, ArrayList<repulsor.obstacle> obstacles, Pose2d target) {
        double yRepelForce = 0.0;
        double currentTime = RobotController.getFPGATime() * 1000.0; // microseconds to milliseconds
        DogLog.log("Commands/repulsor/Time", currentTime);
        for (repulsor.obstacle obs : obstacles) {
            double dist =
                    Math.hypot(
                            obs.obstaclePose.getX() - currentPose.getX(),
                            obs.obstaclePose.getY() - currentPose.getY());
            if (obs.obstaclePose.getY() - currentPose.getY() > 0) {
                yRepelForce += Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist);
            } else {
                yRepelForce -= Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist);
            }
            DogLog.log("Commands/repulsor/yRepel", yRepelForce);
        }
        double yGoalDist = target.getY() - currentPose.getY();
        DogLog.log("Commands/repulsor/yGoalDist", target.getY() - currentPose.getY());
        DogLog.log(
                "Commands/repulsor/FirstObsDistY",
                obstacles.get(0).obstaclePose.getY() - currentPose.getY());
        double yFinalForce = -yGoalDist * PUSHSCALE + yRepelForce * REPELSCALE;
        DogLog.log("Commands/repulsor/yFinalForce", yFinalForce);
        double deltaTime = Timer.getFPGATimestamp() * 1000 - currentTime;
        DogLog.log("Commands/repulsor/DeltaTime", deltaTime);
        if (deltaTime > maxTime) {
            maxTime = deltaTime;
            DogLog.log("Commands/repulsor/MaxDeltaTime", maxTime);
        }
        return yFinalForce;
    }

    public static Command moveWithRepulsor(
            CommandSwerveDrivetrain swerve, Pose2d targetPose, double strength) {
        obstacle obs1 =
                new obstacle(
                        new Pose2d(
                                Units.inchesToMeters(182.11),
                                Units.inchesToMeters(98.85),
                                new Rotation2d()),
                        2.0,
                        2.0,
                        2.0,
                        obstacleType.CIRCLE);
        obstacle obs2 =
                new obstacle(
                        new Pose2d(
                                Units.inchesToMeters(182.11),
                                Units.inchesToMeters(218.85),
                                new Rotation2d()),
                        1.0,
                        2.0,
                        2.0,
                        obstacleType.CIRCLE);
        obstacle obs3 =
                new obstacle(
                        new Pose2d(
                                Units.inchesToMeters(182.11),
                                Units.inchesToMeters(158.85),
                                new Rotation2d()),
                        3.0,
                        4.0,
                        2.0,
                        obstacleType.CIRCLE);
        ArrayList<obstacle> obstacles = new ArrayList<>(Arrays.asList(obs1, obs2, obs3));

        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.05)
                        .withRotationalDeadband(
                                RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.05)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
        final PIDController thetaController = new PIDController(0.1, 0.0, 0.0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return swerve.run(
                () -> {
                    Pose2d currentPose = swerve.getState().Pose;
                    double xVelo = 10 * getXComponents(currentPose, obstacles, targetPose);
                    double yVelo = 10 * getYComponents(currentPose, obstacles, targetPose);
                    double omega =
                            thetaController.calculate(
                                    currentPose.getRotation().getRadians(),
                                    targetPose.getRotation().getRadians());

                    omega *= 20.0;
                    DogLog.log("Commands/repulsor/Omega", omega);
                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(xVelo)
                                    .withVelocityY(yVelo)
                                    .withRotationalRate(omega));
                });
    }
}
