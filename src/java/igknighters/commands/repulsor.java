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

public class Repulsor {
    public enum obstacleType {
        CIRCLE,
        SQUARE;
    }

    static boolean beenPublished = false;

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

    static RepulsorVisualizer visualizer = new RepulsorVisualizer();

    public static double getXRepulse(Pose2d currentPose, ArrayList<Repulsor.obstacle> obstacles) {
        double currentTime = RobotController.getFPGATime() * 1000.0; // microseconds to milliseconds
        DogLog.log("Commands/repulsor/Time", currentTime);
        double xRepelForce = 0.0;
        for (Repulsor.obstacle obs : obstacles) {
            if (obs.type == obstacleType.CIRCLE) {
                double dist =
                        Math.hypot(
                                obs.obstaclePose.getX() - currentPose.getX(),
                                obs.obstaclePose.getY() - currentPose.getY());
                if (obs.obstaclePose.getX() - currentPose.getX() > 0) {
                    xRepelForce += Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist);
                } else {
                    xRepelForce -= Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist);
                }
            } else if (obs.type == obstacleType.SQUARE
                    && currentPose.getY() >= obs.obstaclePose.getY() - obs.height
                    && currentPose.getY() <= obs.obstaclePose.getY() + obs.height) {
                double dist = obs.obstaclePose.getX() - currentPose.getX();
                if (obs.obstaclePose.getX() - currentPose.getX() > 0) {
                    xRepelForce += Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist);
                } else {
                    xRepelForce -= Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 + dist);
                }
            }
        }
        DogLog.log("Commands/repulsor/xRepel", xRepelForce);
        DogLog.log(
                "Commands/repulsor/FirstObsDistX",
                obstacles.get(0).obstaclePose.getX() - currentPose.getX());
        double deltaTime = Timer.getFPGATimestamp() * 1000 - currentTime;
        DogLog.log("Commands/repulsor/DeltaTime", deltaTime);
        if (deltaTime > maxTime) {
            maxTime = deltaTime;
            DogLog.log("Commands/repulsor/MaxDeltaTime", maxTime);
        }
        return xRepelForce;
    }

    public static double getXGoal(Pose2d currentPose, Pose2d target) {
        double xGoalDist = target.getX() - currentPose.getX();
        DogLog.log("Commands/repulsor/xGoalDist", target.getX() - currentPose.getX());
        return xGoalDist;
    }

    public static double getYRepulse(Pose2d currentPose, ArrayList<Repulsor.obstacle> obstacles) {
        double yRepelForce = 0.0;
        double currentTime = RobotController.getFPGATime() * 1000.0; // microseconds to milliseconds
        DogLog.log("Commands/repulsor/Time", currentTime);
        for (Repulsor.obstacle obs : obstacles) {
            if (obs.type == obstacleType.CIRCLE) {
                double dist =
                        Math.hypot(
                                obs.obstaclePose.getX() - currentPose.getX(),
                                obs.obstaclePose.getY() - currentPose.getY());
                if (obs.obstaclePose.getY() - currentPose.getY() > 0) {
                    yRepelForce += Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist);
                } else {
                    yRepelForce -= Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 + dist);
                }
                DogLog.log("Commands/repulsor/yRepel", yRepelForce);
            } else if (obs.type == obstacleType.SQUARE
                    && currentPose.getX() >= obs.obstaclePose.getX() - obs.width
                    && currentPose.getX() <= obs.obstaclePose.getX() + obs.width) {
                double dist = obs.obstaclePose.getY() - currentPose.getY();
                if (obs.obstaclePose.getY() - currentPose.getY() > 0) {
                    yRepelForce += Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist);
                } else {
                    yRepelForce -= Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 + dist);
                }
            }
        }
        DogLog.log(
                "Commands/repulsor/FirstObsDistY",
                obstacles.get(0).obstaclePose.getY() - currentPose.getY());
        double deltaTime = Timer.getFPGATimestamp() * 1000 - currentTime;
        DogLog.log("Commands/repulsor/DeltaTime", deltaTime);
        if (deltaTime > maxTime) {
            maxTime = deltaTime;
            DogLog.log("Commands/repulsor/MaxDeltaTime", maxTime);
        }
        return yRepelForce;
    }

    public static double getYGoal(Pose2d currentPose, Pose2d target) {
        double yGoalDist = target.getY() - currentPose.getY();
        DogLog.log("Commands/repulsor/yGoalDist", target.getY() - currentPose.getY());
        return yGoalDist;
    }

    public static Command moveWithRepulsor(
            CommandSwerveDrivetrain swerve, Pose2d targetPose, double strength) {
        obstacle obs1 =
                new obstacle(
                        new Pose2d(
                                Units.inchesToMeters(182.11),
                                Units.inchesToMeters(98.85),
                                new Rotation2d()),
                        1.0,
                        Units.inchesToMeters(22.2),
                        Units.inchesToMeters(73 / 2),
                        obstacleType.SQUARE);
        obstacle obs2 =
                new obstacle(
                        new Pose2d(
                                Units.inchesToMeters(182.11),
                                Units.inchesToMeters(218.85),
                                new Rotation2d()),
                        1.0,
                        Units.inchesToMeters(22.2),
                        Units.inchesToMeters(73 / 2),
                        obstacleType.SQUARE);
        obstacle obs3 =
                new obstacle(
                        new Pose2d(
                                Units.inchesToMeters(182.11),
                                Units.inchesToMeters(158.85),
                                new Rotation2d()),
                        2,
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
                    double xVelo =
                            10
                                    * (getXRepulse(currentPose, obstacles)
                                            - getXGoal(currentPose, targetPose));
                    double yVelo =
                            10
                                    * (getYRepulse(currentPose, obstacles)
                                            - getYGoal(currentPose, targetPose));
                    double omega =
                            thetaController.calculate(
                                    currentPose.getRotation().getRadians(),
                                    targetPose.getRotation().getRadians());
                    RepulsorVisualizer.update(
                            Math.atan2(
                                    getYGoal(currentPose, targetPose),
                                    getXGoal(currentPose, targetPose)),
                            Math.atan2(
                                    getYRepulse(currentPose, obstacles),
                                    getXRepulse(currentPose, obstacles)),
                            Math.hypot(
                                    getYGoal(currentPose, targetPose),
                                    getXGoal(currentPose, targetPose)),
                            Math.hypot(
                                    getYRepulse(currentPose, obstacles),
                                    getXRepulse(currentPose, obstacles)));

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
