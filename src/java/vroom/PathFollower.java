package vroom;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
import java.util.List;

public class PathFollower {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public PathFollower(double kP, double kI, double kD) {
        // Separate PIDs for X and Y to handle 2D translation
        this.xController = new PIDController(kP, kI, kD);
        this.yController = new PIDController(kP, kI, kD);

        // Heading PID (Rotation)
        this.thetaController = new PIDController(1.0, 0, 0);
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Calculates ChassisSpeeds to reach the setpoint at a specific time. * @param currentPose The
     * actual position of the robot from Odometry.
     *
     * @param path The pre-generated list of Poses.
     * @param timeSeconds The current match time or timer value.
     * @param planner The PathPlanner instance used to find the setpoint.
     * @return ChassisSpeeds in meters per second and radians per second.
     */
    public ChassisSpeeds calculateSpeeds(
            Pose2d currentPose, List<Pose2d> path, double timeSeconds, PathPlanner planner) {
        // 1. Determine where we SHOULD be right now
        Pose2d setpoint = planner.getPoseAtTime(path, timeSeconds);

        // 2. Use PID to calculate required velocity in m/s to close the gap
        double xVelocity = xController.calculate(currentPose.getX(), setpoint.getX());
        double yVelocity = yController.calculate(currentPose.getY(), setpoint.getY());

        // 3. Calculate rotation speed (rad/s) to match the path heading
        double omega =
                thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        setpoint.getRotation().getRadians());

        // 4. Return as ChassisSpeeds (Field Relative)
        // If your drive code expects robot-relative, use ChassisSpeeds.fromFieldRelativeSpeeds
        return new ChassisSpeeds(xVelocity, yVelocity, omega);
    }

    public Command createFollowPathCommand(Swerve swerve, List<Pose2d> path, PathPlanner planner) {
        Timer timer = new Timer();
        return swerve.startRun(
                () -> timer.start(),
                () -> {
                    ChassisSpeeds speeds =
                            calculateSpeeds(swerve.getState().Pose, path, timer.get(), planner);
                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(MetersPerSecond.of(speeds.vxMetersPerSecond))
                                    .withVelocityY(MetersPerSecond.of(speeds.vyMetersPerSecond))
                                    .withRotationalRate(
                                            RadiansPerSecond.of(speeds.omegaRadiansPerSecond)));
                });
    }
}
