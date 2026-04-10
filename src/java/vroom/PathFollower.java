package vroom;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;
import vroom.PathPlanner.PathPoint;

public class PathFollower {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public PathFollower(double kP, double kI, double kD) {
        // Separate PIDs for X and Y to handle 2D translation
        this.xController = new PIDController(kP, kI, kD);
        this.yController = new PIDController(kP, kI, kD);

        // Heading PID (Rotation)
        this.thetaController = new PIDController(1.0, 0, 0);
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    //     /**
    //      * Calculates ChassisSpeeds to reach the setpoint at a specific time. * @param
    // currentPose The
    //      * actual position of the robot from Odometry.
    //      *
    //      * @param path The pre-generated list of Poses.
    //      * @param timeSeconds The current match time or timer value.
    //      * @param planner The PathPlanner instance used to find the setpoint.
    //      * @return ChassisSpeeds in meters per second and radians per second.
    //      */
    //     public ChassisSpeeds calculateSpeeds(Pose2d currentPose, Pose2d[] path, PathPlanner
    // planner) {
    //         if (path.length == 0) return new ChassisSpeeds();

    //         // Use a physical distance lookahead (e.g., 0.4 meters)
    //         // instead of a time-based one.
    //         double lookaheadMeters = 1.2;
    //         Pose2d setpoint = planner.getLookaheadPose(currentPose, path, lookaheadMeters);

    //         // Standard PID calculation
    //         double xVelocity = xController.calculate(currentPose.getX(), setpoint.getX());
    //         double yVelocity = yController.calculate(currentPose.getY(), setpoint.getY());

    //         double omega =
    //                 thetaController.calculate(
    //                         currentPose.getRotation().getRadians(),
    //                         setpoint.getRotation().getRadians());

    //         return new ChassisSpeeds(-xVelocity, -yVelocity, omega);
    //     }

    public Command createFollowPathCommand(Swerve swerve, PathPoint[] path, PathPlanner planner) {
        Timer timer = new Timer();
        return swerve.startRun(
                        () -> timer.start(),
                        () -> {
                            ChassisSpeeds speeds =
                                    planner.calculateSpeeds(
                                            swerve.getState().Pose, path, timer.get());
                            swerve.setControl(
                                    m_driveRequest
                                            .withVelocityX(
                                                    MetersPerSecond.of(speeds.vxMetersPerSecond))
                                            .withVelocityY(
                                                    MetersPerSecond.of(speeds.vyMetersPerSecond))
                                            .withRotationalRate(
                                                    RadiansPerSecond.of(
                                                            speeds.omegaRadiansPerSecond)));
                        })
                .finallyDo(() -> timer.reset());
    }
}
