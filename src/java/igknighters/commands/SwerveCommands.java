package igknighters.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.FieldVisualizer;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.swerveconstants.GeminiConsts;
import igknighters.util.log.Log;
import java.util.function.BooleanSupplier;

/**
 * Factory class for creating swerve-related commands. This class provides static methods to create
 * commands for common swerve tasks like zeroing the gyro, stopping the robot, and moving to
 * specific poses using PID.
 */
public class SwerveCommands {

    /**
     * Creates a command to zero the robot's gyro (heading). Automatically adjusts for the current
     * alliance color.
     *
     * @param swerve The swerve subsystem.
     * @return A command to zero the gyro.
     */
    public static Command zeroGyro(Swerve swerve) {
        return Commands.either(
                Commands.runOnce(
                        () ->
                                swerve.resetPose(
                                        new Pose2d(
                                                swerve.getState().Pose.getX(),
                                                swerve.getState().Pose.getY(),
                                                new Rotation2d(0.0)))),
                Commands.runOnce(
                        () ->
                                swerve.resetPose(
                                        new Pose2d(
                                                swerve.getState().Pose.getX(),
                                                swerve.getState().Pose.getY(),
                                                new Rotation2d(Math.PI)))),
                () -> Robot.isBlue());
    }

    /**
     * Returns the current field-relative pose of the robot.
     *
     * @param swerve The swerve subsystem.
     * @return The current {@link Pose2d}.
     */
    public static Pose2d getPose(Swerve swerve) {
        return swerve.getState().Pose;
    }

    /**
     * Checks if the swerve is at the target velocity and not at the start pose. This was made so
     * that we can see if velocity is 0 but not when we start. Because at the start of climb
     * sequence velocity is 0.
     *
     * @param swerve The swerve subsystem
     * @param targetSpeeds The target chassis speeds
     * @param tolerance The tolerance for each chassis speed component
     * @param startPose The starting pose to compare against
     * @param positionToleranceMeters The position away from start in meters
     * @param angleToleranceRadians The angle difference in radians
     * @return A BooleanSupplier that returns true if the swerve is at the target velocity and not
     *     at the start pose
     */
    public static BooleanSupplier isAtVelocityAndNotAtStart(
            Swerve swerve,
            ChassisSpeeds targetSpeeds,
            ChassisSpeeds tolerance,
            Pose2d startPose,
            double positionToleranceMeters,
            double angleToleranceRadians) {
        return () -> {
            boolean isAtVel = isAtVelocity(swerve, targetSpeeds, tolerance).getAsBoolean();
            boolean isNotAtStart =
                    !isAt(swerve, startPose, positionToleranceMeters, angleToleranceRadians)
                            .getAsBoolean();
            return isAtVel && isNotAtStart;
        };
    }

    /**
     * Returns a supplier that checks if the robot's current field-relative velocity is within a
     * specified tolerance of a target velocity.
     *
     * @param swerve The swerve subsystem.
     * @param targetSpeeds The desired chassis speeds.
     * @param tolerance The allowed error for each component.
     * @return A boolean supplier for the check.
     */
    public static BooleanSupplier isAtVelocity(
            Swerve swerve, ChassisSpeeds targetSpeeds, ChassisSpeeds tolerance) {
        return () -> {
            ChassisSpeeds currentSpeeds = swerve.getFieldRelativeSpeeds();
            boolean isAt =
                    Math.abs(currentSpeeds.vxMetersPerSecond - targetSpeeds.vxMetersPerSecond)
                                    <= tolerance.vxMetersPerSecond
                            && Math.abs(
                                            currentSpeeds.vyMetersPerSecond
                                                    - targetSpeeds.vyMetersPerSecond)
                                    <= tolerance.vyMetersPerSecond
                            && Math.abs(
                                            currentSpeeds.omegaRadiansPerSecond
                                                    - targetSpeeds.omegaRadiansPerSecond)
                                    <= tolerance.omegaRadiansPerSecond;
            return isAt;
        };
    }

    /**
     * Creates a command that immediately stops all robot movement by setting velocities to zero.
     *
     * @param swerve The swerve subsystem.
     * @return A command to stop the robot.
     */
    public static Command stopDriving(Swerve swerve) {
        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(0.0)
                        .withRotationalDeadband(0.0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
        return swerve.runOnce(
                        () -> {
                            swerve.setControl(
                                    m_driveRequest
                                            .withVelocityX(0.0)
                                            .withVelocityY(0.0)
                                            .withRotationalRate(0.0));
                        })
                .withName("Stop Driving");
    }

    /**
     * Returns a supplier that checks if the robot is currently at a target pose within specified
     * position and angle tolerances.
     *
     * @param swerve The swerve subsystem.
     * @param targetPose The destination {@link Pose2d}.
     * @param positionToleranceMeters Allowed distance error in meters.
     * @param angleToleranceRadians Allowed rotation error in radians.
     * @return A boolean supplier for the check.
     */
    public static BooleanSupplier isAt(
            Swerve swerve,
            Pose2d targetPose,
            double positionToleranceMeters,
            double angleToleranceRadians) {
        return () -> {
            Pose2d currentPose = swerve.getState().Pose;
            if (!SubsystemConstants.disableAllLogs) {
                FieldVisualizer.getInstance().updateDrivingTarget(targetPose);
            }

            // Calculate linear distance between current and target positions.
            double positionError =
                    currentPose.getTranslation().getDistance(targetPose.getTranslation());

            // Calculate angular difference, ensuring proper wrapping.
            double currentHeading = currentPose.getRotation().getRadians();
            double targetHeading = targetPose.getRotation().getRadians();
            double rawError = targetHeading - currentHeading;
            double angleError = Math.abs(Math.atan2(Math.sin(rawError), Math.cos(rawError)));

            boolean isAt =
                    positionError <= positionToleranceMeters && angleError <= angleToleranceRadians;

            if (!SubsystemConstants.disableAllLogs) {
                Log.log("ROBOT/Commands/Swerve/IsAt/PositionError", positionError);
                Log.log("ROBOT/Commands/Swerve/IsAt/AngleError", angleError);
                Log.log("ROBOT/Commands/Swerve/IsAt/Reached Target", isAt);
            }

            return isAt;
        };
    }

    /**
     * Creates a command to move the robot to a target pose using simple PID controllers for X, Y,
     * and rotation. This is a basic implementation suitable for short, low-precision movements.
     *
     * @param swerve The swerve subsystem.
     * @param targetPose The destination {@link Pose2d}.
     * @return A command to move the robot.
     */
    @SuppressWarnings("resource")
    public static Command moveToSimple(Swerve swerve, Pose2d targetPose) {
        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(0.0)
                        .withRotationalDeadband(0.0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

        // Basic PID controllers for each axis.
        final PIDController xController = new PIDController(1, 0.2, 0.0);
        xController.setTolerance(0.0);
        final PIDController yController = new PIDController(2, 0.02, 0.0);
        yController.setTolerance(0.0);
        final PIDController thetaController = new PIDController(1, 0.01, 0.0);
        thetaController.setTolerance(0.0);
        thetaController.enableContinuousInput(Math.PI, -Math.PI);

        return swerve.run(
                () -> {
                    Pose2d currentPose = swerve.getState().Pose;
                    final double vx = xController.calculate(currentPose.getX(), targetPose.getX());
                    final double vy = yController.calculate(currentPose.getY(), targetPose.getY());
                    final double omega =
                            thetaController.calculate(
                                    MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                                    MathUtil.angleModulus(targetPose.getRotation().getRadians()));

                    if (!SubsystemConstants.disableAllLogs) {
                        Log.log("ROBOT/Commands/Swerve/MoveToSimple/VX", vx);
                        Log.log("ROBOT/Commands/Swerve/MoveToSimple/VY", vy);
                        Log.log("ROBOT/Commands/Swerve/MoveToSimple/Omega", omega);
                    }

                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(-vx)
                                    .withVelocityY(-vy)
                                    .withRotationalRate(omega));
                });
    }

    /**
     * Creates a command to move the robot to a target pose with velocity limits. Uses PID to
     * calculate desired velocities and then clamps them to the provided maximums.
     *
     * @param swerve The swerve subsystem.
     * @param targetPose The destination {@link Pose2d}.
     * @param maxVelocities A {@link Pose2d} where X/Y are max linear speeds and rotation is max
     *     angular speed.
     * @return A command to move the robot with velocity control.
     */
    @SuppressWarnings("resource")
    public static Command moveToSimpleWithVelocityControl(
            Swerve swerve, Pose2d targetPose, Pose2d maxVelocities) {
        final PIDController xController = new PIDController(.1, 0.0, 0.0);
        final PIDController yController = new PIDController(.1, 0.0, 0.0);
        final PIDController thetaController = new PIDController(.1, 0.0, 0.0);
        thetaController.enableContinuousInput(0, 2 * Math.PI);

        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(GeminiConsts.kSpeedAt12Volts.in(MetersPerSecond) * .01)
                        .withRotationalDeadband(
                                RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .01)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

        return swerve.run(
                () -> {
                    Pose2d currentPose = swerve.getState().Pose;
                    final double vx = xController.calculate(currentPose.getX(), targetPose.getX());
                    final double vy = yController.calculate(currentPose.getY(), targetPose.getY());
                    final double omega =
                            thetaController.calculate(
                                    currentPose.getRotation().getRadians(),
                                    targetPose.getRotation().getRadians());

                    // Clamp speeds to max velocities to prevent aggressive movement.
                    double clampedVx =
                            Math.max(Math.min(-vx, maxVelocities.getX()), -maxVelocities.getX());
                    double clampedVy =
                            Math.max(Math.min(-vy, maxVelocities.getY()), -maxVelocities.getY());
                    double clampedOmega =
                            Math.max(
                                    Math.min(-omega, maxVelocities.getRotation().getRadians()),
                                    -maxVelocities.getRotation().getRadians());

                    if (!SubsystemConstants.disableAllLogs) {
                        Log.log("ROBOT/Commands/Swerve/MoveToVelCtrl/ClampedVX", clampedVx);
                        Log.log("ROBOT/Commands/Swerve/MoveToVelCtrl/ClampedVY", clampedVy);
                        Log.log("ROBOT/Commands/Swerve/MoveToVelCtrl/ClampedOmega", clampedOmega);
                    }

                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(clampedVx)
                                    .withVelocityY(clampedVy)
                                    .withRotationalRate(clampedOmega));
                });
    }
}
