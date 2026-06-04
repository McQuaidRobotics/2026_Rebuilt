package igknighters.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.constants.DrivingSharedState;
import igknighters.subsystems.swerve.Swerve;
import wayfinder.WayfinderManager;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.setpointGenerator.SwerveSetpoint;

public class BigBoyWayfinderCommands {
    public static Command driveToPositionWhileWhippingIt(Swerve swerve, Pose2d targetPose) {
        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(
                                Robot.consts
                                                .swerve()
                                                .getCommonSwerveConsts()
                                                .getMaxSpeedMetersPerSecond()
                                        * .01)
                        .withRotationalDeadband(
                                RotationsPerSecond.of(1.5).in(RadiansPerSecond) * .01)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
        return swerve.startRun(
                        () -> {
                            DrivingSharedState.getInstance().shouldDisableBumpProtection = true;
                        },
                        () -> {
                            SwerveSetpoint setpoint =
                                    WayfinderManager.calculate(
                                            .02,
                                            swerve.getState().Pose,
                                            swerve.getState().Speeds,
                                            targetPose,
                                            new ChassisConstraints(
                                                    new wayfinder.controllers.Types.Constraints(
                                                            swerve.commonSwerveConsts
                                                                    .getMaxSpeedMetersPerSecond(),
                                                            2,
                                                            4),
                                                    new wayfinder.controllers.Types.Constraints(
                                                            3, 6, 6)));

                            double vx = setpoint.fieldSpeeds().vx();
                            double vy = setpoint.fieldSpeeds().vy();
                            double omega = setpoint.fieldSpeeds().omega();

                            swerve.setControl(
                                    m_driveRequest
                                            .withVelocityX(MetersPerSecond.of(vx))
                                            .withVelocityY(MetersPerSecond.of(vy))
                                            .withRotationalRate(RadiansPerSecond.of(omega)));
                        })
                .until(SwerveCommands.isAt(swerve, targetPose, 0.1, 0.1))
                .finallyDo(
                        () -> DrivingSharedState.getInstance().shouldDisableBumpProtection = false);
    }
}
