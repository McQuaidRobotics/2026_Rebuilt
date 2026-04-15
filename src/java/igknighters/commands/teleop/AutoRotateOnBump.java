package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.Robot;
import igknighters.constants.DrivingSharedState;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.log.Log;

public class AutoRotateOnBump extends TeleopSwerveBaseCmd {
    private double detune;
    private final PIDController thetaController = new PIDController(4.0, 0, 0);
    private final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            Robot.consts
                                            .swerve()
                                            .getCommonSwerveConsts()
                                            .getMaxSpeedMetersPerSecond()
                                    * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public AutoRotateOnBump(Swerve swerve, DriverController controller) {
        super(swerve, controller);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    public double getBestAngleToTarget(double currentAngleDegrees) {
        double multipleOf45 = Math.floor(currentAngleDegrees / 45);

        if (multipleOf45 % 2 == 0) {
            return (multipleOf45 + 1) * 45;
        } else {
            return (multipleOf45) * 45;
        }
    }

    @Override
    public void execute() {
        super.execute();
        detune = DrivingSharedState.getInstance().detune;
        Translation2d vt = translationStick();

        double targetAngle =
                Math.toRadians(
                        getBestAngleToTarget(swerve.getState().Pose.getRotation().getDegrees()));
        double currentAngle = swerve.getState().Pose.getRotation().getRadians();

        double rotationRate = thetaController.calculate(currentAngle, targetAngle);

        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/AutoRotateOnBump/Active", true);
            Log.log("ROBOT/Commands/AutoRotateOnBump/CurrentAngle", currentAngle);
            Log.log("ROBOT/Commands/AutoRotateOnBump/TargetAngle", targetAngle);
            Log.log("ROBOT/Commands/AutoRotateOnBump/RotationRate", rotationRate);
        }

        // Force a smaller speed on the bump as requested
        double bumpSpeedMultiplier = 0.5;
        double maxSpeed =
                Robot.consts.swerve().getCommonSwerveConsts().getMaxSpeedMetersPerSecond()
                        * detune
                        * bumpSpeedMultiplier;

        swerve.setControl(
                m_driveRequest
                        .withVelocityX(vt.getX() * maxSpeed)
                        .withVelocityY(vt.getY() * maxSpeed)
                        .withRotationalRate(rotationRate));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/AutoRotateOnBump/Active", false);
        }
    }
}
