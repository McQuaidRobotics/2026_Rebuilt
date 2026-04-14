package igknighters.commands.teleop;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.Robot;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;

public class SlowedDownDrivingWhileShooting extends TeleopSwerveBaseCmd {

    private final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            Robot.consts
                                            .swerve()
                                            .getCommonSwerveConsts()
                                            .getMaxSpeedMetersPerSecond()
                                    * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(1);

    // changed from 1 to 2

    public SlowedDownDrivingWhileShooting(Swerve swerve, DriverController controller) {
        super(swerve, controller);
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        super.execute();
        Translation2d vt = translationStick();

        swerve.setControl(
                m_driveRequest
                        .withVelocityX(
                                xLimiter.calculate(vt.getX())
                                        * Robot.consts
                                                .swerve()
                                                .getCommonSwerveConsts()
                                                .getMaxSpeedMetersPerSecond())
                        .withVelocityY(
                                yLimiter.calculate(vt.getY())
                                        * Robot.consts
                                                .swerve()
                                                .getCommonSwerveConsts()
                                                .getMaxSpeedMetersPerSecond())
                        .withRotationalRate(rotationStick().getX() * 2));
    }
}
