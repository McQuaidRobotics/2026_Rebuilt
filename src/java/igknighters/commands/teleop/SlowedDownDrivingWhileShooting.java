package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.swerveconstants.knightshadeConsts;

public class SlowedDownDrivingWhileShooting extends TeleopSwerveBaseCmd {

    private final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

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
                                vt.getX()
                                        * knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond)
                                        * .5)
                        .withVelocityY(
                                vt.getY()
                                        * knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond)
                                        * .5)
                        .withRotationalRate(
                                .4
                                        * RotationsPerSecond.of(0.75).in(RadiansPerSecond)
                                        * rotationStick().getX()));
    }
}
