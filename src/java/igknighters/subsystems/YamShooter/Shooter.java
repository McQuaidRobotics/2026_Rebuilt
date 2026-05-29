package igknighters.subsystems.YamShooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import igknighters.Robot;
import igknighters.constants.ShootInformation;
import igknighters.subsystems.YamShooter.flywheels.Flywheels;
import igknighters.subsystems.YamShooter.flywheels.FlywheelsFunctioning;
import igknighters.subsystems.YamShooter.hood.Hood;
import igknighters.subsystems.YamShooter.hood.HoodFunctioning;
import igknighters.subsystems.YamShooter.turret.Turret;
import igknighters.subsystems.YamShooter.turret.TurretFunctioning;

public class Shooter {
    public Hood hood;
    public Flywheels flywheels;
    public Turret turret;
    public shotType currentShotType = shotType.SHOT;
    private ShootInformation ableToShootState = ShootInformation.getInstance();
    double goalRPM = 0.0;
    double goalHoodAngle = Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES();
    double goalTurretAngle = 0.0;

    public static enum shotType {
        PASS,
        SHOT
    }

    public Shooter() {
        hood = new HoodFunctioning();
        flywheels = new FlywheelsFunctioning();
        turret = new TurretFunctioning();
    }

    public void targetState(ShooterState state) {
        targetState(state.flywheelSpeed, state.hoodAngle, state.turretAngle);
    }

    public void targetState(AngularVelocity rpm, Angle hoodAngle, Angle turretAngle) {
        goalRPM = rpm.in(RPM);
        goalHoodAngle = hoodAngle.in(Degrees);
        goalTurretAngle = turretAngle.in(Degrees);
        hood.setAngleSetpoint(hoodAngle);
        flywheels.setVelocitySetpoint(rpm);
        turret.setAngleSetpoint(turretAngle);
    }

    public ShooterState getCurrentState() {
        return new ShooterState(flywheels.getVelocity(), hood.getAngle(), turret.getAngle());
    }

    public boolean atGoal(
            AngularVelocity rpmTolerance, Angle hoodAngleTolerance, Angle turretAngleTolerance) {
        boolean atRPM = Math.abs(flywheels.getVelocity().in(RPM) - goalRPM) < rpmTolerance.in(RPM);

        boolean atHoodAngle =
                Math.abs(hood.getAngle().in(Degrees) - goalHoodAngle)
                        < hoodAngleTolerance.in(Degrees);

        boolean atTurretAngle =
                Math.abs(turret.getAngle().in(Degrees) - goalTurretAngle)
                        < turretAngleTolerance.in(Degrees);

        return atRPM && atHoodAngle && atTurretAngle;
    }

    public boolean atTarget(ShooterState state) {
        return atTarget(state.flywheelSpeed, state.hoodAngle, state.turretAngle);
    }

    public boolean atTarget(AngularVelocity rpm, Angle hoodAngle, Angle turretAngle) {
        boolean atRPM = Math.abs(flywheels.getVelocity().in(RPM) - rpm.in(RPM)) < 50;

        boolean atHoodAngle = Math.abs(hood.getAngle().in(Degrees) - hoodAngle.in(Degrees)) < 5;

        boolean atTurretAngle =
                Math.abs(turret.getAngle().in(Degrees) - turretAngle.in(Degrees)) < 5;

        return atRPM && atHoodAngle && atTurretAngle;
    }

    public shotType getShotType() {
        return currentShotType;
    }

    public boolean isHoodSensorTripped() {

        return hood.isLimitSwitchTripped();
    }

    public void setRollerVoltage(double voltage) {
        flywheels.setVoltage(voltage);
    }

    public void periodic() {
        ableToShootState.setAtTarget(atGoal(RPM.of(50), Degrees.of(2), Degrees.of(5)));
    }
}
