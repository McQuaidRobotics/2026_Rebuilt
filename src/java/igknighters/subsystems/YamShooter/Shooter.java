package igknighters.subsystems.YamShooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.constants.ShootInformation;
import igknighters.subsystems.YamShooter.flywheels.Flywheels;
import igknighters.subsystems.YamShooter.hood.Hood;
import igknighters.subsystems.YamShooter.solvers.Math.LerpSolveShot;
import igknighters.subsystems.YamShooter.turret.Turret;

public class Shooter {
    public Hood hood;
    public Flywheels flywheels;
    public Turret turret;
    ShooterStatusIndicators status = new ShooterStatusIndicators();
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

        hood = new Hood();
        flywheels = new Flywheels();
        turret = new Turret();
    }

    public void targetState(ShooterState state) {
        targetState(state.flywheelSpeed, state.turretAngle, state.hoodAngle);
    }

    public void targetState(AngularVelocity rpm, Angle turretAngle, Angle hoodAngle) {
        goalRPM = rpm.in(RPM);
        goalHoodAngle = hoodAngle.in(Degrees);
        goalTurretAngle = turret.wrapAngle(turretAngle).in(Degrees);
        hood.setAngleSetpoint(hoodAngle);
        flywheels.setVelocitySetpoint(rpm);
        turret.setAngleSetpoint(turretAngle);
    }

    public ShooterState getCurrentState() {
        return new ShooterState(flywheels.getVelocity(), turret.getAngle(), hood.getAngle());
    }

    public boolean atGoal(
            AngularVelocity rpmTolerance, Angle hoodAngleTolerance, Angle turretAngleTolerance) {
        double rpm = flywheels.getVelocity().in(RPM);
        boolean atRPM = Math.abs(rpm - goalRPM) < rpmTolerance.in(RPM);
        double hoodAngle = hood.getAngle().in(Degrees);
        boolean atHoodAngle = Math.abs(hoodAngle - goalHoodAngle) < hoodAngleTolerance.in(Degrees);
        double turretAngle = turret.getAngle().in(Degrees);
        boolean atTurretAngle =
                Math.abs(turretAngle - goalTurretAngle) < turretAngleTolerance.in(Degrees);

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
        status.update(
                Math.abs(flywheels.getVelocity().in(RPM) - goalRPM) < 50,
                Math.abs(turret.getAngle().in(Degrees) - goalTurretAngle) < 5,
                Math.abs(hood.getAngle().in(Degrees) - goalHoodAngle) < 5);
    }

    // IDLE DEFAULT COMMANDS
    public Command idleHoodCommand(Shooter shooter) {
        return shooter.hood
                .goToAngleDontStop(Degrees.of(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES()))
                .withName("IDLE THE HOOD : DEFAULT COMMAND");
    }

    public Command idleFlywheelCommand(Shooter shooter) {
        return shooter.flywheels
                .setVelocity(RPM.of(3000))
                .withName("IDLE THE FLYWHEELS : DEFAULT COMMAND");
    }

    public Command idleTurretCommand(Shooter shooter) {

        ShootInformation info = ShootInformation.getInstance();
        return shooter.turret
                .run(
                        () -> {
                            info.setBeingControlled(false);
                            Pose3d targetPose = info.getShotLocation();

                            ShooterState targetingData =
                                    LerpSolveShot.solve(
                                            targetPose,
                                            shooter.getCurrentState().flywheelSpeed.in(RPM),
                                            0.0);

                            shooter.turret.setAngleSetpoint(targetingData.turretAngle);
                        })
                .withName("IDLING THE TURRET : DEFAULT COMMAND");
    }
}
