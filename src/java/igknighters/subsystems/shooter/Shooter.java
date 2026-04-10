package igknighters.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.commands.Shooter.ShooterCommands.shotType;
import igknighters.constants.ShootInformation;
import igknighters.subsystems.shooter.flywheel.*;
import igknighters.subsystems.shooter.hood.*;
import igknighters.subsystems.shooter.turret.Turret;
import igknighters.subsystems.shooter.turret.TurretReal;
import igknighters.subsystems.shooter.turret.TurretSim;
import igknighters.util.LerpTable;
import igknighters.util.LerpTable.LerpTableEntry;
import igknighters.util.log.Log;

public class Shooter extends SubsystemBase {
    private final Flywheel rollers;
    private final Turret turret;
    private final Hood hood;
    public shotType currentShotType = shotType.SHOT;
    private Boolean beingControlled = false;
    private final ShooterVisualizer visualizer;
    private ShootInformation ableToShootState = ShootInformation.getInstance();
    private double goalRPM = 100.0;
    private double goalTurretAngleDegrees = 10.0;
    private double goalHoodAngleDegrees = 10.0;
    private LerpTable rpmTable =
            new LerpTable(
                    new LerpTableEntry[] {
                        new LerpTableEntry(1.0, 2800.0),
                        new LerpTableEntry(3.0, 3000.0),
                        new LerpTableEntry(5.0, 4000.0),
                        new LerpTableEntry(10.0, 4500.0),
                        new LerpTableEntry(15.0, 5500.0),
                        new LerpTableEntry(20.0, 6000.0),
                    });

    public Shooter() {
        if (Robot.isReal()) {
            rollers = new FlywheelReal();
            turret = new TurretReal();
            hood = new HoodReal();
        } else {
            rollers = new FlywheelSimulator();
            turret = new TurretSim();
            hood = new HoodSim();
        }
        visualizer = new ShooterVisualizer();
    }

    private void targetSpeed(AngularVelocity velo) {
        beingControlled = true;
        rollers.setSpeed(velo);
    }

    public double getHoodAngleDegrees() {
        return hood.getAngleDegrees();
    }

    public void setTurretAngleDegrees(Angle angle) {
        beingControlled = true;
        turret.setAngle(angle);
    }

    public double getTurretAngleDegrees() {
        return turret.getAngleDegrees();
    }

    private void goToTurretAngle(Angle angle) {
        beingControlled = true;
        if (!Robot.consts.shooter().kTurret().disableTurretLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/TARGETING", angle.in(Degrees));
        }
        turret.goToAngleDegrees(angle);
    }

    public void targetState(AngularVelocity velo, Angle turretAngle, Angle hoodAngle) {
        if (!Robot.consts.shooter().kTurret().disableTurretLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/TARGETING/RPM", velo.in(RPM));
            Log.log("ROBOT/Subsystems/Shooter/TARGETING/ANGLE", turretAngle.in(Degrees));
            Log.log("ROBOT/Subsystems/Shooter/TARGETING/HoodAngle", hoodAngle.in(Degrees));
        }
        beingControlled = true;
        targetSpeed(velo);
        goToTurretAngle(turretAngle);
        hood.goToAngle(hoodAngle);
        goalRPM = velo.in(RPM);
        goalTurretAngleDegrees = turret.wrapAngleDegrees(turretAngle.in(Degrees));
        goalHoodAngleDegrees = hoodAngle.in(Degrees);
    }

    public void targetState(ShooterState state) {
        targetState(state.flywheelSpeed, state.turretAngle, state.hoodAngle);
    }

    public boolean atSimTarget(
            double toleranceRPM, double toleranceDegrees, double toleranceHoodDegrees) {
        boolean atSpeed = Math.abs(rollers.getSpeed().in(RPM) - goalRPM) < toleranceRPM;
        boolean atTurretAngle =
                Math.abs(getTurretAngleDegrees() - goalTurretAngleDegrees) < toleranceDegrees;
        boolean atHoodAngle =
                Math.abs(hood.getAngleDegrees() - goalHoodAngleDegrees) < toleranceHoodDegrees;
        Log.log("ROBOT/Subsystems/Shooter/AT TARGET/AT SPEED", atSpeed);
        Log.log("ROBOT/Subsystems/Shooter/AT TARGET/AT TURRET ANGLE", atTurretAngle);
        Log.log("ROBOT/Subsystems/Shooter/AT TARGET/AT HOOD ANGLE", atHoodAngle);
        return atSpeed && atTurretAngle && atHoodAngle;
    }

    public boolean atRealTarget(
            double toleranceRPM, double toleranceDegrees, double toleranceHoodDegrees) {
        boolean atSpeed = Math.abs(rollers.getSpeed().in(RPM) - goalRPM) < toleranceRPM;
        boolean atTurretAngle =
                Math.abs(getTurretAngleDegrees() - goalTurretAngleDegrees) < toleranceDegrees;
        boolean atHoodAngle =
                Math.abs(hood.getAngleDegrees() - goalHoodAngleDegrees) < toleranceHoodDegrees;
        if (!Robot.consts.shooter().kFlywheels().disableFlywheelsLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/actual turret degrees", getTurretAngleDegrees());
            Log.log("ROBOT/Subsystems/Shooter/goal turret degrees", goalTurretAngleDegrees);
            Log.log("ROBOT/Subsystems/Shooter/AT TARGET/AT SPEED", atSpeed);
            Log.log("ROBOT/Subsystems/Shooter/AT TARGET/AT TURRET ANGLE", atTurretAngle);
            Log.log("ROBOT/Subsystems/Shooter/AT TARGET/AT HOOD ANGLE", atHoodAngle);
        }
        return atSpeed && atTurretAngle && atHoodAngle;
    }

    public void setTurretPosition(Angle angle) {
        if (!Robot.consts.shooter().kTurret().disableTurretLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/SETSTATE/ANGLE", angle.in(Degrees));
        }
        turret.setAngle(angle);
    }

    public ShooterState getCurrentState() {
        return new ShooterState(
                rollers.getSpeed(),
                Degrees.of(turret.getAngleDegrees()),
                Degrees.of(hood.getAngleDegrees()));
    }

    public void setRollerVoltage(double voltage) {
        beingControlled = true;
        rollers.setVoltage(voltage);
    }

    public double getEstimatedRPM(double distanceMeters) {
        return rpmTable.lerp(distanceMeters);
    }

    public void setHoodAngleDegrees(double angleDegrees) {
        if (!Robot.consts.shooter().kHood().disableHoodLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/SETSTATE/HoodAngle", angleDegrees);
        }
        beingControlled = true;
        hood.setAngle(angleDegrees);
    }

    public void setHoodVoltage(double voltage) {
        if (!Robot.consts.shooter().kHood().disableHoodLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/SETSTATE/HoodVoltage", voltage);
        }
        beingControlled = true;
        hood.setVoltage(voltage);
    }

    public boolean isHoodSensorHit() {
        return hood.isSensorHit();
    }

    @Override
    public void periodic() {
        rollers.periodic();
        turret.periodic();
        hood.periodic();

        if (!Robot.consts.shooter().kTurret().disableTurretLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/BEING CONTROLLED", beingControlled);
        }
        if (!Robot.isReal()) {
            visualizer.update(getCurrentState(), goalRPM, goalHoodAngleDegrees);
        }
        if (Robot.isReal()) {
            if (currentShotType == shotType.SHOT) {
                ableToShootState.setAtTarget(atRealTarget(150, 7, 2.5));
            } else {
                ableToShootState.setAtTarget(atRealTarget(800, 20, 9));
            }
        } else {
            ableToShootState.setAtTarget(atSimTarget(600, 5, 1));
        }
        beingControlled = false;
    }

    public double getHoodPosition() {

        return hood.getAngleDegrees();
    }

    public void resetHoodEncoder() {
        if (!Robot.consts.shooter().kHood().disableHoodLogs()) {
            Log.log("ROBOT/Subsystems/Shooter/Hood/ResetEncoder", true);
        }
        hood.setVoltage(0.0);
        hood.setAngle(Robot.consts.shooter().kHood().MIN_ANGLE_DEGREES());
    }
}
