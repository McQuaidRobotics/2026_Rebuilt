package igknighters.subsystems.shooter;

import dev.doglog.DogLog;
import igknighters.Robot;
import igknighters.constants.Conv;
import igknighters.subsystems.Subsystems.ExclusiveSubsystem;
import igknighters.subsystems.shooter.hood.Hood;
import igknighters.subsystems.shooter.hood.HoodDisabled;
import igknighters.subsystems.shooter.hood.HoodSim;
import igknighters.subsystems.shooter.rollers.Rollers;
import igknighters.subsystems.shooter.rollers.RollersReal;
import igknighters.subsystems.shooter.rollers.RollersSim;
import igknighters.subsystems.shooter.turret.Turret;
import igknighters.subsystems.shooter.turret.TurretDisabled;
import igknighters.subsystems.shooter.turret.TurretSim;

public class Shooter implements ExclusiveSubsystem {
    private final Rollers rollers;
    private final Turret turret;
    private final Hood hood;
    private final ShooterVisualizer visualizer;
    private double goalRPM = 0.0;
    private double goalTurretAngleDegrees = 0.0;
    private double goalHoodAngleDegrees = 0.0;

    public Shooter() {
        if (Robot.isReal()) {
            rollers = new RollersReal();
            turret = new TurretDisabled();
            hood = new HoodDisabled();
        } else {
            rollers = new RollersSim();
            turret = new TurretSim();
            hood = new HoodSim();
        }
        visualizer = new ShooterVisualizer();
    }

    private void targetSpeed(double speedRPM) {
        rollers.setSpeed(speedRPM);
    }

    private void setTurretAngleDegrees(double angleDegrees) {
        turret.setAngleDegrees(angleDegrees);
    }

    private double getTurretAngleDegrees() {
        return turret.getAngleDegrees();
    }

    private void goToTurretAngleDegrees(double angleDegrees) {
        DogLog.log("Subsystems/Shooter/TARGETING", angleDegrees);
        turret.goToAngleDegrees(angleDegrees);
    }

    public void targetState(double rpm, double turretAngleDegrees, double hoodAngleDegrees) {
        DogLog.log("Subsystems/Shooter/TARGETING/RPM", rpm);
        DogLog.log("Subsystems/Shooter/TARGETING/ANGLE", turretAngleDegrees);
        targetSpeed(rpm);
        goToTurretAngleDegrees(turretAngleDegrees);
        hood.goToAngleDegrees(hoodAngleDegrees);
        goalRPM = rpm;
        goalTurretAngleDegrees = turretAngleDegrees;
        goalHoodAngleDegrees = hoodAngleDegrees;
    }

    public boolean atTarget(double tolerance) {
        boolean atSpeed = Math.abs(rollers.getSpeedRPM() - goalRPM) < tolerance;
        boolean atTurretAngle =
                Math.abs(getTurretAngleDegrees() - goalTurretAngleDegrees) < tolerance;
        boolean atHoodAngle = Math.abs(hood.getAngleDegrees() - goalHoodAngleDegrees) < tolerance;
        return atSpeed && atTurretAngle && atHoodAngle;
    }

    public void setTurretPosition(double angleDegrees) {
        DogLog.log("Subsystems/Shooter/SETSTATE/ANGLE", angleDegrees);
        turret.setAngleDegrees(angleDegrees);
    }

    public ShooterState getCurrentState() {
        return new ShooterState(
                rollers.getSpeedRPM(),
                turret.getAngleDegrees() * Conv.DEGREES_TO_RADIANS,
                hood.getAngleDegrees() * Conv.DEGREES_TO_RADIANS);
    }

    public void setRollerVoltage(double voltage) {
        rollers.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        rollers.periodic();
        turret.periodic();
        hood.periodic();

        visualizer.update(getCurrentState(), goalRPM);
    }
}
